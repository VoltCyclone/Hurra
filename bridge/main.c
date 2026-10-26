/**
 * KMBox Bridge - Advanced Autopilot Firmware
 * 
 * This firmware runs on an Adafruit Feather RP2350 and provides:
 * - USB CDC interface for receiving RGB frames from PC
 * - Color-based target tracking with configurable parameters
 * - Hardware UART0 for high-speed communication with RP2350 KMBox UART0
 * - NeoPixel and onboard LED status indicators
 * - High-precision latency tracking using freed PIO resources
 * 
 * Architecture:
 *   PC (capture tool) -> USB CDC -> RP2350 (this) -> UART0 (crossed) -> RP2350 KMBox UART0
 * 
 * Wiring: Bridge UART0 (GPIO27/28) <-> crossed <-> KMBox UART0 (GPIO11/12)
 * This allows direct hardware UART usage instead of PIO-based pin swapping.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/vreg.h"
#include "hardware/adc.h"
#include "tusb.h"
#include "ws2812.pio.h"

#include "config.h"
#include "tracker.h"
#include "hw_uart.h"
#include "latency_tracker.h"
#include "tft_display.h"
#include "makcu_protocol.h"
#include "makcu_translator.h"
#include "ferrum_protocol.h"
#include "ferrum_translator.h"
#include "fast_commands.h"
#include "core1_translator.h"
#include "../lib/kmbox-commands/kmbox_commands.h"

// Hardware UART is now used instead of PIO UART
// DMA circular buffers are handled internally by hw_uart module

// API Mode State (KMBox native, Makcu, or Ferrum)
// Note: api_mode_t is defined in makcu_protocol.h

static api_mode_t current_api_mode = API_MODE_KMBOX;

// Button handling - mode toggle on short press
static uint32_t last_button_check = 0;
static bool button_state = false;
static uint32_t button_press_start = 0;
static bool button_init_done = false;

// Humanization cycle request flag (set by touch handler in tft_display.c)
volatile bool humanization_cycle_requested = false;

// API mode cycle request flag (set by touch handler in tft_display.c)
volatile bool api_cycle_requested = false;

// Ferrum line buffer (file scope for mode change reset)
static char ferrum_line[256];
static uint8_t ferrum_idx = 0;

// ============================================================================
// Movement Coalescer — accumulates rapid moves, flushes at controlled rate
//
// Solves queue overflow under HUMANIZATION_FULL: each smooth_move expands
// into 4-7 queue entries, but at >200 moves/sec the 64-entry queue can't
// keep up. Coalescing sums rapid deltas and sends one FAST_CMD_SMOOTH_MOVE
// per COALESCE_INTERVAL_US, capping the KMBox-side queue rate at ~250Hz.
//
// Properties:
//   - Zero data loss (mouse deltas are purely additive)
//   - Max added latency: 4ms (matches HID frame rate)
//   - First move after idle gap: immediate (no added latency)
//   - Non-move commands (buttons, wheel, clicks, keys): sent immediately
// ============================================================================

static int32_t move_accum_x = 0, move_accum_y = 0;
static uint32_t last_move_flush_us = 0;
static bool move_pending = false;
#define COALESCE_INTERVAL_US  4000   // 4ms = 250Hz max send rate
#define COALESCE_IDLE_US      20000  // 20ms = send immediately after idle gap

// Button config already in config.h (MODE_BUTTON_PIN)
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_LONG_PRESS_MS 2000

// Protocol constants (#12: avoid magic numbers)
#define KMBOX_PACKET_SIZE 8
#define ECHO_PREFIX_LEN 5

// Frame reception state
static uint8_t frame_buffer[FRAME_BUFFER_SIZE];
static volatile bool frame_ready = false;
static volatile uint16_t current_roi_w = ROI_DEFAULT_SIZE;
static volatile uint16_t current_roi_h = ROI_DEFAULT_SIZE;

// Status tracking
typedef enum {
    STATUS_BOOTING,
    STATUS_IDLE,
    STATUS_CDC_CONNECTED,
    STATUS_TRACKING,
    STATUS_DISABLED,
    STATUS_ERROR
} bridge_status_t;

static bridge_status_t current_status = STATUS_BOOTING;
static uint32_t status_change_time = 0;

// Status message buffer from KMBox
static char status_buffer[STATUS_BUFFER_SIZE];
static uint8_t status_buffer_idx = 0;

// KMBox Connection Management
typedef enum {
    KMBOX_DISCONNECTED,
    KMBOX_CONNECTED
} kmbox_connection_state_t;

static kmbox_connection_state_t kmbox_state = KMBOX_DISCONNECTED;
static uint32_t kmbox_last_rx_time = 0;
static uint32_t kmbox_last_ping_time = 0;
static uint32_t kmbox_ping_count = 0;
static uint32_t kmbox_response_count = 0;
static uint32_t kmbox_connect_attempts = 0;

// Connection timing (milliseconds)
#define KMBOX_PING_INTERVAL_MS      2000    // Send ping every 2s (was 1s)
#define KMBOX_TIMEOUT_MS            5000    // Consider disconnected after 5s no RX (was 3s)
#define KMBOX_CONNECT_RETRY_MS      2000    // Retry connect every 2s
#define KMBOX_INITIAL_DELAY_MS      500     // Wait for KMBox to boot

// TFT display tracking variables
static uint32_t boot_time_ms = 0;
static uint32_t tft_mouse_activity_count = 0;
static uint32_t button_press_count = 0;
static uint32_t uart_error_count = 0;
static uint32_t frame_error_count = 0;

// Command rate tracking
static uint32_t last_cmd_rate_calc_ms = 0;
static uint32_t last_cmd_count = 0;
static uint32_t commands_per_sec = 0;

// UART rate tracking for TFT display
static uint32_t last_rate_calc_time_ms = 0;
static uint32_t last_tx_bytes_for_rate = 0;
static uint32_t last_rx_bytes_for_rate = 0;
static uint32_t uart_tx_rate_bps = 0;  // Calculated TX rate
static uint32_t uart_rx_rate_bps = 0;  // Calculated RX rate
static uint32_t uart_tx_peak_bps = 0;  // Peak TX rate seen
static uint32_t uart_rx_peak_bps = 0;  // Peak RX rate seen
static uint32_t injection_count = 0;   // Number of successful km.move injections

// Attached device info from KMBox
static uint16_t attached_vid = 0;
static uint16_t attached_pid = 0;
static char attached_manufacturer[32] = "";
static char attached_product[32] = "";
static uint32_t last_info_request_ms = 0;
#define INFO_REQUEST_INTERVAL_MS 5000  // Request device info every 5 seconds

// Humanization settings from KMBox
static uint8_t kmbox_humanization_mode = 0;
static uint8_t kmbox_inject_mode = 1;  // Default SMOOTH
static uint8_t kmbox_max_per_frame = 16;
static bool kmbox_velocity_matching = true;
static bool kmbox_jitter_enabled = false;
static uint8_t kmbox_queue_depth = 0;
static uint8_t kmbox_queue_capacity = 32;
static uint16_t kmbox_total_injected = 0;
static uint16_t kmbox_queue_overflows = 0;

// Console mode (Xbox passthrough) state from KMBox
static bool kmbox_console_mode = false;
static bool kmbox_console_auth_complete = false;
static uint16_t kmbox_gamepad_buttons = 0;
static int16_t kmbox_gamepad_sticks[4] = {0};
static uint16_t kmbox_gamepad_triggers[2] = {0};
static bool kmbox_humanization_valid = false;
static uint32_t last_humanization_request_ms = 0;
#define HUMANIZATION_REQUEST_INTERVAL_MS 2000  // Request humanization info every 2 seconds (was 3s)
#define HUMANIZATION_INITIAL_DELAY_MS 100      // First request 100ms after connection

// CDC receive state machine
typedef enum {
    RX_STATE_IDLE,
    RX_STATE_HEADER,
    RX_STATE_PIXELS
} rx_state_t;

static rx_state_t rx_state = RX_STATE_IDLE;
static uint32_t rx_bytes_expected = 0;
static uint32_t rx_bytes_received = 0;

// Frame header from PC
typedef struct __attribute__((packed)) {
    uint8_t magic[2];      // 'F' 'R'
    uint16_t width;
    uint16_t height;
    uint16_t reserved;
} frame_header_t;

static frame_header_t pending_header;
static uint8_t header_bytes_received = 0;

// WS2812 NeoPixel state (using SDK PIO program)
static PIO ws2812_pio = pio0;
static uint ws2812_sm = 0;
static uint ws2812_offset = 0;

// ============================================================================
// WS2812 NeoPixel Control (using SDK ws2812.pio)
// ============================================================================

static void ws2812_init(void) {
    // Add the SDK ws2812 program to PIO
    ws2812_offset = pio_add_program(ws2812_pio, &ws2812_program);
    
    // Claim a state machine
    ws2812_sm = pio_claim_unused_sm(ws2812_pio, true);
    
    // Initialize using SDK helper function (800kHz, RGB not RGBW)
    ws2812_program_init(ws2812_pio, ws2812_sm, ws2812_offset, WS2812_PIN, 800000, false);
}

void ws2812_put_rgb(uint8_t r, uint8_t g, uint8_t b) {
    // WS2812 expects GRB order, shifted left 8 bits for 24-bit mode
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    // Non-blocking: skip update if PIO FIFO is full (LED is cosmetic, not critical)
    if (!pio_sm_is_tx_fifo_full(ws2812_pio, ws2812_sm)) {
        pio_sm_put(ws2812_pio, ws2812_sm, grb << 8);
    }
}

// ============================================================================
// Hardware UART Wrappers (using hw_uart module)
// ============================================================================

// UART statistics - synced from hw_uart module periodically
// These shadow variables are used for display and rate calculation
static uint32_t uart_rx_bytes_total = 0;
static uint32_t uart_tx_bytes_total = 0;
static uint32_t uart_rx_overflows = 0;

// KMBox temperature (from 0x0D info packet)
static float kmbox_temperature_c = -999.0f;

// Sync stats from hw_uart module (called periodically)
static void sync_uart_stats(void) {
    hw_uart_get_stats(&uart_tx_bytes_total, &uart_rx_bytes_total, &uart_rx_overflows);
}

// Read internal temperature sensor (RP2040/RP2350)
// Returns temperature in Celsius
static float read_temperature_c(void) {
    // Select ADC input 4 (internal temperature sensor)
    adc_select_input(4);
    
    // Read raw ADC value (12-bit)
    uint16_t raw = adc_read();
    
    // Convert to voltage (ADC reference is 3.3V)
    const float conversion_factor = 3.3f / (1 << 12);
    float voltage = raw * conversion_factor;
    
    // Convert voltage to temperature (formula from RP2040 datasheet)
    // T = 27 - (ADC_voltage - 0.706) / 0.001721
    float temperature = 27.0f - (voltage - 0.706f) / 0.001721f;
    
    return temperature;
}

// Wrapper functions for compatibility with existing code
static inline bool uart_rx_available(void) {
    return hw_uart_rx_available();
}

static inline uint8_t uart_rx_getc(void) {
    int c = hw_uart_getc();
    if (c >= 0) {
        return (uint8_t)c;
    }
    return 0;
}

static inline void uart_tx_byte(uint8_t c) {
    hw_uart_putc(c);
}

// Initialize hardware UART for bridge communication
static void hw_uart_bridge_init(void) {
    if (!hw_uart_init(UART_TX_PIN, UART_RX_PIN, UART_BAUD)) {
        return;
    }
}

// Button handling for API mode toggle
static void button_init(void) {
    gpio_init(MODE_BUTTON_PIN);
    gpio_set_dir(MODE_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(MODE_BUTTON_PIN);
    button_init_done = true;
}

// Send packet via Hardware UART (DMA accelerated) - forward declaration
static inline bool send_uart_packet(const uint8_t* data, size_t len);

static void button_task(void) {
    if (!button_init_done) return;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_button_check < BUTTON_DEBOUNCE_MS) return;
    last_button_check = now;
    
    bool pressed = !gpio_get(MODE_BUTTON_PIN);  // Active low
    
    if (pressed && !button_state) {
        // Button just pressed — record start time
        button_state = true;
        button_press_start = now;
    } else if (!pressed && button_state) {
        // Button just released
        button_state = false;
        uint32_t held_ms = now - button_press_start;
        button_press_count++;
        
        if (held_ms < BUTTON_LONG_PRESS_MS) {
            // Short press: cycle API mode (KMBox → Makcu → Ferrum → KMBox)
            current_api_mode = (api_mode_t)((current_api_mode + 1) % 3);
            // Reset ferrum line buffer on mode change
            ferrum_idx = 0;
            ferrum_line[0] = '\0';
        } else {
            // Long press: cycle humanization mode on KMBox
            uint8_t cycle_pkt[8] = {0x0F, 0, 0, 0, 0, 0, 0, 0};
            send_uart_packet(cycle_pkt, 8);
            printf("[Bridge] Long press: sent humanization cycle command\n");
        }
    }
}

// Send packet via Hardware UART (DMA accelerated)
// Note: Don't increment uart_tx_bytes_total here - hw_uart_send() tracks via total_tx_bytes
// and we sync in tft_submit_stats_task() via hw_uart_get_stats()
static inline bool send_uart_packet(const uint8_t* data, size_t len) {
    return hw_uart_send(data, len);
}

// ============================================================================
// UART RX Processing - Status Messages from KMBox
// ============================================================================

// Echo test tracking (defined earlier in the file)
extern uint32_t echo_test_count;
extern uint32_t echo_response_count;

static void process_status_message(const char* msg, size_t len) {
    
    // Check for echo response: "ECHO:TEST..."
    if (len >= ECHO_PREFIX_LEN && strncmp(msg, "ECHO:", ECHO_PREFIX_LEN) == 0) {
        echo_response_count++;
        // Auto-connect on successful echo
        if (kmbox_state != KMBOX_CONNECTED) {
            kmbox_state = KMBOX_CONNECTED;
        }
        return;
    }
    
    // Check for handshake response
    if (len >= 11 && strncmp(msg, "KMBOX_READY", 11) == 0) {
        if (kmbox_state != KMBOX_CONNECTED) {
            kmbox_state = KMBOX_CONNECTED;
            kmbox_response_count++;
        }
        // Handshake message processed, no debug output needed
        return;
    }
    
    // Check for pong response  
    if (len >= 10 && strncmp(msg, "KMBOX_PONG", 10) == 0) {
        if (kmbox_state != KMBOX_CONNECTED) {
            kmbox_state = KMBOX_CONNECTED;
            kmbox_response_count++;
        }
        // Don't echo pong messages (too frequent)
        return;
    }
    
    // Parse device info responses from KMBox
    if (len >= 10 && strncmp(msg, "KMBOX_VID:", 10) == 0) {
        attached_vid = (uint16_t)strtol(msg + 10, NULL, 16);
        return;
    }
    if (len >= 10 && strncmp(msg, "KMBOX_PID:", 10) == 0) {
        attached_pid = (uint16_t)strtol(msg + 10, NULL, 16);
        return;
    }
    if (len >= 10 && strncmp(msg, "KMBOX_MFR:", 10) == 0) {
        strncpy(attached_manufacturer, msg + 10, sizeof(attached_manufacturer) - 1);
        attached_manufacturer[sizeof(attached_manufacturer) - 1] = '\0';
        return;
    }
    if (len >= 11 && strncmp(msg, "KMBOX_PROD:", 11) == 0) {
        strncpy(attached_product, msg + 11, sizeof(attached_product) - 1);
        attached_product[sizeof(attached_product) - 1] = '\0';
        return;
    }
    
    // Parse humanization info from KMBox: "KMBOX_INFO:hmode=2,imode=1,max=16,vel=1,qd=3,qc=32,inj=1234,ovf=0"
    if (len >= 11 && strncmp(msg, "KMBOX_INFO:", 11) == 0) {
        const char* data = msg + 11;
        int hmode = -1, imode = -1, max = -1, vel = -1;
        int qd = -1, qc = -1, inj = -1, ovf = -1;
        
        // Simple key=value parser
        const char* ptr = data;
        while (*ptr) {
            if (strncmp(ptr, "hmode=", 6) == 0) {
                hmode = atoi(ptr + 6);
            } else if (strncmp(ptr, "imode=", 6) == 0) {
                imode = atoi(ptr + 6);
            } else if (strncmp(ptr, "max=", 4) == 0) {
                max = atoi(ptr + 4);
            } else if (strncmp(ptr, "vel=", 4) == 0) {
                vel = atoi(ptr + 4);
            } else if (strncmp(ptr, "qd=", 3) == 0) {
                qd = atoi(ptr + 3);
            } else if (strncmp(ptr, "qc=", 3) == 0) {
                qc = atoi(ptr + 3);
            } else if (strncmp(ptr, "inj=", 4) == 0) {
                inj = atoi(ptr + 4);
            } else if (strncmp(ptr, "ovf=", 4) == 0) {
                ovf = atoi(ptr + 4);
            }
            // Move to next comma or end
            while (*ptr && *ptr != ',') ptr++;
            if (*ptr == ',') ptr++;
        }
        
        // Update values if valid
        if (hmode >= 0) kmbox_humanization_mode = (uint8_t)hmode;
        if (imode >= 0) kmbox_inject_mode = (uint8_t)imode;
        if (max > 0) kmbox_max_per_frame = (uint8_t)max;
        if (vel >= 0) kmbox_velocity_matching = (vel != 0);
        if (qd >= 0) kmbox_queue_depth = (uint8_t)qd;
        if (qc > 0) kmbox_queue_capacity = (uint8_t)qc;
        if (inj >= 0) kmbox_total_injected = (uint16_t)inj;
        if (ovf >= 0) kmbox_queue_overflows = (uint16_t)ovf;
        // Infer jitter_enabled from humanization mode
        kmbox_jitter_enabled = (kmbox_humanization_mode >= 1);
        kmbox_humanization_valid = true;
        return;
    }
    
    // Forward ALL other status messages to CDC
    if (tud_cdc_connected()) {
        tud_cdc_write_str("[KMBox] ");
        tud_cdc_write(msg, len);
        tud_cdc_write_str("\r\n");
        tud_cdc_write_flush();
    }
}

static void uart_rx_task(void) {
    static uint8_t binary_packet[8];
    static uint8_t binary_idx = 0;
    static bool in_binary_packet = false;
    static uint32_t binary_packet_start_time = 0;
    static uint32_t rx_debug_count = 0;
    
    // Cache timestamp ONCE at start, not per-byte (saves ~50 cycles per byte)
    const uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Batch read up to 64 bytes at once from DMA ring buffer (single memcpy)
    uint8_t rx_batch[64];
    size_t batch_count = hw_uart_read(rx_batch, sizeof(rx_batch));
    if (hw_uart_rx_take_loss()) {
        in_binary_packet = false;
        binary_idx = 0;
        status_buffer_idx = 0;
    }
    if (batch_count == 0) return;
    
    for (size_t batch_i = 0; batch_i < batch_count; batch_i++) {
        uint8_t c = rx_batch[batch_i];
        
        // Update last RX time once per batch (moved outside loop)
        kmbox_last_rx_time = now;
        
        // Reset binary packet state if timeout (stuck packet)
        if (in_binary_packet && (now - binary_packet_start_time) > 100) {
            in_binary_packet = false;
            binary_idx = 0;
        }
        
        // Check for start of binary response packet (0xFF, 0xFE, 0x0D, or 0x0E from KMBox)
        if (!in_binary_packet && (c == FAST_CMD_RESPONSE || c == FAST_CMD_PING ||
                                  (c == FAST_CMD_INFO && status_buffer_idx == 0) ||
                                  c == FAST_CMD_INFO_EXT || c == FAST_CMD_XBOX_STATUS)) {
            in_binary_packet = true;
            binary_idx = 0;
            binary_packet[binary_idx++] = c;
            binary_packet_start_time = now;
            
            // Debug: log binary packet detection
            static uint32_t last_binary_debug = 0;
            if (now - last_binary_debug > 1000) {
                printf("[Bridge RX] Binary packet start: 0x%02X\r\n", c);
                last_binary_debug = now;
            }
            continue;
        }
        
        // Continue receiving binary packet
        if (in_binary_packet) {
            binary_packet[binary_idx++] = c;
            if (binary_idx >= 8) {
                // Complete packet received
                if (binary_packet[0] == FAST_CMD_RESPONSE) {
                    // Ping/response packet
                    kmbox_response_count++;
                    
                    // Forward response packet to PC over CDC
                    if (tud_cdc_connected()) {
                        tud_cdc_write(binary_packet, 8);
                        tud_cdc_write_flush();
                    }
                    
                    if (kmbox_state != KMBOX_CONNECTED) {
                        kmbox_state = KMBOX_CONNECTED;
                    }
                } else if (binary_packet[0] == 0x0D) {
                    // Info response: [0x0D][hmode][imode][max_per_frame][queue_count][temp_lo][temp_hi][flags]
                    // flags: [0]=jitter_en [1]=vel_match [2:4]=queue_depth_3bit
                    kmbox_humanization_mode = binary_packet[1];
                    kmbox_inject_mode = binary_packet[2];
                    kmbox_max_per_frame = binary_packet[3];
                    kmbox_queue_depth = binary_packet[4];
                    kmbox_humanization_valid = true;
                    
                    // Parse temperature (int16_t in 0.1°C units)
                    int16_t temp_decideg = (int16_t)(binary_packet[5] | (binary_packet[6] << 8));
                    kmbox_temperature_c = temp_decideg / 10.0f;
                    
                    // Parse flags byte
                    uint8_t flags = binary_packet[7];
                    kmbox_jitter_enabled = (flags & 0x01) != 0;
                    kmbox_velocity_matching = (flags & 0x02) != 0;
                    
                    if (kmbox_state != KMBOX_CONNECTED) {
                        kmbox_state = KMBOX_CONNECTED;
                    }
                } else if (binary_packet[0] == 0x0E) {
                    // Extended stats: [0x0E][queue_count][queue_cap][hmode][total_lo][total_hi][ovf_lo][ovf_hi]
                    kmbox_queue_depth = binary_packet[1];
                    kmbox_queue_capacity = binary_packet[2];
                    // binary_packet[3] = hmode (redundant, but useful if 0x0D wasn't received)
                    if (!kmbox_humanization_valid) {
                        kmbox_humanization_mode = binary_packet[3];
                    }
                    kmbox_total_injected = (uint16_t)(binary_packet[4] | (binary_packet[5] << 8));
                    kmbox_queue_overflows = (uint16_t)(binary_packet[6] | (binary_packet[7] << 8));

                    if (kmbox_state != KMBOX_CONNECTED) {
                        kmbox_state = KMBOX_CONNECTED;
                    }
                } else if (binary_packet[0] == 0x28) {
                    // Xbox console status: [0x28][mode][auth][btns_lo][btns_hi][stk_lx_lo][stk_lx_hi][trigs]
                    kmbox_console_mode = (binary_packet[1] != 0);
                    kmbox_console_auth_complete = (binary_packet[2] != 0);
                    kmbox_gamepad_buttons = (uint16_t)(binary_packet[3] | (binary_packet[4] << 8));
                    kmbox_gamepad_sticks[0] = (int16_t)(binary_packet[5] | (binary_packet[6] << 8));
                    // Remaining stick/trigger data comes from extended packets
                }
                in_binary_packet = false;
                binary_idx = 0;
            }
            continue;
        }
        
        // Process as text message
        if (c == '\n' || c == '\r') {
            if (status_buffer_idx > 0) {
                status_buffer[status_buffer_idx] = '\0';
                process_status_message(status_buffer, status_buffer_idx);
                status_buffer_idx = 0;
            }
        } else if (c >= 0x20 && c < 0x7F) {  // Printable ASCII only
            if (status_buffer_idx < STATUS_BUFFER_SIZE - 1) {
                status_buffer[status_buffer_idx++] = c;
            } else {
                // Buffer overflow - reset
                status_buffer_idx = 0;
            }
        }
        // Ignore other non-printable bytes
    } // end for batch_i
}

// ============================================================================
// KMBox Connection Management
// ============================================================================

static void send_kmbox_ping(void) {
    // In KMBOX text mode, send a simple text ping instead of binary
    // This avoids corrupting the text command stream with 0xFE bytes
    if (current_api_mode == API_MODE_KMBOX) {
        // Send text ping: "P\n" - simple, won't interfere with M/W/B commands
        hw_uart_puts("P\n");
        kmbox_ping_count++;
        // Don't add to uart_tx_bytes_total - hw_uart tracks internally
    } else {
        // Binary ping for other modes
        uint8_t ping_packet[8] = {0xFE, 0, 0, 0, 0, 0, 0, 0};
        send_uart_packet(ping_packet, 8);
        kmbox_ping_count++;
    }
}

// Request device info from KMBox (VID, PID, manufacturer, product)
static void request_kmbox_info(void) {
    hw_uart_puts("KMBOX_INFO\n");
    // Don't add to uart_tx_bytes_total - hw_uart tracks internally
}

// UART Echo test - send test string and look for echo response
uint32_t echo_test_count = 0;
uint32_t echo_response_count = 0;

static void send_uart_echo_test(void) {
    echo_test_count++;
    char test_msg[32];
    snprintf(test_msg, sizeof(test_msg), "ETEST%lu\n", echo_test_count);
    
    // Use hw_uart_puts to avoid conflicting with DMA transfers
    hw_uart_puts(test_msg);
    // Don't add to uart_tx_bytes_total - hw_uart tracks internally
    
    if (tud_cdc_connected()) {
        char dbg[64];
        snprintf(dbg, sizeof(dbg), "[ECHO_TEST] Sent: %s", test_msg);
        tud_cdc_write_str(dbg);
        tud_cdc_write_flush();
    }
}

static void send_kmbox_handshake(void) {
    // Send bridge sync request - KMBox responds with "KMBOX_READY"
    hw_uart_puts("KMBOX_BRIDGE_SYNC\n");
    // Don't add to uart_tx_bytes_total - hw_uart tracks internally
}

static void kmbox_connection_task(void) {
    static uint32_t last_rx_count = 0;
    static uint32_t ping_sent_time = 0;
    static bool ping_pending = false;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Check if new data has arrived since last check
    bool got_new_data = (uart_rx_bytes_total > last_rx_count);
    if (got_new_data) {
        last_rx_count = uart_rx_bytes_total;
        ping_pending = false;  // Got response, cancel pending timeout
        
        // Transition to connected if not already
        if (kmbox_state != KMBOX_CONNECTED) {
            kmbox_state = KMBOX_CONNECTED;
            // Request info immediately on first connection (use offset to trigger soon)
            last_info_request_ms = now - INFO_REQUEST_INTERVAL_MS;
            last_humanization_request_ms = now - HUMANIZATION_REQUEST_INTERVAL_MS + HUMANIZATION_INITIAL_DELAY_MS;
            
            // Send initial info request right away (binary)
            uint8_t info_req[8] = {FAST_CMD_INFO_REQUEST, 0, 0, 0, 0, 0, 0, 0};
            send_uart_packet(info_req, 8);
        }
    }
    
    if (kmbox_state == KMBOX_CONNECTED) {
        // Send keepalive ping periodically
        if (now - kmbox_last_ping_time >= KMBOX_PING_INTERVAL_MS) {
            kmbox_last_ping_time = now;
            ping_sent_time = now;
            ping_pending = true;
            send_kmbox_ping();
        }
        
        // Request device info periodically (for TFT display)
        if (now - last_info_request_ms >= INFO_REQUEST_INTERVAL_MS) {
            last_info_request_ms = now;
            request_kmbox_info();
        }
        
        // Request humanization info periodically (for TFT display) - use binary
        if (now - last_humanization_request_ms >= HUMANIZATION_REQUEST_INTERVAL_MS) {
            last_humanization_request_ms = now;
            
            // Alternate between 0x0D (basic info + temp) and 0x0E (extended stats)
            static uint8_t info_cycle = 0;
            if (info_cycle % 2 == 0) {
                // Primary: humanization mode, inject mode, queue depth, temperature, flags
                uint8_t info_req[8] = {FAST_CMD_INFO_REQUEST, 0, 0, 0, 0, 0, 0, 0};
                send_uart_packet(info_req, 8);
            } else {
                // Extended: total injected, queue overflows, queue capacity
                uint8_t ext_req[8] = {0x0E, 0, 0, 0, 0, 0, 0, 0};
                send_uart_packet(ext_req, 8);
            }
            info_cycle++;
            
            // Debug: confirm we're sending the request
            static uint32_t last_info_debug = 0;
            if (now - last_info_debug > 5000) {
                printf("[Bridge TX] Sending info request (cycle=%d)\n", info_cycle - 1);
                last_info_debug = now;
            }
        }
        
        // Check for ping timeout (no response after sending ping)
        if (ping_pending && (now - ping_sent_time >= KMBOX_TIMEOUT_MS)) {
            kmbox_state = KMBOX_DISCONNECTED;
            ping_pending = false;
        }
    } else {
        // Not connected - try to connect periodically
        if (now - kmbox_last_ping_time >= KMBOX_CONNECT_RETRY_MS) {
            kmbox_last_ping_time = now;
            kmbox_connect_attempts++;
            
            // Alternate strategies
            if (kmbox_connect_attempts % 3 == 0) {
                send_kmbox_handshake();
            } else {
                send_kmbox_ping();
            }
        }
        
        // Send 0x0D info request even when disconnected to test UART RX
        if (now - last_humanization_request_ms >= HUMANIZATION_REQUEST_INTERVAL_MS) {
            last_humanization_request_ms = now;
            uint8_t info_req[8] = {FAST_CMD_INFO_REQUEST, 0, 0, 0, 0, 0, 0, 0};
            bool sent = send_uart_packet(info_req, 8);

            // Debug: confirm we're sending the request
            static uint32_t last_info_debug_disc = 0;
            if (now - last_info_debug_disc > 5000) {
                printf("[Bridge TX DISC] 0x0D: %02X %02X %02X %02X %02X %02X %02X %02X (sent=%d)\n",
                       info_req[0], info_req[1], info_req[2], info_req[3],
                       info_req[4], info_req[5], info_req[6], info_req[7], sent);
                last_info_debug_disc = now;
            }
        }
    }
}

static void uart_debug_task(void) {
    static uint32_t last_debug_time = 0;
    
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    if (now - last_debug_time >= 10000) {  // Every 10 seconds
        last_debug_time = now;
        if (tud_cdc_connected()) {
            const char* state_str = (kmbox_state == KMBOX_CONNECTED) ? "CONNECTED" : "DISCONNECTED";
            
            char debug_msg[128];
            snprintf(debug_msg, sizeof(debug_msg), 
                     "[Bridge] %s | TX:%lu RX:%lu\r\n", 
                     state_str, uart_tx_bytes_total, uart_rx_bytes_total);
            tud_cdc_write_str(debug_msg);
            tud_cdc_write_flush();
        }
    }
}

// ============================================================================
// Mouse Command Protocol - Send to KMBox (Binary Fast Commands)
// ============================================================================

// Coalesce a move delta into the accumulator, flushing immediately if idle
static void coalesce_move(int16_t x, int16_t y) {
    uint32_t now = time_us_32();
    move_accum_x += x;
    move_accum_y += y;

    if (!move_pending) {
        // First move — check if we've been idle long enough to send immediately
        uint32_t elapsed = now - last_move_flush_us;
        if (elapsed >= COALESCE_IDLE_US) {
            // Idle gap: flush immediately for zero-latency first move
            uint8_t pkt[FAST_CMD_PACKET_SIZE];
            fast_build_smooth_move(pkt, (int16_t)move_accum_x, (int16_t)move_accum_y,
                                   INJECT_MODE_SMOOTH);
            if (send_uart_packet(pkt, FAST_CMD_PACKET_SIZE)) {
                tft_mouse_activity_count++;
                injection_count++;
                move_accum_x = 0;
                move_accum_y = 0;
                last_move_flush_us = now;
                // move_pending stays false — data was sent
                return;
            }
            // Send failed (DMA busy) — fall through to set move_pending
            // so coalesce_flush_task retries on next main loop iteration
        }
    }
    move_pending = true;
}

// Periodic flush of coalesced moves — call from main loop
static void coalesce_flush_task(void) {
    if (!move_pending) return;

    uint32_t now = time_us_32();
    uint32_t elapsed = now - last_move_flush_us;
    if (elapsed < COALESCE_INTERVAL_US) return;

    // Flush accumulated movement as a single FAST_CMD_SMOOTH_MOVE (0x07)
    uint8_t pkt[FAST_CMD_PACKET_SIZE];
    fast_build_smooth_move(pkt, (int16_t)move_accum_x, (int16_t)move_accum_y,
                           INJECT_MODE_SMOOTH);
    if (!send_uart_packet(pkt, FAST_CMD_PACKET_SIZE)) {
        // DMA busy — don't reset accumulators, retry next iteration.
        // Update timestamp so we don't busy-spin; retry after 1ms.
        last_move_flush_us = now - COALESCE_INTERVAL_US + 1000;
        return;
    }
    tft_mouse_activity_count++;
    injection_count++;

    move_accum_x = 0;
    move_accum_y = 0;
    last_move_flush_us = now;
    move_pending = false;
}

// Send a binary mouse move command to KMBox (via coalescer for smooth humanization)
static void send_smooth_mouse_move(int16_t dx, int16_t dy) {
    // Non-blocking LED pulse using timestamp comparison
    static uint32_t led_off_time = 0;
    gpio_put(LED_PIN, 1);
    led_off_time = time_us_32() + 50;  // Schedule LED off

    // Route through coalescer → FAST_CMD_SMOOTH_MOVE (0x07)
    coalesce_move(dx, dy);

    // Non-blocking LED off (check in next iteration or turn off now if time passed)
    if (time_us_32() >= led_off_time) {
        gpio_put(LED_PIN, 0);
    }
}

// Send transform command to KMBox: "km.transform(scale_x, scale_y, enabled)\n"
// scale: 256 = 1.0x, 0 = block, -256 = invert, 128 = 0.5x
static void send_transform_command(int16_t scale_x, int16_t scale_y, bool enabled) {
    char cmd[48];
    int len = snprintf(cmd, sizeof(cmd), "km.transform(%d,%d,%d)\n", 
                       scale_x, scale_y, enabled ? 1 : 0);
    
    send_uart_packet((const uint8_t*)cmd, len);
    // Don't add to uart_tx_bytes_total - hw_uart tracks internally
    
    if (tud_cdc_connected()) {
        char dbg[64];
        snprintf(dbg, sizeof(dbg), "[UART>transform %d,%d,%d]\r\n", 
                 scale_x, scale_y, enabled ? 1 : 0);
        tud_cdc_write_str(dbg);
        tud_cdc_write_flush();
    }
}

// Legacy binary send (keep for now)
static void send_mouse_command(int16_t dx, int16_t dy, uint8_t buttons) {
    // Use binary smooth move for movement
    send_smooth_mouse_move(dx, dy);
    
    // If buttons changed, send instant button state
    if (buttons) {
        uint8_t pkt[FAST_CMD_PACKET_SIZE];
        size_t len = fast_build_mouse_move(pkt, 0, 0, buttons, 0);
        send_uart_packet(pkt, len);
    }
}

// ============================================================================
// CDC Data Reception - Binary Commands, Text Commands, and Frames
// ============================================================================

// Fast command byte detection (matches KMBox protocol)
// Note: We exclude 0x0A (\n) and 0x0D (\r) since they are line terminators
static inline bool is_fast_cmd_byte(uint8_t b) {
    return (b >= 0x01 && b <= 0x0B && b != 0x0A) || b == 0xFE;
}

// Text command buffer
static char cmd_buffer[128];
static uint8_t cmd_buffer_idx = 0;

// Makcu frame reception state
static uint8_t makcu_frame_buffer[260];  // Header + max payload
static uint16_t makcu_frame_idx = 0;
static uint16_t makcu_frame_expected = 0;
typedef enum {
    MAKCU_STATE_IDLE,
    MAKCU_STATE_HEADER,
    MAKCU_STATE_PAYLOAD
} makcu_state_t;
static makcu_state_t makcu_state = MAKCU_STATE_IDLE;

// Ferrum line buffer (moved from file scope for better organization)
// Note: ferrum_line and ferrum_idx already defined at file scope above

// Handle text commands from PC
static void handle_text_command(const char* cmd) {
    // km.move(x,y) - relative mouse movement (route through coalescer for humanization)
    int x, y;
    if (kmbox_parse_move_command(cmd, &x, &y)) {
        coalesce_move((int16_t)x, (int16_t)y);
        return;
    }
    
    // km.click(btn) - mouse click (convert to binary)
    int btn;
    if (kmbox_parse_click_command(cmd, &btn)) {
        uint8_t pkt[FAST_CMD_PACKET_SIZE];
        fast_build_mouse_click(pkt, (uint8_t)(1u << btn), 1);
        send_uart_packet(pkt, sizeof(pkt));
        return;
    }
    
    // km.version() - report version
    if (kmbox_is_version_command(cmd)) {
        if (tud_cdc_connected()) {
            tud_cdc_write_str("kmbox bridge v1.0\r\n");
            tud_cdc_write_flush();
        }
        return;
    }
    
    // ping - connection test
    if (strcmp(cmd, "ping") == 0) {
        send_kmbox_ping();
        if (tud_cdc_connected()) {
            char resp[64];
            snprintf(resp, sizeof(resp), "pong (kmbox: %s)\r\n", 
                     kmbox_state == KMBOX_CONNECTED ? "connected" : "disconnected");
            tud_cdc_write_str(resp);
            tud_cdc_write_flush();
        }
        return;
    }
    
    // stats - show statistics
    if (strcmp(cmd, "stats") == 0) {
        if (tud_cdc_connected()) {
            char resp[256];
            latency_stats_t lat_stats;
            latency_get_stats(&lat_stats);
            
            snprintf(resp, sizeof(resp), 
                     "TX: %lu bytes | RX: %lu bytes | KMBox: %s\r\n"
                     "Latency: min=%luus avg=%luus max=%luus jitter=%luus (n=%lu)\r\n",
                     uart_tx_bytes_total, uart_rx_bytes_total,
                     kmbox_state == KMBOX_CONNECTED ? "connected" : "disconnected",
                     lat_stats.min_us, lat_stats.avg_us, lat_stats.max_us,
                     lat_stats.jitter_us, lat_stats.sample_count);
            tud_cdc_write_str(resp);
            tud_cdc_write_flush();
        }
        return;
    }
    
    // Unknown command - forward km.* commands as text to KMBox
    if (strncmp(cmd, "km.", 3) == 0) {
        // Forward unknown km.* commands as text (for km.info() etc)
        char fwd[256];
        snprintf(fwd, sizeof(fwd), "%s\r\n", cmd);
        hw_uart_puts(fwd);
        // Don't send response - wait for KMBox to respond
        return;
    }
    
    // Other unknown commands
    if (tud_cdc_connected()) {
        tud_cdc_write_str("ERR: unknown command\r\n");
        tud_cdc_write_flush();
    }
}

static void process_cdc_byte(uint8_t b) {
    switch (rx_state) {
        case RX_STATE_IDLE:
            if (b == FRAME_MAGIC_0) {
                header_bytes_received = 1;
                ((uint8_t*)&pending_header)[0] = b;
                rx_state = RX_STATE_HEADER;
            }
            break;

        case RX_STATE_HEADER:
            ((uint8_t*)&pending_header)[header_bytes_received++] = b;
            if (header_bytes_received == sizeof(frame_header_t)) {
                if (pending_header.magic[1] == FRAME_MAGIC_1 &&
                    pending_header.width <= ROI_MAX_SIZE &&
                    pending_header.height <= ROI_MAX_SIZE &&
                    pending_header.width > 0 &&
                    pending_header.height > 0) {
                    
                    rx_bytes_expected = pending_header.width * pending_header.height * 3;
                    rx_bytes_received = 0;
                    rx_state = RX_STATE_PIXELS;
                } else {
                    rx_state = RX_STATE_IDLE;
                }
            }
            break;

        case RX_STATE_PIXELS:
            if (rx_bytes_received < sizeof(frame_buffer)) {
                frame_buffer[rx_bytes_received++] = b;
            }
            if (rx_bytes_received >= rx_bytes_expected) {
                current_roi_w = pending_header.width;
                current_roi_h = pending_header.height;
                frame_ready = true;
                rx_state = RX_STATE_IDLE;
            }
            break;
    }
}

static void cdc_task(void) {
    static uint8_t response_buffer[256];  // Batch responses
    static uint16_t response_idx = 0;
    static uint32_t last_flush_time = 0;
    
    if (!tud_cdc_connected() || !tud_cdc_available()) {
        return;
    }

    uint8_t buf[2048];  // Match increased CFG_TUD_CDC_RX_BUFSIZE
    uint32_t count = tud_cdc_read(buf, sizeof(buf));
    
    // Early exit if no data
    if (count == 0) {
        return;
    }
    
    // Fast-path: Check first byte for protocol hint (better branch prediction)
    uint8_t first_byte = buf[0];
    bool likely_makcu = (first_byte == MAKCU_FRAME_START);  // 0x50
    bool likely_frame = (first_byte == FRAME_MAGIC_0);      // 'F'
    
    for (uint32_t i = 0; i < count; i++) {
        uint8_t b = buf[i];
        
        // Check for Makcu frame start (0x50)
        if (makcu_state == MAKCU_STATE_IDLE && b == MAKCU_FRAME_START) {
            makcu_state = MAKCU_STATE_HEADER;
            makcu_frame_idx = 0;
            makcu_frame_buffer[makcu_frame_idx++] = b;
            continue;
        }
        
        // Handle Makcu frame reception
        if (makcu_state != MAKCU_STATE_IDLE) {
            makcu_frame_buffer[makcu_frame_idx++] = b;
            
            // Check if we have the complete header (4 bytes)
            if (makcu_state == MAKCU_STATE_HEADER && makcu_frame_idx >= 4) {
                makcu_frame_header_t* hdr = (makcu_frame_header_t*)makcu_frame_buffer;
                makcu_frame_expected = sizeof(makcu_frame_header_t) + hdr->len;
                
                // Validate payload length
                if (hdr->len > 256) {
                    // Invalid frame, reset
                    makcu_state = MAKCU_STATE_IDLE;
                    makcu_frame_idx = 0;
                    continue;
                }
                
                if (hdr->len > 0) {
                    makcu_state = MAKCU_STATE_PAYLOAD;
                } else {
                    // No payload, process immediately
                    makcu_state = MAKCU_STATE_IDLE;
                    
                    // Translate and forward
                    translated_cmd_t translated;
                    if (makcu_translate_command(hdr->cmd, NULL, 0, &translated) == TRANSLATE_OK) {
                        if (translated.length > 0) {
                            send_uart_packet(translated.buffer, translated.length);
                        }
                    }
                    
                    // Send response if needed (buffered)
                    if (makcu_cmd_needs_response(hdr->cmd)) {
                        uint8_t response[8];
                        uint16_t resp_len = makcu_build_response(hdr->cmd, MAKCU_STATUS_OK, NULL, 0, response);
                        if (tud_cdc_connected() && (response_idx + resp_len < sizeof(response_buffer))) {
                            memcpy(&response_buffer[response_idx], response, resp_len);
                            response_idx += resp_len;
                        }
                    }
                    makcu_frame_idx = 0;
                }
            }
            
            // Check if we have the complete frame
            if (makcu_state == MAKCU_STATE_PAYLOAD && makcu_frame_idx >= makcu_frame_expected) {
                makcu_frame_header_t* hdr = (makcu_frame_header_t*)makcu_frame_buffer;
                uint8_t* payload = makcu_frame_buffer + sizeof(makcu_frame_header_t);
                
                // Translate and forward
                translated_cmd_t translated;
                if (makcu_translate_command(hdr->cmd, payload, hdr->len, &translated) == TRANSLATE_OK) {
                    if (translated.length > 0) {
                        send_uart_packet(translated.buffer, translated.length);
                    }
                }
                
                // Send response if needed (buffered)
                if (makcu_cmd_needs_response(hdr->cmd)) {
                    uint8_t response[8];
                    uint16_t resp_len = makcu_build_response(hdr->cmd, MAKCU_STATUS_OK, NULL, 0, response);
                    if (tud_cdc_connected() && (response_idx + resp_len < sizeof(response_buffer))) {
                        memcpy(&response_buffer[response_idx], response, resp_len);
                        response_idx += resp_len;
                    }
                }
                
                makcu_state = MAKCU_STATE_IDLE;
                makcu_frame_idx = 0;
            }
            continue;
        }
        
        // Handle Ferrum text commands (km.* format)
        // Check if this is part of a km. command
        if (ferrum_idx == 0 && b == 'k') {
            ferrum_line[ferrum_idx++] = b;
            continue;
        }
        if (ferrum_idx == 1 && b == 'm') {
            ferrum_line[ferrum_idx++] = b;
            continue;
        }
        if (ferrum_idx == 2 && b == '.') {
            ferrum_line[ferrum_idx++] = b;
            continue;
        }
        
        // If we've started a km. command, continue collecting
        if (ferrum_idx > 0) {
            if (b == '\r' || b == '\n') {
                // End of Ferrum command
                ferrum_line[ferrum_idx] = '\0';
                
                ferrum_translated_t translated;
                if (ferrum_translate_line(ferrum_line, ferrum_idx, &translated)) {
                    if (translated.length > 0) {
                        // Intercept move packets (0x01 with non-zero x/y) → coalescer
                        if (translated.buffer[0] == FAST_CMD_MOUSE_MOVE) {
                            int16_t mx = (int16_t)(translated.buffer[1] | (translated.buffer[2] << 8));
                            int16_t my = (int16_t)(translated.buffer[3] | (translated.buffer[4] << 8));
                            if (mx != 0 || my != 0) {
                                coalesce_move(mx, my);
                            } else {
                                // Button-only or wheel-only: send immediately
                                send_uart_packet(translated.buffer, translated.length);
                            }
                        } else {
                            send_uart_packet(translated.buffer, translated.length);
                        }
                    }
                    // Send Ferrum response (buffered)
                    if (translated.needs_response && tud_cdc_connected()) {
                        const char* resp = FERRUM_RESPONSE;
                        size_t resp_len = strlen(resp);
                        if (response_idx + resp_len < sizeof(response_buffer)) {
                            memcpy(&response_buffer[response_idx], resp, resp_len);
                            response_idx += resp_len;
                        }
                    }
                } else {
                    // Invalid Ferrum command, treat as regular text
                    ferrum_line[ferrum_idx] = '\0';
                    handle_text_command(ferrum_line);
                }
                
                ferrum_idx = 0;
                continue;
            }
            
            // Continue collecting Ferrum command
            if (ferrum_idx < sizeof(ferrum_line) - 1) {
                ferrum_line[ferrum_idx++] = b;
            } else {
                // Buffer overflow, reset
                ferrum_idx = 0;
            }
            continue;
        }
        
        // Regular text command handling (for native KMBox commands)
        // Text commands (printable ASCII)
        if (b >= 0x20 && b <= 0x7E) {
            if (cmd_buffer_idx < sizeof(cmd_buffer) - 1) {
                cmd_buffer[cmd_buffer_idx++] = b;
            }
            continue;
        }
        
        // End of text command
        if (b == '\r' || b == '\n') {
            if (cmd_buffer_idx > 0) {
                cmd_buffer[cmd_buffer_idx] = '\0';
                handle_text_command(cmd_buffer);
                cmd_buffer_idx = 0;
            }
            continue;
        }
    }
    
    // Flush buffered responses if:
    // 1. Buffer has data AND (buffer is >75% full OR 5ms has passed since last flush)
    uint32_t now = time_us_32();
    bool should_flush = (response_idx > 0) && 
                       ((response_idx >= sizeof(response_buffer) * 3 / 4) || 
                        ((now - last_flush_time) >= 5000));
    
    if (should_flush && tud_cdc_connected()) {
        tud_cdc_write(response_buffer, response_idx);
        tud_cdc_write_flush();
        response_idx = 0;
        last_flush_time = now;
    }
}

// ============================================================================
// Status LED Management
// ============================================================================

static void heartbeat_task(void) {
    static uint32_t last_toggle = 0;
    uint32_t now = time_us_32();
    
    if (now - last_toggle > 500000) {
        gpio_xor_mask(1u << LED_PIN);
        last_toggle = now;
    }
}

static void set_status(bridge_status_t new_status) {
    if (current_status != new_status) {
        current_status = new_status;
        status_change_time = time_us_32();
    }
}

static void update_status_neopixel(void) {
    static uint32_t last_update = 0;
    static uint8_t activity_countdown = 0;
    static uint8_t brightness = 255;
    static int8_t brightness_dir = -2;
    uint32_t now = time_us_32();
    
    if (activity_countdown > 0) {
        ws2812_put_rgb(LED_COLOR_ACTIVITY);
        activity_countdown--;
        return;
    }
    
    if (now - last_update < 50000) return;
    last_update = now;
    
    if (!tracker_is_enabled()) {
        set_status(STATUS_DISABLED);
    } else if (frame_ready) {
        set_status(STATUS_TRACKING);
        activity_countdown = 2;
    } else if (tud_cdc_connected()) {
        set_status(STATUS_CDC_CONNECTED);
    } else {
        set_status(STATUS_IDLE);
    }
    
    bool apply_breathing = (current_status == STATUS_BOOTING || 
                           current_status == STATUS_IDLE ||
                           current_status == STATUS_ERROR);
    
    if (apply_breathing) {
        brightness += brightness_dir;
        if (brightness <= 50) {
            brightness = 50;
            brightness_dir = 2;
        } else if (brightness >= 255) {
            brightness = 255;
            brightness_dir = -2;
        }
    } else {
        brightness = 255;
    }
    
    uint8_t r, g, b;
    switch (current_status) {
        case STATUS_BOOTING:
            r = 0; g = 0; b = 255;
            break;
        case STATUS_IDLE:
            r = 255; g = 0; b = 0;
            break;
        case STATUS_CDC_CONNECTED:
            r = 0; g = 255; b = 255;
            break;
        case STATUS_TRACKING:
            r = 0; g = 255; b = 0;
            break;
        case STATUS_DISABLED:
            r = 255; g = 255; b = 0;
            break;
        case STATUS_ERROR:
        default:
            r = 255; g = 0; b = 0;
            break;
    }
    
    if (apply_breathing) {
        r = (r * brightness) / 255;
        g = (g * brightness) / 255;
        b = (b * brightness) / 255;
    }
    
    ws2812_put_rgb(r, g, b);
}

// ============================================================================
// TFT Display — Stats Gathering (runs every main-loop iteration)
//
// This is now very lightweight: it gathers numbers into a struct and hands
// them to the display module.  The CPU-intensive formatting and pixel
// drawing happen in a 100ms hardware-timer ISR inside tft_display.c,
// so the main loop is free to service USB/UART/tracking without stalling.
// ============================================================================

static void tft_submit_stats_task(void) {
    // Sync UART stats from hw_uart module (single source of truth for byte counts)
    sync_uart_stats();
    
    // Recalculate rates once per second
    static uint32_t last_rate_calc_ms = 0;
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms - last_rate_calc_ms >= 1000) {
        uint32_t tx_delta = uart_tx_bytes_total - last_tx_bytes_for_rate;
        uint32_t rx_delta = uart_rx_bytes_total - last_rx_bytes_for_rate;
        uart_tx_rate_bps = tx_delta;
        uart_rx_rate_bps = rx_delta;
        
        if (uart_tx_rate_bps > uart_tx_peak_bps) uart_tx_peak_bps = uart_tx_rate_bps;
        if (uart_rx_rate_bps > uart_rx_peak_bps) uart_rx_peak_bps = uart_rx_rate_bps;
        
        uint32_t cmd_delta = tft_mouse_activity_count - last_cmd_count;
        commands_per_sec = cmd_delta;
        last_cmd_count = tft_mouse_activity_count;
        
        last_tx_bytes_for_rate = uart_tx_bytes_total;
        last_rx_bytes_for_rate = uart_rx_bytes_total;
        last_rate_calc_ms = now_ms;
    }
    
    // Gather stats (cheap: just reading variables, no formatting)
    tft_stats_t stats = {0};
    
    stats.kmbox_connected = (kmbox_state == KMBOX_CONNECTED);
    stats.cdc_connected = tud_cdc_connected();
    stats.api_mode = (uint8_t)current_api_mode;
    
    stats.tx_bytes = uart_tx_bytes_total;
    stats.rx_bytes = uart_rx_bytes_total;
    stats.rx_buffer_level = hw_uart_rx_count();
    stats.tx_rate_bps = uart_tx_rate_bps;
    stats.rx_rate_bps = uart_rx_rate_bps;
    stats.uart_baud = UART_BAUD;
    
    stats.tx_peak_bps = uart_tx_peak_bps;
    stats.rx_peak_bps = uart_rx_peak_bps;
    
    stats.device_vid = attached_vid;
    stats.device_pid = attached_pid;
    strncpy(stats.device_product, attached_product, sizeof(stats.device_product) - 1);
    
    stats.humanization_mode = kmbox_humanization_mode;
    stats.humanization_valid = kmbox_humanization_valid;
    stats.inject_mode = kmbox_inject_mode;
    stats.max_per_frame = kmbox_max_per_frame;
    stats.queue_depth = kmbox_queue_depth;
    stats.queue_capacity = kmbox_queue_capacity;
    stats.jitter_enabled = kmbox_jitter_enabled;
    stats.velocity_matching = kmbox_velocity_matching;
    stats.total_injected = kmbox_total_injected;
    stats.queue_overflows = kmbox_queue_overflows;
    
    stats.mouse_moves = tft_mouse_activity_count;
    stats.button_presses = button_press_count;
    stats.commands_per_sec = commands_per_sec;
    
    stats.uart_errors = uart_error_count + uart_rx_overflows;
    stats.frame_errors = frame_error_count;
    
    stats.cpu_freq_mhz = clock_get_hz(clk_sys) / 1000000;
    stats.uptime_sec = (now_ms - boot_time_ms) / 1000;
    
    latency_stats_t lat_stats;
    latency_get_stats(&lat_stats);
    stats.latency_min_us = lat_stats.min_us;
    stats.latency_avg_us = lat_stats.avg_us;
    stats.latency_max_us = lat_stats.max_us;
    stats.latency_jitter_us = lat_stats.jitter_us;
    stats.latency_samples = lat_stats.sample_count;
    
    // Temperature: read only every 500ms (ADC is slow), cache result
    static uint32_t last_temp_read_ms = 0;
    static float cached_temp = -999.0f;
    if (now_ms - last_temp_read_ms >= 500) {
        cached_temp = read_temperature_c();
        last_temp_read_ms = now_ms;
    }
    stats.bridge_temperature_c = cached_temp;
    stats.kmbox_temperature_c = kmbox_temperature_c;

    // Console mode (Xbox passthrough)
    stats.console_mode = kmbox_console_mode;
    stats.console_auth_complete = kmbox_console_auth_complete;
    stats.gamepad_buttons = kmbox_gamepad_buttons;
    memcpy(stats.gamepad_sticks, kmbox_gamepad_sticks, sizeof(stats.gamepad_sticks));
    memcpy(stats.gamepad_triggers, kmbox_gamepad_triggers, sizeof(stats.gamepad_triggers));

    // Hand stats to display module (just a memcpy — timer ISR renders later)
    tft_display_submit_stats(&stats);
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main(void) {
    // Overclock RP2350 to 240MHz, increase VREG voltage to 1.20V
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);  // Let voltage stabilize
    set_sys_clock_khz(240000, true);
    
    stdio_init_all();
    sleep_ms(100);
    
    // Initialize GPIO
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
    
    // Initialize ADC for temperature sensor
    adc_init();
    adc_set_temp_sensor_enabled(true);
    
    // Initialize peripherals (fast init only)
    ws2812_init();
    button_init();
    hw_uart_bridge_init();  // Hardware UART with DMA (replaces PIO UART)
    latency_tracker_init(); // Initialize latency tracking
    
    // Initialize Core1 translator
    core1_translator_init();
    
    // Initialize tracker
    tracker_init();
    
    // Initial LED state
    ws2812_put_rgb(LED_COLOR_BOOTING);
    set_status(STATUS_BOOTING);
    
    // TinyUSB initialization - MUST happen early for CDC to enumerate
    tusb_init();
    
    // Now init TFT (has long delays, but USB is already initializing)
    tft_display_init();     // Initialize TFT display (SPI1)
    tft_display_splash();   // Show splash screen
    
    // Initialize connection state
    uint32_t boot_time = to_ms_since_boot(get_absolute_time());
    boot_time_ms = boot_time;  // Store for TFT uptime display
    kmbox_last_rx_time = boot_time;
    kmbox_last_ping_time = boot_time;
    
    // Wait for KMBox to boot
    sleep_ms(KMBOX_INITIAL_DELAY_MS);
    
    uint32_t last_frame_time = 0;
    uint32_t frame_count = 0;
    uint32_t fps_timer = 0;
    bool startup_msg_sent = false;
    
    while (1) {
        tud_task();
        cdc_task();
        uart_rx_task();
        coalesce_flush_task();
        
        kmbox_connection_task();
        button_task();
        tft_display_handle_touch();  // Handle touch screen input (zone-based: view/API/humanization)
        
        // Send humanization cycle command if requested by touch (bottom zone)
        if (humanization_cycle_requested) {
            humanization_cycle_requested = false;
            uint8_t cycle_pkt[8] = {0x0F, 0, 0, 0, 0, 0, 0, 0};
            send_uart_packet(cycle_pkt, 8);
            printf("[Bridge] Touch: sent humanization cycle command\n");
        }
        
        // Process menu requests (from TFT menu view)
        if (tft_menu_request.set_humanization_mode) {
            tft_menu_request.set_humanization_mode = false;
            uint8_t config_pkt[FAST_CMD_PACKET_SIZE];
            fast_build_set_human(config_pkt, tft_menu_request.humanization_mode_val);
            send_uart_packet(config_pkt, sizeof(config_pkt));
            printf("[Bridge] Menu: set humanization mode -> %d\n", tft_menu_request.humanization_mode_val);
        }
        if (tft_menu_request.set_max_per_frame || tft_menu_request.set_velocity_matching) {
            uint8_t mpf = tft_menu_request.set_max_per_frame ? tft_menu_request.max_per_frame_val : kmbox_max_per_frame;
            uint8_t vel = tft_menu_request.set_velocity_matching ? tft_menu_request.velocity_matching_val :
                          (kmbox_velocity_matching ? 1 : 0);
            tft_menu_request.set_max_per_frame = false;
            tft_menu_request.set_velocity_matching = false;
            uint8_t smooth_cfg_pkt[FAST_CMD_PACKET_SIZE];
            fast_build_smooth_config(smooth_cfg_pkt, mpf, vel);
            send_uart_packet(smooth_cfg_pkt, sizeof(smooth_cfg_pkt));
            printf("[Bridge] Menu: smooth cfg mpf=%d vel=%d\n", mpf, vel);
        }
        if (tft_menu_request.set_inject_mode) {
            tft_menu_request.set_inject_mode = false;
            // No dedicated wire command for inject mode — use WIRE_CONFIG with a new param
            // For now, this is informational (inject mode is per-movement in SMOOTH16)
            printf("[Bridge] Menu: inject mode -> %d (display-only)\n", tft_menu_request.inject_mode_val);
        }
        
        // Cycle API mode if requested by touch (middle zone)
        if (api_cycle_requested) {
            api_cycle_requested = false;
            current_api_mode = (api_mode_t)((current_api_mode + 1) % 3);
            ferrum_idx = 0;
            ferrum_line[0] = '\0';
            printf("[Bridge] Touch: API mode -> %d\n", (int)current_api_mode);
        }
        
        // Periodic UART echo test (every 2 seconds while not connected)
        static uint32_t last_echo_test = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (kmbox_state != KMBOX_CONNECTED && (now - last_echo_test >= 2000)) {
            last_echo_test = now;
            send_uart_echo_test();
        }
        
        // Dequeue translated commands from Core1 and send
        uint8_t translated_packet[8];
        while (core1_has_kmbox_commands()) {
            if (core1_dequeue_kmbox_command(translated_packet)) {
                // Core1 translator outputs 8-byte fast command packets
                // Intercept 0x01 move packets → coalescer for humanization
                if (translated_packet[0] == FAST_CMD_MOUSE_MOVE) {
                    int16_t mx = (int16_t)(translated_packet[1] | (translated_packet[2] << 8));
                    int16_t my = (int16_t)(translated_packet[3] | (translated_packet[4] << 8));
                    if (mx != 0 || my != 0) {
                        coalesce_move(mx, my);
                    } else {
                        // Button-only or wheel-only: send immediately
                        send_uart_packet(translated_packet, 8);
                    }
                } else {
                    send_uart_packet(translated_packet, 8);
                }
            }
        }
        
        // Send startup message once CDC is connected
        if (tud_cdc_connected() && !startup_msg_sent) {
            sleep_ms(100);
            printf("\n=== KMBox Bridge v2.1 FIFO-FIX ===\n");
            printf("Board: Adafruit Feather RP2350\n");
            printf("System clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
            printf("Peripheral clock: %lu MHz\n", clock_get_hz(clk_peri) / 1000000);
            printf("Core1: Protocol translator active\n");
            printf("UART: Hardware UART0 @ %d baud (DMA RX+TX)\n", UART_BAUD);
            printf("  TX: GPIO%d -> crossed wire -> KMBox UART0 RX\n", UART_TX_PIN);
            printf("  RX: GPIO%d <- crossed wire <- KMBox UART0 TX\n", UART_RX_PIN);
            printf("PIO: Available for timing/WS2812 (freed from UART duty)\n");
            printf("Protocols: KMBox, Makcu, Ferrum (button to toggle)\n");
            printf("Waiting for KMBox connection...\n\n");
            startup_msg_sent = true;
        }
        
        // Process frame if ready
        if (frame_ready) {
            frame_ready = false;
            
            uint32_t now = time_us_32();
            
            // Track command latency
            uint32_t timing_start = latency_start_timing();
            
            tracker_result_t result;
            tracker_process_frame(frame_buffer, current_roi_w, current_roi_h, &result);
            
            // Only send commands if KMBox is connected
            if (result.valid && tracker_is_enabled() && kmbox_state == KMBOX_CONNECTED) {
                send_mouse_command(result.dx, result.dy, 0);
            }
            
            latency_end_timing(timing_start);
            
            // FPS calculation
            frame_count++;
            if (now - fps_timer >= 1000000) {
                tracker_update_fps(frame_count);
                frame_count = 0;
                fps_timer = now;
            }
            
            last_frame_time = now;
        }
        
        // Update status indicators
        heartbeat_task();
        update_status_neopixel();
        uart_debug_task();
        tft_submit_stats_task();   // Gather stats for display (lightweight memcpy)
        tft_display_flush();       // Push rendered frame via SPI DMA (if ready)
        
        tight_loop_contents();
    }
    
    return 0;
}