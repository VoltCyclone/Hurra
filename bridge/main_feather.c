/*
 * KMBox Feather Bridge — Minimal Latency Firmware
 *
 * Clean reimplementation for Adafruit Feather RP2350.
 * No TFT, no tracker, no Makcu. Just CDC -> Wire Protocol -> KMBox.
 *
 * Architecture:
 *   PC (km.move text) -> USB CDC -> Feather RP2350 -> UART0 (fixed-size fast protocol) -> KMBox
 *
 * Wiring:
 *   Feather GPIO 0 (UART0 TX) -> KMBox GPIO 9 (UART1 RX)
 *   Feather GPIO 1 (UART0 RX) <- KMBox GPIO 8 (UART1 TX)
 *   GND <-> GND
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/adc.h"
#include "tusb.h"

#include "config.h"
#include "hw_uart.h"
#include "latency_tracker.h"
#include "ferrum_protocol.h"
#include "ferrum_translator.h"
#include "neopixel_dma.h"
#include "fast_commands.h"
#include "kmbox_commands.h"
#include "input_classifier.h"

#include "hardware/pio.h"
#include "ws2812.pio.h"

// ============================================================================
// Connection State
// ============================================================================

typedef enum {
    BRIDGE_DISCONNECTED,
    BRIDGE_CONNECTED
} bridge_conn_state_t;

static bridge_conn_state_t bridge_conn_state = BRIDGE_DISCONNECTED;
static uint32_t kmbox_last_rx_time = 0;
static uint32_t kmbox_last_ping_time = 0;

#define KMBOX_PING_INTERVAL_MS   2000
#define KMBOX_TIMEOUT_MS         5000
#define KMBOX_INITIAL_DELAY_MS   500

// ============================================================================
// Ferrum Line Buffer
// ============================================================================

static char ferrum_line[256];
static uint8_t ferrum_idx = 0;

// Text command buffer (for native KMBox text commands)
static char cmd_buffer[128];
static uint8_t cmd_buffer_idx = 0;

// ============================================================================
// UART Helpers
// ============================================================================

static inline bool send_fast_packet(const uint8_t *data, size_t len) {
    return hw_uart_send(data, len);
}

// ============================================================================
// CDC RX Task — Parse Ferrum and KMBox text, emit fixed-size fast packets
// ============================================================================

static void handle_text_command(const char *cmd) {
    int x, y;
    if (kmbox_parse_move_command(cmd, &x, &y)) {
        input_classifier_record_move((int16_t)x, (int16_t)y);
        uint8_t pkt[FAST_CMD_PACKET_SIZE];
        size_t len = fast_build_mouse_move(pkt, (int16_t)x, (int16_t)y, 0, 0);
        send_fast_packet(pkt, len);
        return;
    }

    // Forward other text commands as-is to KMBox via UART
    hw_uart_puts(cmd);
    hw_uart_putc('\n');
}

static void cdc_rx_task(void) {
    static uint8_t response_buffer[256];
    static uint16_t response_idx = 0;
    static uint32_t last_flush_time = 0;

    if (!tud_cdc_connected() || !tud_cdc_available()) {
        return;
    }

    uint8_t buf[2048];
    uint32_t count = tud_cdc_read(buf, sizeof(buf));
    if (count == 0) return;

    for (uint32_t i = 0; i < count; i++) {
        uint8_t b = buf[i];

        // Ferrum text commands (km.* format)
        if (ferrum_idx == 0 && b == 'k') { ferrum_line[ferrum_idx++] = b; continue; }
        if (ferrum_idx == 1 && b == 'm') { ferrum_line[ferrum_idx++] = b; continue; }
        if (ferrum_idx == 2 && b == '.') { ferrum_line[ferrum_idx++] = b; continue; }

        if (ferrum_idx > 0) {
            if (b == '\r' || b == '\n') {
                ferrum_line[ferrum_idx] = '\0';

                ferrum_translated_t translated;
                if (ferrum_translate_line(ferrum_line, ferrum_idx, &translated)) {
                    if (translated.length > 0) {
                        if (translated.buffer[0] == FAST_CMD_MOUSE_MOVE &&
                            translated.length == FAST_CMD_PACKET_SIZE) {
                            int16_t mx = (int16_t)(translated.buffer[1] | (translated.buffer[2] << 8));
                            int16_t my = (int16_t)(translated.buffer[3] | (translated.buffer[4] << 8));
                            input_classifier_record_move(mx, my);
                        }
                        send_fast_packet(translated.buffer, translated.length);
                    }
                    if (translated.needs_response && tud_cdc_connected()) {
                        const char *resp = FERRUM_RESPONSE;
                        size_t resp_len = strlen(resp);
                        if (response_idx + resp_len < sizeof(response_buffer)) {
                            memcpy(&response_buffer[response_idx], resp, resp_len);
                            response_idx += resp_len;
                        }
                    }
                } else {
                    // Invalid Ferrum command, try as native text
                    ferrum_line[ferrum_idx] = '\0';
                    handle_text_command(ferrum_line);
                }
                ferrum_idx = 0;
                continue;
            }

            if (ferrum_idx < sizeof(ferrum_line) - 1) {
                ferrum_line[ferrum_idx++] = b;
            } else {
                ferrum_idx = 0;
            }
            continue;
        }

        // Regular text command handling (KMBox native: M, W, B, P, etc.)
        if (b >= 0x20 && b <= 0x7E) {
            if (cmd_buffer_idx < sizeof(cmd_buffer) - 1) {
                cmd_buffer[cmd_buffer_idx++] = b;
            }
            continue;
        }

        if (b == '\r' || b == '\n') {
            if (cmd_buffer_idx > 0) {
                cmd_buffer[cmd_buffer_idx] = '\0';
                handle_text_command(cmd_buffer);
                cmd_buffer_idx = 0;
            }
            continue;
        }
    }

    // Flush buffered responses
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
// UART RX Task — Read fixed-size responses from KMBox
// ============================================================================

static void uart_rx_task(void) {
    // Keep parser state across DMA reads; a UART packet need not arrive in one batch.
    static uint8_t packet[FAST_CMD_PACKET_SIZE];
    static uint8_t packet_idx;
    static char line[128];
    static size_t line_len;
    static uint32_t packet_started_ms;
    uint8_t batch[128];
    size_t n = hw_uart_read(batch, sizeof(batch));
    if (hw_uart_rx_take_loss()) {
        packet_idx = 0;
        line_len = 0;
    }
    if (n == 0) return;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (packet_idx && now - packet_started_ms > 100) packet_idx = 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t c = batch[i];
        if (packet_idx) {
            packet[packet_idx++] = c;
            if (packet_idx == sizeof(packet)) {
                packet_idx = 0;
                kmbox_last_rx_time = now;
                bridge_conn_state = BRIDGE_CONNECTED;
            }
            continue;
        }
        // Feather only requests ping replies. Other bytes are text, including CR/LF.
        if (c == FAST_CMD_RESPONSE || c == FAST_CMD_PING) {
            packet_idx = 1;
            packet[0] = c;
            packet_started_ms = now;
        } else if (c == '\r' || c == '\n') {
            if (line_len) {
                kmbox_last_rx_time = now;
                bridge_conn_state = BRIDGE_CONNECTED;
                if (tud_cdc_connected()) {
                    tud_cdc_write(line, line_len);
                    tud_cdc_write_str("\r\n");
                    tud_cdc_write_flush();
                }
                line_len = 0;
            }
        } else if (c >= 0x20 && c <= 0x7E && line_len < sizeof(line)) {
            line[line_len++] = (char)c;
        }
    }
}

// ============================================================================
// Connection Management
// ============================================================================

static void connection_task(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Timeout: disconnect if no RX for too long
    if (bridge_conn_state == BRIDGE_CONNECTED &&
        (now - kmbox_last_rx_time > KMBOX_TIMEOUT_MS)) {
        bridge_conn_state = BRIDGE_DISCONNECTED;
    }

    // Periodic ping
    if (now - kmbox_last_ping_time >= KMBOX_PING_INTERVAL_MS) {
        kmbox_last_ping_time = now;
        uint8_t pkt[FAST_CMD_PACKET_SIZE];
        fast_build_ping(pkt);
        send_fast_packet(pkt, sizeof(pkt));
    }

    // Evaluate input classification (~1Hz)
    {
        static uint32_t last_classify_ms = 0;
        if (now - last_classify_ms >= 1000) {
            last_classify_ms = now;
            input_class_t cls = input_classifier_evaluate();
            static input_class_t last_sent_class = INPUT_UNKNOWN;
            if (cls != INPUT_UNKNOWN && cls != last_sent_class &&
                bridge_conn_state == BRIDGE_CONNECTED) {
                uint8_t hmode = (cls == INPUT_PRE_HUMANIZED) ? 1 : 2;
                uint8_t cfg[FAST_CMD_PACKET_SIZE];
                fast_build_set_human(cfg, hmode);
                if (send_fast_packet(cfg, sizeof(cfg))) last_sent_class = cls;
                printf("[Bridge] Auto-detected input: %s -> humanization %s\n",
                       cls == INPUT_PRE_HUMANIZED ? "pre-humanized" : "raw",
                       hmode == 1 ? "MICRO" : "FULL");
            }
        }
    }
}

// ============================================================================
// Status LED
// ============================================================================

static void heartbeat_task(void) {
    static uint32_t last_toggle = 0;
    uint32_t now = time_us_32();
    if (now - last_toggle > 500000) {
        gpio_xor_mask(1u << LED_PIN);
        last_toggle = now;
    }
}

static void neopixel_status_task(void) {
    if (bridge_conn_state == BRIDGE_CONNECTED) {
        neopixel_dma_set_status(NEO_STATUS_TRACKING);  // Green = connected
    } else if (tud_cdc_connected()) {
        neopixel_dma_set_status(NEO_STATUS_CDC_CONNECTED);
    } else {
        neopixel_dma_set_status(NEO_STATUS_IDLE);
    }
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    // Overclock to 240MHz
    vreg_set_voltage(VREG_VOLTAGE_1_25);
    sleep_ms(10);
    set_sys_clock_khz(240000, true);

    stdio_init_all();
    sleep_ms(100);

    // GPIO init
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // ADC for temperature
    adc_init();
    adc_set_temp_sensor_enabled(true);

    // Peripherals
    neopixel_dma_init(pio0, WS2812_PIN);
    neopixel_dma_set_status(NEO_STATUS_BOOTING);

    // Hardware UART to KMBox
    hw_uart_init(UART_TX_PIN, UART_RX_PIN, UART_BAUD);

    // Latency tracking
    latency_tracker_init();

    // Ferrum translator
    ferrum_translator_init();

    // TinyUSB
    tusb_init();

    // Wait for KMBox to boot
    uint32_t boot_time = to_ms_since_boot(get_absolute_time());
    kmbox_last_rx_time = boot_time;
    kmbox_last_ping_time = boot_time;
    sleep_ms(KMBOX_INITIAL_DELAY_MS);

    bool startup_msg_sent = false;

    // Main loop — minimal, latency-focused
    while (1) {
        tud_task();
        cdc_rx_task();
        uart_rx_task();
        connection_task();
        heartbeat_task();
        neopixel_status_task();

        // Startup message once CDC connects
        if (tud_cdc_connected() && !startup_msg_sent) {
            sleep_ms(100);
            printf("\n=== KMBox Feather Bridge v3.0 ===\n");
            printf("Wire Protocol v2 (minimal latency)\n");
            printf("Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
            printf("UART: HW UART0 @ %d baud (DMA)\n", UART_BAUD);
            printf("  TX: GPIO%d -> KMBox RX\n", UART_TX_PIN);
            printf("  RX: GPIO%d <- KMBox TX\n", UART_RX_PIN);
            printf("Protocols: KMBox text, Ferrum text\n");
            printf("Waiting for KMBox...\n\n");
            startup_msg_sent = true;
        }

        tight_loop_contents();
    }

    return 0;
}
