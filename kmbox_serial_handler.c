/*
 * KMBox Serial Command Handler
 * 
 * Handles UART communication with RP2350 bridge for mouse/keyboard injection.
 * Supports text protocol (M<x>,<y>) and 8-byte binary protocol for low latency.
 */

#include "kmbox_serial_handler.h"
#include "lib/kmbox-commands/kmbox_commands.h"
#include "usb_hid.h"
#include "led_control.h"
#include "smooth_injection.h"
#include "xbox_gip.h"
#include "xbox_device.h"
#include "xbox_host.h"
#include "uart_buffers.h"
#include "dma_uart.h"
#include "dcp_helpers.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include <string.h>
#include <stdio.h>

//--------------------------------------------------------------------+
// Configuration (UART_RX_BUFFER_SIZE defined in uart_buffers.h)
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// Temperature Sensor (with caching for performance)
//--------------------------------------------------------------------+

static int16_t g_cached_temperature = 250;  // 25.0°C default
static uint32_t g_last_temp_read_ms = 0;
#define TEMP_CACHE_MS 1000

static inline int16_t read_temperature_decidegrees(void) {
    adc_select_input(4);
    uint16_t raw = adc_read();
    // Use DCP hardware acceleration for temperature conversion
    float temp_c = dcp_adc_to_temp(raw);
    return (int16_t)(temp_c * 10.0f);  // Return in 0.1°C units
}

static inline int16_t get_cached_temperature(uint32_t now_ms) {
    if ((now_ms - g_last_temp_read_ms) >= TEMP_CACHE_MS) {
        g_cached_temperature = read_temperature_decidegrees();
        g_last_temp_read_ms = now_ms;
    }
    return g_cached_temperature;
}

//--------------------------------------------------------------------+
// Helper Functions
//--------------------------------------------------------------------+

static __force_inline int16_t read_i16_le(const uint8_t *p) {
    return (int16_t)(p[0] | (p[1] << 8));
}

//--------------------------------------------------------------------+
// RX DMA Circular Buffer (zero-CPU reception)
// DMA writes continuously, main loop reads via write pointer comparison
//--------------------------------------------------------------------+

static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE] __attribute__((aligned(UART_RX_BUFFER_SIZE)));
static volatile uint32_t uart_rx_read_pos = 0;
static int uart_rx_dma_chan = -1;
static uint32_t uart_rx_poll_write_pos;
static bool text_discarding;
static dma_rx_state_t uart_rx_state;
static uint32_t uart_rx_reported_epoch;
static bool uart_rx_loss_pending;

//--------------------------------------------------------------------+
// Queued UART transmission (DMA when available, otherwise TX IRQ).
// Records remain owned until DMA completes; queue overflow drops whole records.
//--------------------------------------------------------------------+
// Fixed-size records preserve message boundaries across DMA/IRQ and both cores.
#define UART_TX_RECORD_SIZE 256
#define UART_TX_RECORD_COUNT 8
#define UART_TX_RECORD_MASK (UART_TX_RECORD_COUNT - 1)
typedef struct {
    uint8_t data[UART_TX_RECORD_SIZE];
    uint16_t len;
} uart_tx_record_t;
static uart_tx_record_t tx_records[UART_TX_RECORD_COUNT];
static uint8_t tx_head, tx_tail;
static uint16_t tx_offset;
static bool tx_dma_active;
static int uart_tx_dma_chan = -1;
static spin_lock_t *tx_lock;
static uint32_t g_uart_tx_dropped;

// DMA-based RX: get current write position from DMA write address
// __force_inline guarantees inlining even at -Os (Pico SDK macro)
static __force_inline uint32_t rx_get_write_pos(void) {
    if (uart_rx_dma_chan < 0) {
        // Startup and allocation-failure fallback. Reserve one ring slot so
        // a full buffer cannot masquerade as empty; leave remaining bytes in FIFO.
        while (uart_is_readable(KMBOX_UART)) {
            uint32_t next = (uart_rx_poll_write_pos + 1) & UART_RX_BUFFER_MASK;
            if (next == uart_rx_read_pos) break;
            uart_rx_buffer[uart_rx_poll_write_pos] = (uint8_t)uart_getc(KMBOX_UART);
            uart_rx_poll_write_pos = next;
        }
        return uart_rx_poll_write_pos;
    }
    uintptr_t write_addr = (uintptr_t)dma_channel_hw_addr(uart_rx_dma_chan)->write_addr;
    return (uint32_t)((write_addr - (uintptr_t)uart_rx_buffer) & UART_RX_BUFFER_MASK);
}

static __force_inline void rx_sync_state(void) {
    uart_rx_read_pos = uart_rx_state.consumed & UART_RX_BUFFER_MASK;
    if (uart_rx_reported_epoch != uart_rx_state.loss_epoch) {
        uart_rx_reported_epoch = uart_rx_state.loss_epoch;
        uart_rx_loss_pending = true;
        text_discarding = false;
    }
}

static __force_inline uint16_t rx_available(void) {
    if (uart_rx_dma_chan < 0) {
        return (uint16_t)((rx_get_write_pos() - uart_rx_read_pos) & UART_RX_BUFFER_MASK);
    }
    uint32_t avail = dma_rx_poll(&uart_rx_state, uart_rx_dma_chan);
    rx_sync_state();
    return (uint16_t)avail;
}

static __force_inline uint8_t rx_peek_at(uint16_t offset) {
    return uart_rx_buffer[(uart_rx_read_pos + offset) & UART_RX_BUFFER_MASK];
}

static bool rx_copy_peek(uint8_t *dest, size_t len) {
    if (len > rx_available()) return false;
    if (uart_rx_dma_chan >= 0) {
        bool valid = dma_rx_peek_copy(&uart_rx_state, uart_rx_dma_chan,
                                      uart_rx_buffer, dest, 0, len);
        rx_sync_state();
        return valid;
    }
    for (size_t i = 0; i < len; i++) dest[i] = rx_peek_at((uint16_t)i);
    return true;
}

static __force_inline void rx_consume(uint16_t count) {
    if (count > rx_available()) return;
    if (uart_rx_dma_chan >= 0) {
        uart_rx_state.consumed += count;
        rx_sync_state();
    } else {
        uart_rx_read_pos = (uart_rx_read_pos + count) & UART_RX_BUFFER_MASK;
    }
}

//--------------------------------------------------------------------+
// Forward declarations for the shared TX queue.
//--------------------------------------------------------------------+
static void on_uart_irq(void);
static void tx_service_locked(void);

//--------------------------------------------------------------------+
// Connection State
//--------------------------------------------------------------------+

static bridge_connection_state_t g_connection_state = BRIDGE_STATE_WAITING;
static uint32_t g_last_data_time_ms = 0;
static uint32_t g_last_heartbeat_check_ms = 0;
static uint32_t g_last_ping_time_ms = 0;

//--------------------------------------------------------------------+
// Clock Sync State (for timed commands)
//--------------------------------------------------------------------+

static struct {
    uint64_t device_base_us;
    uint32_t pc_base_us;
    uint8_t  seq_num;
    bool     synced;
} clock_sync = {0};

//--------------------------------------------------------------------+
// Timed Move (hardware alarm — microsecond-accurate, zero polling)
//--------------------------------------------------------------------+

// Volatile struct for alarm callback to inject movement at exact time
static volatile struct {
    int16_t x, y;
    uint8_t mode;
    volatile bool pending;  // Set by alarm callback, cleared by main loop check
    volatile bool fired;    // Alarm fired, needs injection in main context
} pending_timed_move = {0};

static alarm_id_t g_timed_move_alarm_id = -1;

// Hardware alarm callback — fires at exact microsecond target time.
// Runs in IRQ context, so we just flag it for main loop injection.
// Returns 0 = don't reschedule.
static int64_t timed_move_alarm_callback(alarm_id_t id, void *user_data) {
    (void)id;
    (void)user_data;
    pending_timed_move.fired = true;
    g_timed_move_alarm_id = -1;
    return 0;  // Don't reschedule
}

// Process fired timed move in main loop context (safe for smooth_inject_movement)
static inline void process_pending_timed_move(void) {
    if (pending_timed_move.fired) {
        pending_timed_move.fired = false;
        pending_timed_move.pending = false;
        inject_mode_t mode = (pending_timed_move.mode <= 3) 
            ? (inject_mode_t)pending_timed_move.mode : INJECT_MODE_SMOOTH;
        smooth_inject_movement(pending_timed_move.x, pending_timed_move.y, mode);
    }
}

//--------------------------------------------------------------------+
// Fast Binary Command Stats
//--------------------------------------------------------------------+

static uint32_t fast_cmd_count = 0;
static uint32_t fast_cmd_errors = 0;

//--------------------------------------------------------------------+
// TX queue service. Call with tx_lock held; never wait for the peripheral.
//--------------------------------------------------------------------+
static void tx_service_locked(void) {
    if (uart_tx_dma_chan < 0) {
        uart_set_irq_enables(KMBOX_UART, false, tx_head != tx_tail);
        return;
    }
    if (tx_dma_active) {
        if (dma_channel_is_busy(uart_tx_dma_chan)) return;
        tx_dma_active = false;
        tx_tail = (tx_tail + 1) & UART_TX_RECORD_MASK;
        tx_offset = 0;
    }
    if (tx_head == tx_tail) return;
    uart_tx_record_t *record = &tx_records[tx_tail];
    dma_channel_set_read_addr(uart_tx_dma_chan, record->data + tx_offset, false);
    tx_dma_active = true;
    __dmb();  // Publish slot contents to the DMA bus master before triggering.
    dma_channel_set_trans_count(uart_tx_dma_chan, record->len - tx_offset, true);
}

static void tx_service(void) {
    if (!tx_lock) return;
    uint32_t save = spin_lock_blocking(tx_lock);
    tx_service_locked();
    spin_unlock(tx_lock, save);
}

static void __not_in_flash_func(on_uart_irq)(void) {
    uint32_t save = spin_lock_blocking(tx_lock);
    if (uart_tx_dma_chan < 0) {
        while (tx_head != tx_tail && uart_is_writable(KMBOX_UART)) {
            uart_tx_record_t *record = &tx_records[tx_tail];
            uart_putc_raw(KMBOX_UART, record->data[tx_offset++]);
            if (tx_offset == record->len) {
                tx_offset = 0;
                tx_tail = (tx_tail + 1) & UART_TX_RECORD_MASK;
            }
        }
        uart_set_irq_enables(KMBOX_UART, false, tx_head != tx_tail);
    }
    spin_unlock(tx_lock, save);
}

// Enqueue all bytes or none. CRLF is part of the same record as its body.
static bool uart_enqueue_record(const uint8_t *data, size_t len, bool crlf) {
    size_t suffix = crlf ? 2u : 0u;
    if (!tx_lock) return false;
    uint32_t save = spin_lock_blocking(tx_lock);
    if (!data || len > UART_TX_RECORD_SIZE - suffix) {
        g_uart_tx_dropped += (uint32_t)len + (uint32_t)suffix;
        spin_unlock(tx_lock, save);
        return false;
    }
    if (len + suffix == 0) {
        spin_unlock(tx_lock, save);
        return true;
    }
    // Reclaim a completed DMA record before deciding whether the queue is full.
    tx_service_locked();
    uint8_t next = (tx_head + 1) & UART_TX_RECORD_MASK;
    if (next == tx_tail) {
        g_uart_tx_dropped += (uint32_t)len + (uint32_t)suffix;
        spin_unlock(tx_lock, save);
        return false;
    }
    uart_tx_record_t *record = &tx_records[tx_head];
    memcpy(record->data, data, len);
    if (crlf) {
        record->data[len] = '\r';
        record->data[len + 1] = '\n';
    }
    record->len = (uint16_t)(len + suffix);
    tx_head = next;
    tx_service_locked();
    spin_unlock(tx_lock, save);
    return true;
}

static bool uart_send_bytes(const uint8_t *data, size_t len) {
    if (len == 0) return true;
    return uart_enqueue_record(data, len, false);
}

static bool command_reply_write(const char *data, size_t len) {
    return uart_send_bytes((const uint8_t *)data, len);
}

//--------------------------------------------------------------------+
// Public TX API (Non-blocking)
//--------------------------------------------------------------------+

void kmbox_send_response(const char *response) {
    if (!response) return;
    uart_enqueue_record((const uint8_t *)response, strlen(response), true);
}

void kmbox_send_status(const char *message) {
    if (message) kmbox_send_response(message);
}

void kmbox_send_ping_to_bridge(void) {
    uint8_t ping[8] = {FAST_CMD_PING, 0, 0, 0, 0, 0, 0, 0};
    uart_send_bytes(ping, 8);
    // Non-blocking: TX IRQ handles actual transmission
}

// Get TX buffer stats (for debugging)
uint32_t kmbox_get_tx_dropped_bytes(void) {
    if (!tx_lock) return 0;
    uint32_t save = spin_lock_blocking(tx_lock);
    uint32_t dropped = g_uart_tx_dropped;
    spin_unlock(tx_lock, save);
    return dropped;
}

//--------------------------------------------------------------------+
// Connection State Management
//--------------------------------------------------------------------+

bridge_connection_state_t kmbox_get_connection_state(void) {
    return g_connection_state;
}

bool kmbox_is_connected(void) {
    return (g_connection_state == BRIDGE_STATE_CONNECTED || 
            g_connection_state == BRIDGE_STATE_ACTIVE);
}

static void set_connection_state(bridge_connection_state_t new_state) {
    if (g_connection_state == new_state) return;
    g_connection_state = new_state;
    
    switch (new_state) {
        case BRIDGE_STATE_WAITING:     neopixel_set_status_override(STATUS_BRIDGE_WAITING); break;
        case BRIDGE_STATE_CONNECTING:  neopixel_set_status_override(STATUS_BRIDGE_CONNECTING); break;
        case BRIDGE_STATE_CONNECTED:   neopixel_set_status_override(STATUS_BRIDGE_CONNECTED); break;
        case BRIDGE_STATE_ACTIVE:      neopixel_set_status_override(STATUS_BRIDGE_ACTIVE); break;
        case BRIDGE_STATE_DISCONNECTED: neopixel_set_status_override(STATUS_BRIDGE_DISCONNECTED); break;
    }
}

static void check_connection_timeout(uint32_t now_ms) {
    if (now_ms - g_last_heartbeat_check_ms < BRIDGE_HEARTBEAT_CHECK_MS) return;
    g_last_heartbeat_check_ms = now_ms;
    
    if (kmbox_is_connected() && (now_ms - g_last_data_time_ms > BRIDGE_HEARTBEAT_TIMEOUT_MS)) {
        set_connection_state(BRIDGE_STATE_DISCONNECTED);
    }
    
    if (g_connection_state == BRIDGE_STATE_DISCONNECTED &&
        (now_ms - g_last_data_time_ms > BRIDGE_HEARTBEAT_TIMEOUT_MS * 2)) {
        set_connection_state(BRIDGE_STATE_WAITING);
    }
}

static __force_inline void mark_activity(uint32_t now_ms) {
    g_last_data_time_ms = now_ms;
    if (__builtin_expect(!kmbox_is_connected(), 0)) set_connection_state(BRIDGE_STATE_CONNECTED);
    if (__builtin_expect(g_connection_state == BRIDGE_STATE_CONNECTED, 0)) set_connection_state(BRIDGE_STATE_ACTIVE);
}

//--------------------------------------------------------------------+
// Bridge Protocol Parser (variable-length binary, sync byte 0xBD)
//--------------------------------------------------------------------+

static uint8_t __not_in_flash_func(process_bridge_packet)(const uint8_t *data, size_t available) {
    if (available < 2) return 0;
    if (data[0] != BRIDGE_SYNC_BYTE) return 1;  // Skip invalid sync
    
    switch (data[1]) {
        case BRIDGE_CMD_MOUSE_MOVE:
            if (available < 6) return 0;
            kmbox_add_mouse_movement(read_i16_le(data + 2), read_i16_le(data + 4));
            neopixel_signal_activity(COLOR_BRIDGE_ACTIVE);
            return 6;
            
        case BRIDGE_CMD_MOUSE_WHEEL:
            if (available < 3) return 0;
            kmbox_add_wheel_movement((int8_t)data[2]);
            return 3;
            
        case BRIDGE_CMD_BUTTON_SET:
            if (available < 4) return 0;
            kmbox_update_physical_buttons(data[3] ? data[2] : 0);
            return 4;
            
        case BRIDGE_CMD_MOUSE_MOVE_WHEEL:
            if (available < 7) return 0;
            kmbox_add_mouse_movement(read_i16_le(data + 2), read_i16_le(data + 4));
            kmbox_add_wheel_movement((int8_t)data[6]);
            neopixel_signal_activity(COLOR_BRIDGE_ACTIVE);
            return 7;
            
        case BRIDGE_CMD_PING:
            return 2;
            
        case BRIDGE_CMD_RESET:
            kmbox_add_mouse_movement(0, 0);
            kmbox_add_wheel_movement(0);
            kmbox_update_physical_buttons(0);
            return 2;
            
        default:
            return 1;  // Unknown command, skip sync byte
    }
}

//--------------------------------------------------------------------+
// Fast Binary Command Processing (8-byte fixed packets)
//--------------------------------------------------------------------+

extern void process_mouse_report(const hid_mouse_report_t *report);
extern void process_kbd_report(const hid_keyboard_report_t *report);

// 256-bit bitmap: O(1) single-load lookup replaces chained range comparisons.
// Bit N is set if byte value N is a valid fast command start byte.
// Excludes 0x0A (\n) and 0x0D (\r) to avoid text protocol conflict.
static const uint32_t g_fast_cmd_bitmap[8] = {
    // Bits 0-31:   commands 0x01-0x0F (excluding 0x0A, 0x0D)
    //   0x01 MOUSE_MOVE, 0x02 MOUSE_CLICK, 0x07 SMOOTH_MOVE, 0x08 SMOOTH_CONFIG,
    //   0x09 SMOOTH_CLEAR, 0x0B MULTI_MOVE, 0x0C KEY_COMBO, 0x0E INFO_EXT, 0x0F CYCLE_HUMAN
    (1u << 0x01) | (1u << 0x02) | (1u << 0x07) | (1u << 0x08) |
    (1u << 0x09) | (1u << 0x0B) | (1u << 0x0C) | (1u << 0x0E) | (1u << 0x0F) |
    (1u << FAST_CMD_SET_HUMAN) | (1u << FAST_CMD_INFO_REQUEST),
    // Bits 32-63:  Xbox commands 0x20-0x28
    (1u << (0x20 - 32)) | (1u << (0x22 - 32)) | (1u << (0x23 - 32)) |
    (1u << (0x27 - 32)) | (1u << (0x28 - 32)),
    0, 0, 0, 0, 0,  // Bits 64-223: none
    // Bits 224-255: 0xFC SYNC, 0xFE PING
    (1u << (0xFC - 224)) | (1u << (0xFE - 224)),
};

static __force_inline bool is_fast_cmd_start(uint8_t byte) {
    return (g_fast_cmd_bitmap[byte >> 5] >> (byte & 31)) & 1;
}

static bool __not_in_flash_func(process_fast_command)(const uint8_t *pkt) {
    // Hint: mouse move is the most common command by far (~95% of traffic)
    // Direct accumulator path bypasses process_mouse_report() overhead:
    // - Skips transform (bridge moves are pre-transformed)
    // - Skips smooth_record_physical_movement (only for actual HW mouse)
    // - Skips neopixel flash (too expensive for high-rate moves)
    if (__builtin_expect(pkt[0] == FAST_CMD_MOUSE_MOVE, 1)) {
        const fast_cmd_move_t *m = (const fast_cmd_move_t *)pkt;
        kmbox_set_injected_buttons(m->buttons);
        if (m->x || m->y) kmbox_add_mouse_movement(m->x, m->y);
        if (m->wheel) kmbox_add_wheel_movement(m->wheel);
        // Passive LED activity — single volatile store, DMA pushes to PIO at ~30 Hz
        neopixel_signal_activity(COLOR_BRIDGE_ACTIVE);
        fast_cmd_count++;
        return true;
    }
    
    switch (pkt[0]) {
        
        case FAST_CMD_MOUSE_CLICK: {
            const fast_cmd_click_t *c = (const fast_cmd_click_t *)pkt;
            // The fast protocol carries a HID button mask, not a button number.
            // FAST_BTN: 1=left, 2=right, 4=middle, 8=side1, 16=side2
            kmbox_button_t btn;
            switch (c->button) {
                case 0: case 1: btn = KMBOX_BUTTON_LEFT;   break;
                case 2:         btn = KMBOX_BUTTON_RIGHT;  break;
                case 4:         btn = KMBOX_BUTTON_MIDDLE; break;
                case 8:         btn = KMBOX_BUTTON_SIDE1;  break;
                case 16:        btn = KMBOX_BUTTON_SIDE2;  break;
                default:        return false;
            }
            // Use timed click mechanism — press/hold/release with randomized
            // timing, handled by kmbox_update_states() on each main loop tick.
            // Only single-click is supported; the count field is ignored for now
            // because multi-click requires waiting for each click to complete.
            uint32_t click_now = to_ms_since_boot(get_absolute_time());
            kmbox_start_button_click(btn, click_now);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_KEY_COMBO: {
            // Handles both KEY_PRESS and KEY_COMBO (same command ID 0x0C)
            const fast_cmd_combo_t *c = (const fast_cmd_combo_t *)pkt;
            hid_keyboard_report_t report = {.modifier = c->modifiers};
            uint8_t n = 0;
            // Use keycode field for single key, or keys array for combo
            if (c->keycode) report.keycode[n++] = c->keycode;
            for (int i = 0; i < 4 && c->keys[i] && n < 6; i++) report.keycode[n++] = c->keys[i];
            process_kbd_report(&report);
            memset(&report, 0, sizeof(report));
            process_kbd_report(&report);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_MULTI_MOVE: {
            // Direct accumulator path — same as FAST_CMD_MOUSE_MOVE.
            // Bridge moves are pre-transformed, so skip transform/velocity tracking
            // that process_mouse_report() would apply.  Sum all 3 sub-moves into
            // a single kmbox_add_mouse_movement() call (1 spinlock instead of 3).
            const fast_cmd_multi_t *m = (const fast_cmd_multi_t *)pkt;
            int16_t sum_x = m->x1 + m->x2 + m->x3;
            int16_t sum_y = m->y1 + m->y2 + m->y3;
            if (sum_x || sum_y) kmbox_add_mouse_movement(sum_x, sum_y);
            neopixel_signal_activity(COLOR_BRIDGE_ACTIVE);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SMOOTH_MOVE: {
            const fast_cmd_smooth_t *s = (const fast_cmd_smooth_t *)pkt;
            inject_mode_t mode = (s->mode <= 3) ? (inject_mode_t)s->mode : INJECT_MODE_SMOOTH;
            smooth_inject_movement(s->x, s->y, mode);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SMOOTH_CONFIG: {
            const fast_cmd_config_t *c = (const fast_cmd_config_t *)pkt;
            if (c->max_per_frame > 0) smooth_set_max_per_frame(c->max_per_frame);
            smooth_set_velocity_matching(c->vel_match != 0);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SMOOTH_CLEAR:
            smooth_clear_queue();
            fast_cmd_count++;
            return true;
        
        case FAST_CMD_TIMED_MOVE: {
            const fast_cmd_timed_t *t = (const fast_cmd_timed_t *)pkt;
            if (clock_sync.synced && t->time_us > 0) {
                uint64_t target = clock_sync.device_base_us + t->time_us;
                uint64_t now = time_us_64();
                if (target > now && (target - now) < 100000) {
                    // Cancel any previous pending alarm
                    if (g_timed_move_alarm_id >= 0) {
                        cancel_alarm(g_timed_move_alarm_id);
                        g_timed_move_alarm_id = -1;
                    }
                    // Store movement params for alarm callback
                    pending_timed_move.x = t->x;
                    pending_timed_move.y = t->y;
                    pending_timed_move.mode = t->mode;
                    pending_timed_move.pending = true;
                    pending_timed_move.fired = false;
                    // Schedule hardware alarm at exact target time (µs precision)
                    uint64_t delay_us = target - now;
                    g_timed_move_alarm_id = add_alarm_in_us(delay_us, timed_move_alarm_callback, NULL, true);
                    fast_cmd_count++;
                    return true;
                }
            }
            inject_mode_t mode = (t->mode <= 3) ? (inject_mode_t)t->mode : INJECT_MODE_SMOOTH;
            smooth_inject_movement(t->x, t->y, mode);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_SYNC: {
            const fast_cmd_sync_t *s = (const fast_cmd_sync_t *)pkt;
            clock_sync.device_base_us = time_us_64();
            clock_sync.pc_base_us = s->timestamp;
            clock_sync.seq_num = s->seq_num;
            clock_sync.synced = true;
            
            uint8_t resp[8] = {FAST_CMD_RESPONSE, 0x01, 0, 0, 0, 0, 0, 0};
            uint32_t ts = (uint32_t)(clock_sync.device_base_us & 0xFFFFFFFF);
            memcpy(resp + 4, &ts, 4);
            uart_send_bytes(resp, 8);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_PING: {
            uint8_t resp[8] = {FAST_CMD_RESPONSE, 0x00, 0, 0, 0, 0, 0, 0};
            uint32_t ts = (uint32_t)(time_us_64() & 0xFFFFFFFF);
            memcpy(resp + 4, &ts, 4);
            uart_send_bytes(resp, 8);
            return true;
        }
        
        case FAST_CMD_SET_HUMAN:
            if (pkt[1] < HUMANIZATION_MODE_COUNT) {
                smooth_set_humanization_mode((humanization_mode_t)pkt[1]);
            }
            return true;

        case FAST_CMD_INFO_REQUEST:
        case FAST_CMD_INFO: {
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            int16_t temp = get_cached_temperature(now_ms);
            // Pack extra flags into byte 7 bitfield:
            // [0]=jitter_en [1]=vel_match [2:4]=queue_depth_3bit [5:7]=reserved
            uint8_t queue_count = 0;
            smooth_get_stats(NULL, NULL, NULL, &queue_count);
            uint8_t flags = 0;
            humanization_mode_t hm = smooth_get_humanization_mode();
            // Jitter is enabled for LOW/MEDIUM/HIGH modes
            if (hm >= HUMANIZATION_MICRO) flags |= 0x01;
            if (smooth_get_velocity_matching()) flags |= 0x02;
            // Queue depth as 3-bit value (0-7, clamped from 0-32)
            uint8_t q3 = (queue_count > 28) ? 7 : (queue_count / 4);
            flags |= (q3 & 0x07) << 2;
            
            uint8_t resp[8] = {
                FAST_CMD_INFO,
                (uint8_t)hm,
                (uint8_t)smooth_get_inject_mode(),
                (uint8_t)smooth_get_max_per_frame(),
                queue_count,
                (uint8_t)(temp & 0xFF),
                (uint8_t)((temp >> 8) & 0xFF),
                flags
            };
            uart_send_bytes(resp, 8);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_INFO_EXT: {
            // Extended stats packet:
            // [0x0E][queue_count][queue_cap][overshoot%][total_inj_lo][total_inj_hi][overflows_lo][overflows_hi]
            uint32_t total_injected = 0, frames_processed = 0, queue_overflows = 0;
            uint8_t queue_count = 0;
            smooth_get_stats(&total_injected, &frames_processed, &queue_overflows, &queue_count);
            
            uint8_t resp[8] = {
                FAST_CMD_INFO_EXT,
                queue_count,
                SMOOTH_QUEUE_SIZE,  // queue capacity (compile-time constant)
                (uint8_t)smooth_get_humanization_mode(),  // redundant but useful standalone
                (uint8_t)(total_injected & 0xFF),
                (uint8_t)((total_injected >> 8) & 0xFF),
                (uint8_t)(queue_overflows & 0xFF),
                (uint8_t)((queue_overflows >> 8) & 0xFF)
            };
            uart_send_bytes(resp, 8);
            fast_cmd_count++;
            return true;
        }
        
        case FAST_CMD_CYCLE_HUMAN: {
            // Cycle humanization mode (triggered by bridge button/touch)
            humanization_mode_t new_mode = smooth_cycle_humanization_mode();
            
            // Show mode with LED flash (same as local button press)
            uint32_t mode_color;
            switch (new_mode) {
                case HUMANIZATION_OFF:    mode_color = COLOR_HUMANIZATION_OFF;    break;
                case HUMANIZATION_MICRO:  mode_color = COLOR_HUMANIZATION_MICRO;  break;
                case HUMANIZATION_FULL:   mode_color = COLOR_HUMANIZATION_FULL;   break;
                default:                  mode_color = COLOR_ERROR;               break;
            }
            neopixel_set_color(mode_color);
            neopixel_trigger_mode_flash(mode_color, 1500);
            
            // Send updated info back to bridge immediately
            kmbox_send_info_to_bridge();
            fast_cmd_count++;
            return true;
        }
        
        // Xbox gamepad injection commands
        case FAST_CMD_XBOX_INPUT: {
            // [0x20][btn_lo][btn_hi][trig_l_lo][trig_l_hi][trig_r_lo][trig_r_hi][pad]
            uint16_t buttons = (uint16_t)(pkt[1] | (pkt[2] << 8));
            uint16_t trig_l  = (uint16_t)(pkt[3] | (pkt[4] << 8));
            uint16_t trig_r  = (uint16_t)(pkt[5] | (pkt[6] << 8));
            xbox_inject_buttons(buttons, trig_l, trig_r);
            fast_cmd_count++;
            return true;
        }

        case FAST_CMD_XBOX_STICK_L: {
            // [0x22][x_lo][x_hi][y_lo][y_hi][pad][pad][pad]
            int16_t x = read_i16_le(pkt + 1);
            int16_t y = read_i16_le(pkt + 3);
            xbox_inject_stick_left(x, y);
            fast_cmd_count++;
            return true;
        }

        case FAST_CMD_XBOX_STICK_R: {
            // [0x23][x_lo][x_hi][y_lo][y_hi][pad][pad][pad]
            int16_t x = read_i16_le(pkt + 1);
            int16_t y = read_i16_le(pkt + 3);
            xbox_inject_stick_right(x, y);
            fast_cmd_count++;
            return true;
        }

        case FAST_CMD_XBOX_RELEASE: {
            // [0x27][pad*7] - clear all injection overrides
            xbox_inject_clear();
            fast_cmd_count++;
            return true;
        }

        default:
            fast_cmd_errors++;
            return false;
    }
}

//--------------------------------------------------------------------+
// Fast Integer Parser (replaces sscanf for performance)
//--------------------------------------------------------------------+

// Fast signed integer parser - returns pointer to char after number, or NULL on error
static const char* __not_in_flash_func(parse_int16)(const char *p, int16_t *out) {
    int32_t val = 0;
    bool neg = false;
    
    if (*p == '-') { neg = true; p++; }
    else if (*p == '+') { p++; }
    
    if (*p < '0' || *p > '9') return NULL;
    
    while (*p >= '0' && *p <= '9') {
        val = val * 10 + (*p - '0');
        // Bail out early if value exceeds int16_t range (prevents int32_t overflow
        // on pathologically long digit strings)
        if (val > 32767) {
            val = 32767;
            // Skip remaining digits
            while (*(++p) >= '0' && *p <= '9') {}
            break;
        }
        p++;
    }
    
    *out = (int16_t)(neg ? -val : val);
    return p;
}

//--------------------------------------------------------------------+
// Text Protocol Handling
//--------------------------------------------------------------------+

static bool handle_text_command(const char *line, size_t len, uint32_t now_ms) {
    (void)now_ms;
    
    // Mouse move: M<x>,<y>
    if (len >= 3 && line[0] == 'M') {
        int16_t x, y;
        const char *p = parse_int16(line + 1, &x);
        if (p && *p == ',') {
            p = parse_int16(p + 1, &y);
            if (p) {
                kmbox_add_mouse_movement(x, y);
                return true;
            }
        }
    }
    
    // Wheel: W<delta>
    if (len >= 2 && line[0] == 'W') {
        int16_t delta;
        const char *p = parse_int16(line + 1, &delta);
        if (p) {
            kmbox_add_wheel_movement((int8_t)delta);
            return true;
        }
    }
    
    // Buttons: B<mask>
    if (len >= 2 && line[0] == 'B') {
        int16_t mask;
        const char *p = parse_int16(line + 1, &mask);
        if (p) {
            kmbox_update_physical_buttons((uint8_t)mask);
            return true;
        }
    }
    
    // Ping: P
    if (len >= 1 && line[0] == 'P') {
        kmbox_send_response("KMBOX_PONG");
        return true;
    }
    
    // Echo: E<data>
    if (len >= 2 && line[0] == 'E') {
        char response[KMBOX_CMD_BUFFER_SIZE + 8];
        snprintf(response, sizeof(response), "ECHO:%s", line + 1);
        kmbox_send_response(response);
        return true;
    }
    
    // Protocol: KMBOX_BRIDGE_SYNC
    if (len >= 17 && strncmp(line, "KMBOX_BRIDGE_SYNC", 17) == 0) {
        kmbox_send_response("KMBOX_READY");
        set_connection_state(BRIDGE_STATE_CONNECTED);
        return true;
    }
    
    // Protocol: KMBOX_PING
    if (len >= 10 && strncmp(line, "KMBOX_PING", 10) == 0) {
        kmbox_send_response("KMBOX_PONG:v" KMBOX_PROTOCOL_VERSION);
        if (g_connection_state == BRIDGE_STATE_WAITING || g_connection_state == BRIDGE_STATE_DISCONNECTED) {
            set_connection_state(BRIDGE_STATE_CONNECTING);
        }
        return true;
    }
    
    // Protocol: KMBOX_CONNECT
    if (len >= 13 && strncmp(line, "KMBOX_CONNECT", 13) == 0) {
        kmbox_send_response("KMBOX_READY");
        set_connection_state(BRIDGE_STATE_CONNECTED);
        return true;
    }
    
    // Protocol: KMBOX_DISCONNECT
    if (len >= 16 && strncmp(line, "KMBOX_DISCONNECT", 16) == 0) {
        kmbox_send_response("KMBOX_BYE");
        set_connection_state(BRIDGE_STATE_WAITING);
        return true;
    }
    
    // Protocol: KMBOX_STATUS
    if (len >= 12 && strncmp(line, "KMBOX_STATUS", 12) == 0) {
        static const char *responses[] = {
            "KMBOX_STATE:WAITING",
            "KMBOX_STATE:CONNECTING",
            "KMBOX_STATE:CONNECTED",
            "KMBOX_STATE:ACTIVE",
            "KMBOX_STATE:DISCONNECTED"
        };
        if (g_connection_state < 5) {
            kmbox_send_response(responses[g_connection_state]);
        }
        return true;
    }
    
    // Protocol: KMBOX_INFO
    // Note: This is a diagnostic command, not hot path, but we still optimize it
    if (len >= 10 && strncmp(line, "KMBOX_INFO", 10) == 0) {
        char resp[128];
        uint16_t vid = get_attached_vid();
        uint16_t pid = get_attached_pid();
        const char *mfr = get_attached_manufacturer();
        const char *prod = get_attached_product();
        
        // Format hex values manually (faster than snprintf for simple hex)
        static const char hex[] = "0123456789ABCDEF";
        resp[0] = 'K'; resp[1] = 'M'; resp[2] = 'B'; resp[3] = 'O'; resp[4] = 'X'; resp[5] = '_';
        resp[6] = 'V'; resp[7] = 'I'; resp[8] = 'D'; resp[9] = ':';
        resp[10] = hex[(vid >> 12) & 0xF];
        resp[11] = hex[(vid >> 8) & 0xF];
        resp[12] = hex[(vid >> 4) & 0xF];
        resp[13] = hex[vid & 0xF];
        resp[14] = '\0';
        kmbox_send_response(resp);
        
        resp[6] = 'P'; resp[7] = 'I'; resp[8] = 'D';
        resp[10] = hex[(pid >> 12) & 0xF];
        resp[11] = hex[(pid >> 8) & 0xF];
        resp[12] = hex[(pid >> 4) & 0xF];
        resp[13] = hex[pid & 0xF];
        kmbox_send_response(resp);
        
        // Manufacturer and product need snprintf for string concatenation
        snprintf(resp, sizeof(resp), "KMBOX_MFR:%s", mfr[0] ? mfr : "Unknown");
        kmbox_send_response(resp);
        snprintf(resp, sizeof(resp), "KMBOX_PROD:%s", prod[0] ? prod : "Unknown");
        kmbox_send_response(resp);
        
        // Humanization info - still needs snprintf for multiple integers
        uint32_t total_inj = 0, frames_proc = 0, q_overflows = 0;
        uint8_t q_count = 0;
        smooth_get_stats(&total_inj, &frames_proc, &q_overflows, &q_count);
        snprintf(resp, sizeof(resp), "KMBOX_INFO:hmode=%d,imode=%d,max=%d,vel=%d,qd=%d,qc=%d,inj=%lu,ovf=%lu",
                 (int)smooth_get_humanization_mode(),
                 (int)smooth_get_inject_mode(),
                 (int)smooth_get_max_per_frame(),
                 (int)smooth_get_velocity_matching(),
                 (int)q_count,
                 (int)SMOOTH_QUEUE_SIZE,
                 (unsigned long)total_inj,
                 (unsigned long)q_overflows);
        kmbox_send_response(resp);
        return true;
    }
    
    return false;
}

// Parse a complete line from the ring buffer
static bool parse_text_line(char *buf, size_t bufsize, size_t *out_len) {
    if (!buf || bufsize < 2 || !out_len) return false;
    *out_len = 0;
    uint16_t avail = rx_available();
    if (avail == 0) return false;

    uint16_t line_end = 0;
    while (line_end < avail && rx_peek_at(line_end) != '\n' &&
           rx_peek_at(line_end) != '\r') line_end++;
    if (line_end == avail) {
        if (text_discarding || avail >= bufsize) {
            // Discard an oversized line until its terminator, across task calls.
            rx_consume(avail);
            text_discarding = true;
        }
        return false;
    }
    if (!text_discarding && line_end < bufsize) {
        if (!rx_copy_peek((uint8_t *)buf, line_end)) return false;
        buf[line_end] = '\0';
        *out_len = line_end;
    }
    text_discarding = false;
    uint16_t consume = line_end + 1;
    if (consume < avail) {
        uint8_t next = rx_peek_at(consume);
        if (next == '\n' || next == '\r') consume++;
    }
    rx_consume(consume);
    return true;
}

static uint16_t process_text_commands(uint32_t now_ms, uint16_t budget) {
    uint16_t processed = 0;
    char line[KMBOX_CMD_BUFFER_SIZE];
    size_t len;
    while (rx_available() > 0 && processed < budget) {
        // Recheck after every line: a binary packet may follow text in the same
        // DMA batch, and its payload may contain CR or LF.
        uint8_t peek = rx_peek_at(0);
        if (!text_discarding && (is_fast_cmd_start(peek) || peek == BRIDGE_SYNC_BYTE)) break;
        if (!parse_text_line(line, sizeof(line), &len)) break;
        processed++;
        if (len > 0) {
            mark_activity(now_ms);
            if (!handle_text_command(line, len, now_ms)) {
                kmbox_process_serial_line(line, len, "\n", 1, now_ms);
            }
        }
    }

    return processed;
}

//--------------------------------------------------------------------+
// Initialization
//--------------------------------------------------------------------+

void kmbox_serial_init(void) {
    // Fix clk_peri for stable UART at 240MHz overclock
    if (clock_get_hz(clk_sys) > 133000000) {
        clock_configure(clk_peri, 0,
                        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                        48000000, 48000000);
    }
    
    // Initialize state
    uart_rx_read_pos = 0;
    uart_rx_poll_write_pos = 0;
    text_discarding = false;
    uart_rx_reported_epoch = 0;
    uart_rx_loss_pending = false;
    memset(&uart_rx_state, 0, sizeof(uart_rx_state));

    // Claim once for firmware lifetime; both USB cores may enqueue status text.
    if (!tx_lock) tx_lock = spin_lock_init((unsigned)spin_lock_claim_unused(true));
    tx_head = tx_tail = 0;
    tx_offset = 0;
    tx_dma_active = false;
    g_uart_tx_dropped = 0;
    
    g_connection_state = BRIDGE_STATE_WAITING;
    g_last_data_time_ms = to_ms_since_boot(get_absolute_time());
    
    // Configure GPIO pins
    gpio_set_function(KMBOX_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(KMBOX_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(KMBOX_UART_RX_PIN);
    
    // Initialize UART
    uart_init(KMBOX_UART, KMBOX_UART_BAUDRATE);
    uart_set_format(KMBOX_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(KMBOX_UART, true);
    
    // Disable hardware flow control (no CTS/RTS)
    uart_set_hw_flow(KMBOX_UART, false, false);
    
    // Disable UART RX interrupts - we use DMA for RX
    // TX IRQ handler still needed for non-blocking TX
    uart_set_irq_enables(KMBOX_UART, false, false);
    
    // Set up TX-only IRQ handler
    int uart_irq = (KMBOX_UART == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, on_uart_irq);
    irq_set_priority(uart_irq, 0);  // Highest priority for TX
    irq_set_enabled(uart_irq, true);
    
    // Drain any garbage from UART FIFO
    while (uart_is_readable(KMBOX_UART)) {
        uart_getc(KMBOX_UART);
    }
    
    // Initialize kmbox commands library
    kmbox_commands_init();
    kmbox_set_output_callback(command_reply_write);
    kmbox_update_states(g_last_data_time_ms);
    
    // Set initial LED
    neopixel_set_status_override(STATUS_BRIDGE_WAITING);
    
    // Send startup message
    sleep_ms(50);
    kmbox_send_response("KMBOX_READY");
}

void kmbox_serial_init_dma(void) {
    // Set up DMA circular buffer for UART RX (zero-CPU reception)
    uart_rx_dma_chan = dma_claim_unused_channel(false);
    if (uart_rx_dma_chan < 0) {
        printf("[KMBOX] WARNING: Failed to claim RX DMA channel, falling back to polling\n");
        return;
    }
    
    // Configure RX DMA with circular (ring) buffer
    dma_channel_config rx_cfg = dma_channel_get_default_config(uart_rx_dma_chan);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_cfg, false);   // Always read from UART DR
    channel_config_set_write_increment(&rx_cfg, true);    // Increment write pointer
    channel_config_set_dreq(&rx_cfg, uart_get_dreq(KMBOX_UART, false));  // RX DREQ
    channel_config_set_ring(&rx_cfg, true, __builtin_ctz(UART_RX_BUFFER_SIZE));  // Ring on write side
    channel_config_set_irq_quiet(&rx_cfg, true);
    channel_config_set_high_priority(&rx_cfg, true);      // High priority for low latency
    
    dma_rx_init(&uart_rx_state, UART_RX_BUFFER_MASK, uart_rx_read_pos,
                uart_rx_poll_write_pos, KMBOX_UART_BAUDRATE, time_us_64());
    uart_rx_state.fifo_status = &uart_get_hw(KMBOX_UART)->fr;
    // Self-trigger mode exposes byte progress while reloading without CPU rearming.
    dma_channel_configure(
        uart_rx_dma_chan,
        &rx_cfg,
        uart_rx_buffer + uart_rx_poll_write_pos, // Preserve unread polled bytes
        &uart_get_hw(KMBOX_UART)->dr,           // Read from UART data register
        dma_encode_transfer_count_with_self_trigger(DMA_RX_TRANSFER_COUNT),
        true                                     // Start immediately
    );
    
    printf("[KMBOX] DMA RX initialized: ch=%d, buf=%u bytes\n", uart_rx_dma_chan, UART_RX_BUFFER_SIZE);
    
    //--------------------------------------------------------------------+
    // DMA TX: Zero-CPU UART transmission
    //--------------------------------------------------------------------+
    int new_tx_dma_chan = dma_claim_unused_channel(false);
    if (new_tx_dma_chan < 0) {
        printf("[KMBOX] WARNING: Failed to claim TX DMA channel, using IRQ fallback\n");
        return;
    }
    
    dma_channel_config tx_cfg = dma_channel_get_default_config(new_tx_dma_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_cfg, true);     // Increment through TX buffer
    channel_config_set_write_increment(&tx_cfg, false);    // Always write to UART DR
    channel_config_set_dreq(&tx_cfg, uart_get_dreq(KMBOX_UART, true));  // TX DREQ
    channel_config_set_high_priority(&tx_cfg, true);       // High priority for low latency
    
    // Configure channel but don't start yet (will be triggered per-transfer)
    dma_channel_configure(
        new_tx_dma_chan,
        &tx_cfg,
        &uart_get_hw(KMBOX_UART)->dr,   // Write to UART data register
        NULL,                            // Set from the queued record at submission
        0,                               // Transfer count set per-transfer
        false                            // Don't start yet
    );
    
    // Switch ownership under the same lock as the IRQ consumer. If a record is
    // partly written, DMA resumes at tx_offset after the bytes already in FIFO.
    uint32_t save = spin_lock_blocking(tx_lock);
    uart_set_irq_enables(KMBOX_UART, false, false);
    uart_tx_dma_chan = new_tx_dma_chan;
    tx_service_locked();
    spin_unlock(tx_lock, save);

}

// Called on Core 0 after RX polling has reported a discontinuity.
static void rx_handle_loss(void) {
    if (!uart_rx_loss_pending) return;
    uart_rx_loss_pending = false;
    kmbox_set_injected_buttons(0);
    smooth_clear_queue();
    if (g_timed_move_alarm_id >= 0) {
        cancel_alarm(g_timed_move_alarm_id);
        g_timed_move_alarm_id = -1;
    }
    pending_timed_move.pending = false;
    pending_timed_move.fired = false;
    if (g_xbox_mode) xbox_inject_clear();
}

void kmbox_get_rx_loss_stats(uint32_t *overruns, uint64_t *discarded_bytes,
                            uint32_t *ambiguous_polls) {
    if (overruns) *overruns = uart_rx_state.overruns;
    if (discarded_bytes) *discarded_bytes = uart_rx_state.discarded_bytes;
    if (ambiguous_polls) *ambiguous_polls = uart_rx_state.ambiguous_polls;
}

//--------------------------------------------------------------------+
// Main Task
//--------------------------------------------------------------------+

void kmbox_serial_task(void) {
    tx_service();
    rx_handle_loss();
    // Share the boot-time millisecond clock with click/report processing.
    // Dividing time_us_32() first would reset this clock every ~71 minutes.
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    
    // Periodic ping to bridge (every 5 seconds)
    if (now_ms - g_last_ping_time_ms > 5000) {
        g_last_ping_time_ms = now_ms;
        kmbox_send_ping_to_bridge();
    }
    
    // Check connection timeout
    check_connection_timeout(now_ms);
    
    // Process pending timed moves
    process_pending_timed_move();
    
    // Process RX buffer — drain up to 64 packets per call.
    // The 0x01 fast move path is just two atomic adds (~10ns each),
    // so we can process aggressively without starving tud_task().
    // At 3 Mbaud with 8-byte packets, 64 packets = ~1.7ms of buffer.
    uint16_t packets_processed = 0;
    const uint16_t MAX_PACKETS_PER_CALL = 64;
    
    while (rx_available() > 0 && packets_processed < MAX_PACKETS_PER_CALL) {
        if (text_discarding) break;
        uint8_t first = rx_peek_at(0);
        
        //--------------------------------------------------------------
        // Bridge protocol (sync byte 0xBD)
        //--------------------------------------------------------------
        if (first == BRIDGE_SYNC_BYTE) {
            uint16_t avail = rx_available();
            if (avail < 2) break;  // Need more data
            
            uint8_t pkt[8];
            size_t copy = (avail < 8) ? avail : 8;
            if (!rx_copy_peek(pkt, copy)) break;

            uint8_t consumed = process_bridge_packet(pkt, copy);
            if (consumed >= 2) {
                rx_consume(consumed);
                mark_activity(now_ms);
                packets_processed++;
                continue;
            } else if (consumed == 1) {
                rx_consume(1);  // Skip bad sync
                continue;
            }
            break;  // Incomplete packet
        }
        
        //--------------------------------------------------------------
        // Fast binary commands — copy and validate before executing.
        //--------------------------------------------------------------
        if (is_fast_cmd_start(first)) {
            if (rx_available() >= 8) {
                uint8_t pkt[8];
                if (!rx_copy_peek(pkt, sizeof(pkt))) break;

                rx_consume(8);
                process_fast_command(pkt);
                mark_activity(now_ms);
                packets_processed++;
                continue;
            }
            break;  // Wait for complete packet
        }
        
        //--------------------------------------------------------------
        // Skip stray binary markers
        //--------------------------------------------------------------
        if (first == 0xFE || first == 0xFF || first == 0x00) {
            rx_consume(1);
            continue;
        }

        // Not a binary command start — fall through to text processing.
        // But if the first byte is non-printable (<0x20, excluding \r\n),
        // it's likely a corrupted/partial binary packet byte.  Skip it
        // to prevent the text parser from consuming binary data that
        // happens to contain 0x0A (\n) and destroying framing.
        if (first < 0x20 && first != '\r' && first != '\n') {
            rx_consume(1);
            continue;
        }

        break;
    }

    //------------------------------------------------------------------
    // Text command processing — only runs when the buffer does NOT start
    // with a binary packet leader.  If the binary loop exited because of
    // an incomplete fast command (< 8 bytes available), the partial packet
    // bytes are still in the ring buffer.  Running the text parser on them
    // would corrupt framing: any binary data byte equaling 0x0A (\n) gets
    // treated as a line terminator, consuming and destroying the partial
    // packet.  The remaining bytes arrive within ~27µs at 3 Mbaud, so the
    // next kmbox_serial_task() call will process the completed packet.
    //------------------------------------------------------------------
    packets_processed += process_text_commands(now_ms, MAX_PACKETS_PER_CALL - packets_processed);

    rx_handle_loss();
    // Update button states
    kmbox_update_states(now_ms);
    tx_service();
}

//--------------------------------------------------------------------+
// Mouse Report
//--------------------------------------------------------------------+

bool kmbox_send_mouse_report(void) {
    if (!tud_hid_ready()) return false;
    
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    kmbox_update_states(now_ms);
    
    uint8_t buttons;
    int8_t x, y, wheel, pan;
    kmbox_get_mouse_report(&buttons, &x, &y, &wheel, &pan);
    
    return tud_hid_mouse_report(REPORT_ID_MOUSE, buttons, x, y, wheel, pan);
}

//--------------------------------------------------------------------+
// Statistics & Compat Stubs
//--------------------------------------------------------------------+

void fast_cmd_get_stats(uint32_t *count, uint32_t *errors, uint32_t *overflows) {
    if (count) *count = fast_cmd_count;
    if (errors) *errors = fast_cmd_errors;
    if (overflows) *overflows = 0;
}

void kmbox_process_bridge_injections(void) {
    // No-op: movements go through shared accumulator
}

//--------------------------------------------------------------------+
// Send Info Packet to Bridge
//--------------------------------------------------------------------+

void kmbox_send_info_to_bridge(void) {
    int16_t temp = read_temperature_decidegrees();
    uint8_t queue_count = 0;
    smooth_get_stats(NULL, NULL, NULL, &queue_count);
    
    // Pack flags byte: [0]=jitter_en [1]=vel_match [2:4]=queue_depth_3bit
    humanization_mode_t hm = smooth_get_humanization_mode();
    uint8_t flags = 0;
    if (hm >= HUMANIZATION_MICRO) flags |= 0x01;
    if (smooth_get_velocity_matching()) flags |= 0x02;
    uint8_t q3 = (queue_count > 28) ? 7 : (queue_count / 4);
    flags |= (q3 & 0x07) << 2;
    
    uint8_t info_pkt[8] = {
        FAST_CMD_INFO,
        (uint8_t)hm,
        (uint8_t)smooth_get_inject_mode(),
        (uint8_t)smooth_get_max_per_frame(),
        queue_count,
        (uint8_t)(temp & 0xFF),
        (uint8_t)((temp >> 8) & 0xFF),
        flags
    };
    uart_send_bytes(info_pkt, 8);
}

void kmbox_send_xbox_status_to_bridge(void) {
    if (!g_xbox_mode) return;

    xbox_gamepad_state_t *gs = xbox_host_get_gamepad();
    uint32_t save = spin_lock_blocking(gs->lock);
    uint16_t buttons = gs->physical.buttons;
    int16_t  lx = gs->physical.stick_lx;
    int16_t  ly = gs->physical.stick_ly;
    uint16_t tl = gs->physical.trigger_left;
    uint16_t tr = gs->physical.trigger_right;
    spin_unlock(gs->lock, save);

    // Pack triggers into a single summary byte: high 4 bits = left, low 4 = right
    // Each scaled from 0-1023 down to 0-15
    uint8_t trig_summary = (uint8_t)(((tl >> 6) & 0x0F) << 4) | ((tr >> 6) & 0x0F);

    uint8_t auth_complete = (xbox_host_get_auth_state() == XBOX_AUTH_COMPLETE) ? 1 : 0;

    uint8_t status_pkt[8] = {
        FAST_CMD_XBOX_STATUS,
        1,                                      // console_mode = true
        auth_complete,
        (uint8_t)(buttons & 0xFF),
        (uint8_t)((buttons >> 8) & 0xFF),
        (uint8_t)((lx >> 8) & 0xFF),           // stick LX high byte
        (uint8_t)((ly >> 8) & 0xFF),           // stick LY high byte
        trig_summary
    };
    uart_send_bytes(status_pkt, 8);
}
