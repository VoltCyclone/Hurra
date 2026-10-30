/**
 * TFT Display Implementation for KMBox Bridge - OPTIMIZED
 * 
 * Optimizations:
 * - Full-screen redraw per update (no dirty-region tracking)
 * - Pre-formatted string buffers with lean u32/hex formatters (no snprintf)
 * - Precomputed sin/cos LUT for gauge arcs (no runtime trig)
 * - Cached string lengths to avoid redundant strlen calls
 * - Single rate-limit layer (100ms)
 */

#include "tft_display.h"
#include "tft.h"
#include "tft_config.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if (TFT_RAW_WIDTH >= 240) && defined(BRIDGE_TOUCH_SDA_PIN)
// Capacitive touch via FT6206 over I2C (Adafruit ILI9341 shield)
#include "ft6206_touch.h"
#define TOUCH_ENABLED 1
#else
#define TOUCH_ENABLED 0
#endif

extern void picotft_init(void);

// ============================================================================
// Layout — adaptive to display size
//   ST7735:  128x160, 8x16 font, tight layout
//   ILI9341: 240x320, 8x16 font, spacious layout with larger sections
// ============================================================================

#define UPDATE_INTERVAL_MS  100  // 10 FPS - single rate limit
#define FONT_W              8
#define FONT_H              16

#if (TFT_RAW_WIDTH >= 240)
// ILI9341 240x320 — spacious layout
#define LINE_H              18
#define MARGIN              4
#define SEP_GAP             4
#define SECTION_GAP         6
#else
// ST7735 128x160 — tight layout
#define LINE_H              14
#define MARGIN              2
#define SEP_GAP             2
#define SECTION_GAP         2
#endif

// Palette indices into tft_palette[256]
// Indices 0x00-0x0F = grayscale ramp (black→white)
// Indices 0xF0-0xFF = saturated colors
#define COL_BG              0x00        // Black
#define COL_WHITE           0x0F        // Pure white  (255,255,255)
#define COL_GREEN           0xF4        // Bright green (32,255,0)
#define COL_YELLOW          0xF1        // Bright yellow-orange (255,186,0)
#define COL_RED             0xFF        // Pure red (255,0,0)
#define COL_CYAN            0xF7        // Bright cyan (0,255,255)
#define COL_GRAY            0x0A        // Medium-bright gray (172,170,172) - labels
#define COL_DARK            0x06        // Medium gray (98,101,98) - secondary text
#define COL_DIM_LINE        0x04        // Dim gray (65,68,65) - separator lines & zone hints

// ============================================================================
// Fast integer-to-string helpers (avoid snprintf overhead)
// ============================================================================

// Write uint32 to buf, return pointer past last char written.
static char *u32_to_str(char *buf, uint32_t v) {
    char tmp[10];
    int i = 0;
    if (v == 0) { *buf++ = '0'; return buf; }
    while (v) { tmp[i++] = '0' + (v % 10); v /= 10; }
    while (i--) *buf++ = tmp[i];
    return buf;
}

// Write uint32 as zero-padded 2-digit decimal.
static char *u32_to_str02(char *buf, uint32_t v) {
    *buf++ = '0' + (v / 10) % 10;
    *buf++ = '0' + v % 10;
    return buf;
}

// Write uint16 as 4-digit uppercase hex.
static char *u16_to_hex4(char *buf, uint16_t v) {
    static const char hex[] = "0123456789ABCDEF";
    *buf++ = hex[(v >> 12) & 0xF];
    *buf++ = hex[(v >> 8)  & 0xF];
    *buf++ = hex[(v >> 4)  & 0xF];
    *buf++ = hex[v & 0xF];
    return buf;
}

// Format into buf, NUL-terminate, return length.
static inline int fmt_len(const char *buf, char *end) {
    *end = '\0';
    return (int)(end - buf);
}

// ============================================================================
// Precomputed sin/cos LUT for gauge arcs (step=2°, 0..358°)
// Stored as Q15 fixed-point: value = (int)(sinf(a) * 32767)
// ============================================================================

#define SINCOS_LUT_SIZE 180  // 360° / 2° step

static const int16_t sin_lut[SINCOS_LUT_SIZE] = {
        0,   1143,   2285,   3425,   4560,   5690,   6812,   7927,
     9032,  10125,  11207,  12274,  13327,  14364,  15383,  16383,
    17363,  18323,  19260,  20173,  21062,  21925,  22762,  23570,
    24350,  25100,  25820,  26509,  27165,  27787,  28377,  28931,
    29450,  29934,  30381,  30791,  31163,  31497,  31793,  32050,
    32269,  32448,  32587,  32687,  32747,  32767,  32747,  32687,
    32587,  32448,  32269,  32050,  31793,  31497,  31163,  30791,
    30381,  29934,  29450,  28931,  28377,  27787,  27165,  26509,
    25820,  25100,  24350,  23570,  22762,  21925,  21062,  20173,
    19260,  18323,  17363,  16383,  15383,  14364,  13327,  12274,
    11207,  10125,   9032,   7927,   6812,   5690,   4560,   3425,
     2285,   1143,      0,  -1143,  -2285,  -3425,  -4560,  -5690,
    -6812,  -7927,  -9032, -10125, -11207, -12274, -13327, -14364,
   -15383, -16383, -17363, -18323, -19260, -20173, -21062, -21925,
   -22762, -23570, -24350, -25100, -25820, -26509, -27165, -27787,
   -28377, -28931, -29450, -29934, -30381, -30791, -31163, -31497,
   -31793, -32050, -32269, -32448, -32587, -32687, -32747, -32767,
   -32747, -32687, -32587, -32448, -32269, -32050, -31793, -31497,
   -31163, -30791, -30381, -29934, -29450, -28931, -28377, -27787,
   -27165, -26509, -25820, -25100, -24350, -23570, -22762, -21925,
   -21062, -20173, -19260, -18323, -17363, -16383, -15383, -14364,
   -13327, -12274, -11207, -10125,  -9032,  -7927,  -6812,  -5690,
    -4560,  -3425,  -2285,  -1143,
};

// cos(a) = sin(a + 90°)
static inline int16_t lut_sin(int angle_idx) { return sin_lut[angle_idx % SINCOS_LUT_SIZE]; }
static inline int16_t lut_cos(int angle_idx) { return sin_lut[(angle_idx + 45) % SINCOS_LUT_SIZE]; } // +45 entries = +90°

// Fast integer square root (Newton's method, 4 iterations — good for r<=320)
static inline int fast_isqrt(int v) {
    if (v <= 0) return 0;
    int x = v;
    // Initial estimate: highest bit / 2
    int s = 1;
    while (s * s <= v) s <<= 1;
    x = s;
    x = (x + v / x) >> 1;
    x = (x + v / x) >> 1;
    x = (x + v / x) >> 1;
    // Correct off-by-one
    if ((x + 1) * (x + 1) <= v) x++;
    return x;
}

// ============================================================================
// State — timer-driven background rendering
//
// Architecture:
//   Main loop calls tft_display_submit_stats() to copy stats into shared buf.
//   A repeating hardware timer marks rendering due every 100ms.
//   The main loop formats/draws the latest stats and advances one row of SPI
//   DMA per flush call. No rendering runs in IRQ context.
// ============================================================================

static bool initialized = false;
static uint32_t last_tx_bytes = 0;
static uint32_t last_rx_bytes = 0;
static tft_view_mode_t current_view = TFT_VIEW_DETAILED;

// Shared menu request structure (menu writes, main loop reads & clears)
volatile tft_menu_request_t tft_menu_request = {0};

// Touch handling - zone-based tap detection
// Screen is divided into 3 horizontal zones:
//   Top    (y 0..106):   Cycle display view
//   Middle (y 107..213): Cycle API protocol
//   Bottom (y 214..319): Cycle humanization mode
static volatile bool touch_view_requested = false;
static volatile bool touch_api_requested = false;
static volatile bool touch_human_requested = false;

// Touch zone boundaries (display split into thirds, adapts to any height)
#define TOUCH_ZONE_TOP_END     (TFT_HEIGHT / 3)
#define TOUCH_ZONE_MID_END     (TFT_HEIGHT * 2 / 3)

// ============================================================================
// Menu System State
// ============================================================================

#define MENU_ITEM_COUNT 5
#define MENU_ROW_H      36      // Height of each menu row (touch target)
#define MENU_HEADER_H   40      // Height of menu header area
#define MENU_MARGIN      8
#define MENU_BTN_W      48      // Width of -/+ button areas
#define MENU_HIGHLIGHT_MS 200   // Brief highlight on tap

typedef enum {
    MENU_HUMANIZATION_MODE,
    MENU_INJECT_MODE,
    MENU_MAX_PER_FRAME,
    MENU_VELOCITY_MATCHING,
    MENU_JITTER,
} menu_item_id_t;

// Cached local menu values (snapshot from stats, updated by touch)
static uint8_t menu_humanization_mode = 0;
static uint8_t menu_inject_mode = 1;
static uint8_t menu_max_per_frame = 16;
static bool    menu_velocity_matching = true;
static bool    menu_jitter_enabled = false;
static bool    menu_values_initialized = false;
static int8_t  menu_highlight_item = -1;  // Item being highlighted (-1 = none)
static uint32_t menu_highlight_time = 0;  // When highlight started

// Latest stats are submitted and rendered by the main loop.
static tft_stats_t shared_stats;           // Latest stats from main loop
static bool stats_pending = false; // Main loop sets and clears

// Hold the rendered frame until its asynchronous DMA transfer completes.
static bool frame_ready = false;
static volatile bool render_due = false;
static bool transfer_pending = false;

// Stats hash for frame-skip optimization: skip render if stats unchanged
static uint32_t last_stats_hash = 0;

// Lightweight FNV-1a hash over the key volatile fields of stats
static uint32_t stats_quick_hash(const tft_stats_t *s) {
    uint32_t h = 2166136261u;
    #define HASH_MIX(val) do { \
        uint32_t v = (uint32_t)(val); \
        h ^= v; h *= 16777619u; \
    } while(0)
    HASH_MIX(s->cdc_connected);
    HASH_MIX(s->kmbox_connected);
    HASH_MIX(s->humanization_mode);
    HASH_MIX(s->inject_mode);
    HASH_MIX(s->queue_depth);
    HASH_MIX(s->tx_rate_bps);
    HASH_MIX(s->rx_rate_bps);
    HASH_MIX(s->mouse_moves);
    HASH_MIX(s->commands_per_sec);
    HASH_MIX(s->latency_avg_us);
    HASH_MIX(s->latency_jitter_us);
    HASH_MIX(s->uptime_sec);
    HASH_MIX(s->button_presses);
    HASH_MIX(s->total_injected);
    HASH_MIX(s->uart_errors + s->frame_errors);
    HASH_MIX((uint32_t)(s->bridge_temperature_c * 10));
    HASH_MIX((uint32_t)(s->kmbox_temperature_c * 10));
    HASH_MIX(s->console_mode);
    HASH_MIX(s->gamepad_buttons);
    #undef HASH_MIX
    return h;
}

// Repeating timer handle
static repeating_timer_t tft_render_timer;

// Pre-allocated format buffers with cached lengths
typedef struct { char str[24]; int len; } fmtbuf_t;

static fmtbuf_t fmt_baud;
static fmtbuf_t fmt_uptime;
static fmtbuf_t fmt_tx_rate;
static fmtbuf_t fmt_rx_rate;
static fmtbuf_t fmt_rx_buf;
static fmtbuf_t fmt_moves;
static fmtbuf_t fmt_vid_pid;
static fmtbuf_t fmt_br_temp;
static fmtbuf_t fmt_km_temp;
static fmtbuf_t fmt_hmode;
static fmtbuf_t fmt_hdetail;
static fmtbuf_t fmt_queuebar;
static fmtbuf_t fmt_injcount;
static fmtbuf_t fmt_cpu;
static fmtbuf_t fmt_cmd_rate;
static fmtbuf_t fmt_lat_avg;
static fmtbuf_t fmt_lat_range;
static fmtbuf_t fmt_lat_jitter;
static fmtbuf_t fmt_errors;
static fmtbuf_t fmt_buttons;
static fmtbuf_t fmt_tx_peak;
static fmtbuf_t fmt_rx_peak;

// Helper to clear a fmtbuf
static inline void fmt_clear(fmtbuf_t *f) { f->str[0] = '\0'; f->len = 0; }

// ============================================================================
// Drawing Helpers
// ============================================================================

static inline void hline(int y, uint8_t color) {
    if (y >= 0 && y < TFT_HEIGHT) {
        memset(&tft_input[y * TFT_WIDTH + MARGIN], color, TFT_WIDTH - MARGIN * 2);
    }
}

static inline void clear_line(int y) {
    if (y >= 0 && y < TFT_HEIGHT) {
        memset(&tft_input[y * TFT_WIDTH], COL_BG, TFT_WIDTH);
    }
}

static inline void clear_region(int y, int h) {
    int y0 = (y < 0) ? 0 : y;
    int y1 = (y + h > TFT_HEIGHT) ? TFT_HEIGHT : y + h;
    if (y1 > y0) {
        memset(&tft_input[y0 * TFT_WIDTH], COL_BG, (y1 - y0) * TFT_WIDTH);
    }
}

static void box(int x, int y, int w, int h, uint8_t color) {
    int y0 = (y < 0) ? 0 : y;
    int y1 = (y + h > TFT_HEIGHT) ? TFT_HEIGHT : y + h;
    int x0 = (x < 0) ? 0 : x;
    int x1 = (x + w > TFT_WIDTH) ? TFT_WIDTH : x + w;
    int width = x1 - x0;
    if (width <= 0) return;
    
    for (int row = y0; row < y1; row++) {
        memset(&tft_input[row * TFT_WIDTH + x0], color, width);
    }
}

static inline uint8_t temp_color(float t) {
    if (t > 70.0f) return COL_RED;
    if (t > 55.0f) return COL_YELLOW;
    if (t < 10.0f) return COL_CYAN;
    return COL_WHITE;
}

static inline uint8_t hmode_color(uint8_t m) {
    static const uint8_t colors[] = { COL_DARK, COL_YELLOW, COL_GREEN, COL_RED };
    return (m < 3) ? colors[m] : COL_RED;
}

#define TEMP_VALID(t) ((t) > -50.0f && (t) < 150.0f)

static inline uint8_t latency_color(uint32_t us) {
    if (us < 200)  return COL_GREEN;
    if (us < 500)  return COL_YELLOW;
    return COL_RED;
}

// ============================================================================
// Touch Interrupt Callback
// ============================================================================

#if TOUCH_ENABLED
// Menu touch callback: handles taps when in menu view
static void menu_touch_event(uint16_t x, uint16_t y);

static void touch_event_callback(uint16_t x, uint16_t y) {
    if (current_view == TFT_VIEW_MENU) {
        menu_touch_event(x, y);
        return;
    }
    // Non-menu views: route tap to the correct zone flag based on Y coordinate
    // The display is 240x320, divided into 3 horizontal bands
    if (y <= TOUCH_ZONE_TOP_END) {
        touch_view_requested = true;       // Top: cycle view
    } else if (y <= TOUCH_ZONE_MID_END) {
        touch_api_requested = true;        // Middle: cycle API protocol
    } else {
        touch_human_requested = true;      // Bottom: cycle humanization
    }
}

// Menu touch handler — called from touch_event_callback when in MENU view
static void menu_touch_event(uint16_t x, uint16_t y) {
    // Header zone: tap to go back to detailed view
    if (y < MENU_HEADER_H) {
        touch_view_requested = true;
        return;
    }

    // Determine which menu row was tapped
    int row = (y - MENU_HEADER_H) / MENU_ROW_H;
    if (row < 0 || row >= MENU_ITEM_COUNT) return;

    // Left half = decrease, right half = increase
    bool increase = (x >= TFT_WIDTH / 2);

    menu_highlight_item = (int8_t)row;
    menu_highlight_time = to_ms_since_boot(get_absolute_time());

    switch ((menu_item_id_t)row) {
        case MENU_HUMANIZATION_MODE:
            if (increase) {
                menu_humanization_mode = (menu_humanization_mode + 1) % 3;
            } else {
                menu_humanization_mode = (menu_humanization_mode + 2) % 3;
            }
            tft_menu_request.humanization_mode_val = menu_humanization_mode;
            tft_menu_request.set_humanization_mode = true;
            break;

        case MENU_INJECT_MODE:
            if (increase) {
                menu_inject_mode = (menu_inject_mode + 1) % 4;
            } else {
                menu_inject_mode = (menu_inject_mode + 3) % 4;
            }
            // Inject mode is part of smooth config — signal both
            tft_menu_request.inject_mode_val = menu_inject_mode;
            tft_menu_request.set_inject_mode = true;
            break;

        case MENU_MAX_PER_FRAME:
            if (increase) {
                if (menu_max_per_frame < 32) menu_max_per_frame++;
            } else {
                if (menu_max_per_frame > 1) menu_max_per_frame--;
            }
            tft_menu_request.max_per_frame_val = menu_max_per_frame;
            tft_menu_request.set_max_per_frame = true;
            break;

        case MENU_VELOCITY_MATCHING:
            menu_velocity_matching = !menu_velocity_matching;
            tft_menu_request.velocity_matching_val = menu_velocity_matching ? 1 : 0;
            tft_menu_request.set_velocity_matching = true;
            break;

        case MENU_JITTER:
            // Jitter is implicitly controlled by humanization mode
            // (>= MICRO enables jitter), but we allow override here
            menu_jitter_enabled = !menu_jitter_enabled;
            // Toggling jitter is achieved by toggling humanization mode
            // If turning on: set to MICRO, if turning off: set to OFF
            if (menu_jitter_enabled && menu_humanization_mode == 0) {
                menu_humanization_mode = 1;  // Enable MICRO for jitter
                tft_menu_request.humanization_mode_val = menu_humanization_mode;
                tft_menu_request.set_humanization_mode = true;
            } else if (!menu_jitter_enabled && menu_humanization_mode > 0) {
                menu_humanization_mode = 0;  // Disable humanization to stop jitter
                tft_menu_request.humanization_mode_val = menu_humanization_mode;
                tft_menu_request.set_humanization_mode = true;
            }
            break;
    }
}
#endif

// ============================================================================
// Screen Layout - Simplified full redraw with pre-formatted strings
// ============================================================================

static void format_stats(const tft_stats_t *stats) {
    char *p;

    // Baud rate: "115K" or "921K"
    p = u32_to_str(fmt_baud.str, stats->uart_baud / 1000);
    *p++ = 'K';
    fmt_baud.len = fmt_len(fmt_baud.str, p);

    // Uptime: "12m05s"
    p = u32_to_str(fmt_uptime.str, stats->uptime_sec / 60);
    *p++ = 'm';
    p = u32_to_str02(p, stats->uptime_sec % 60);
    *p++ = 's';
    fmt_uptime.len = fmt_len(fmt_uptime.str, p);

    // TX/RX byte rates
    p = u32_to_str(fmt_tx_rate.str, stats->tx_rate_bps);
    *p++ = ' '; *p++ = 'B'; *p++ = '/'; *p++ = 's';
    fmt_tx_rate.len = fmt_len(fmt_tx_rate.str, p);

    p = u32_to_str(fmt_rx_rate.str, stats->rx_rate_bps);
    *p++ = ' '; *p++ = 'B'; *p++ = '/'; *p++ = 's';
    fmt_rx_rate.len = fmt_len(fmt_rx_rate.str, p);

    // RX buffer level
    if (stats->rx_buffer_level > 0) {
        p = fmt_rx_buf.str;
        *p++ = 'b'; *p++ = 'u'; *p++ = 'f'; *p++ = ' ';
        p = u32_to_str(p, stats->rx_buffer_level);
        fmt_rx_buf.len = fmt_len(fmt_rx_buf.str, p);
    } else { fmt_clear(&fmt_rx_buf); }

    // Mouse moves
    p = u32_to_str(fmt_moves.str, stats->mouse_moves);
    fmt_moves.len = fmt_len(fmt_moves.str, p);

    // VID:PID
    if (stats->device_vid != 0) {
        p = u16_to_hex4(fmt_vid_pid.str, stats->device_vid);
        *p++ = ':';
        p = u16_to_hex4(p, stats->device_pid);
        fmt_vid_pid.len = fmt_len(fmt_vid_pid.str, p);
    } else { fmt_clear(&fmt_vid_pid); }

    // Bridge temp: "Bridge 42C"
    if (TEMP_VALID(stats->bridge_temperature_c)) {
        p = fmt_br_temp.str;
        p = u32_to_str(p, (uint32_t)(stats->bridge_temperature_c + 0.5f));
        *p++ = 'C';
        fmt_br_temp.len = fmt_len(fmt_br_temp.str, p);
    } else { fmt_clear(&fmt_br_temp); }

    // KMBox temp: "KMBox 38C"
    if (stats->kmbox_connected && TEMP_VALID(stats->kmbox_temperature_c)) {
        p = fmt_km_temp.str;
        p = u32_to_str(p, (uint32_t)(stats->kmbox_temperature_c + 0.5f));
        *p++ = 'C';
        fmt_km_temp.len = fmt_len(fmt_km_temp.str, p);
    } else { fmt_clear(&fmt_km_temp); }

    // Humanization mode: "Off", "Low", "Med", "High"
    if (stats->humanization_valid) {
        static const char *hmode_labels[] = { "Off", "Micro", "Full" };
        const char *label = (stats->humanization_mode < 3) ? hmode_labels[stats->humanization_mode] : "?";
        p = fmt_hmode.str;
        const char *lp = label;
        while (*lp) *p++ = *lp++;
        fmt_hmode.len = fmt_len(fmt_hmode.str, p);
    } else { fmt_clear(&fmt_hmode); }

    // Humanization detail flags
    if (stats->humanization_valid) {
        p = fmt_hdetail.str;
        if (stats->jitter_enabled && stats->velocity_matching) {
            const char *s = "Jit+Vel";
            while (*s) *p++ = *s++;
        } else if (stats->jitter_enabled) {
            const char *s = "Jitter";
            while (*s) *p++ = *s++;
        } else if (stats->velocity_matching) {
            const char *s = "VelMatch";
            while (*s) *p++ = *s++;
        } else {
            const char *s = "None";
            while (*s) *p++ = *s++;
        }
        fmt_hdetail.len = fmt_len(fmt_hdetail.str, p);
    } else { fmt_clear(&fmt_hdetail); }

    // Queue depth: "3/32"
    if (stats->humanization_valid) {
        p = fmt_queuebar.str;
        p = u32_to_str(p, stats->queue_depth);
        *p++ = '/';
        p = u32_to_str(p, stats->queue_capacity);
        fmt_queuebar.len = fmt_len(fmt_queuebar.str, p);
    } else { fmt_clear(&fmt_queuebar); }

    // Injection count
    if (stats->humanization_valid && stats->total_injected > 0) {
        p = u32_to_str(fmt_injcount.str, stats->total_injected);
        fmt_injcount.len = fmt_len(fmt_injcount.str, p);
    } else { fmt_clear(&fmt_injcount); }

    // CPU frequency: "150 MHz"
    p = u32_to_str(fmt_cpu.str, stats->cpu_freq_mhz);
    *p++ = ' '; *p++ = 'M'; *p++ = 'H'; *p++ = 'z';
    fmt_cpu.len = fmt_len(fmt_cpu.str, p);

    // Command rate: "123 cmd/s"
    if (stats->commands_per_sec > 0) {
        p = u32_to_str(fmt_cmd_rate.str, stats->commands_per_sec);
        *p++ = ' '; *p++ = 'c'; *p++ = 'm'; *p++ = 'd'; *p++ = '/'; *p++ = 's';
        fmt_cmd_rate.len = fmt_len(fmt_cmd_rate.str, p);
    } else { fmt_clear(&fmt_cmd_rate); }

    // Latency
    if (stats->latency_samples > 0) {
        p = u32_to_str(fmt_lat_avg.str, stats->latency_avg_us);
        *p++ = ' '; *p++ = 'u'; *p++ = 's';
        fmt_lat_avg.len = fmt_len(fmt_lat_avg.str, p);

        p = u32_to_str(fmt_lat_range.str, stats->latency_min_us);
        *p++ = '-';
        p = u32_to_str(p, stats->latency_max_us);
        *p++ = ' '; *p++ = 'u'; *p++ = 's';
        fmt_lat_range.len = fmt_len(fmt_lat_range.str, p);

        p = fmt_lat_jitter.str; *p++ = '+'; *p++ = '/';
        *p++ = '-';
        p = u32_to_str(p, stats->latency_jitter_us);
        *p++ = ' '; *p++ = 'u'; *p++ = 's';
        fmt_lat_jitter.len = fmt_len(fmt_lat_jitter.str, p);
    } else {
        fmt_clear(&fmt_lat_avg);
        fmt_clear(&fmt_lat_range);
        fmt_clear(&fmt_lat_jitter);
    }

    // Errors
    if (stats->uart_errors > 0 || stats->frame_errors > 0) {
        p = fmt_errors.str;
        p = u32_to_str(p, stats->uart_errors);
        *p++ = ' '; *p++ = 'U'; *p++ = 'A'; *p++ = 'R'; *p++ = 'T';
        *p++ = ' ';
        p = u32_to_str(p, stats->frame_errors);
        *p++ = ' '; *p++ = 'F'; *p++ = 'r'; *p++ = 'm';
        fmt_errors.len = fmt_len(fmt_errors.str, p);
    } else { fmt_clear(&fmt_errors); }

    // Button presses
    if (stats->button_presses > 0) {
        p = u32_to_str(fmt_buttons.str, stats->button_presses);
        fmt_buttons.len = fmt_len(fmt_buttons.str, p);
    } else { fmt_clear(&fmt_buttons); }

    // Peak rates
    if (stats->tx_peak_bps > 0) {
        p = fmt_tx_peak.str;
        *p++ = 'p'; *p++ = 'k'; *p++ = ' ';
        p = u32_to_str(p, stats->tx_peak_bps);
        fmt_tx_peak.len = fmt_len(fmt_tx_peak.str, p);
    } else { fmt_clear(&fmt_tx_peak); }

    if (stats->rx_peak_bps > 0) {
        p = fmt_rx_peak.str;
        *p++ = 'p'; *p++ = 'k'; *p++ = ' ';
        p = u32_to_str(p, stats->rx_peak_bps);
        fmt_rx_peak.len = fmt_len(fmt_rx_peak.str, p);
    } else { fmt_clear(&fmt_rx_peak); }
}

// Helper to draw a section header label with accent bar
static int draw_section_header(int y, const char *title) {
    hline(y, COL_DIM_LINE);
    y += SEP_GAP + 1;
    // Small accent bar before title
    box(MARGIN, y + 4, 3, FONT_H - 8, COL_CYAN);
    tft_draw_string(MARGIN + 6, y, COL_CYAN, title);
    y += LINE_H;
    return y;
}

// Helper to draw a label: value pair, right-aligned value
static void draw_label_value(int y, const char *label, const char *value, uint8_t val_color) {
    tft_draw_string(MARGIN, y, COL_GRAY, label);
    int lbl_len = 0;
    const char *lp = label;
    while (*lp++) lbl_len++;
    int val_x = MARGIN + (lbl_len + 1) * FONT_W;
    tft_draw_string(val_x, y, val_color, value);
}

// Helper to draw label on left, value right-aligned
static void draw_row_lr(int y, const char *label, uint8_t lbl_col,
                        const char *value, int val_len, uint8_t val_col) {
    tft_draw_string(MARGIN, y, lbl_col, label);
    int rx = TFT_WIDTH - MARGIN - val_len * FONT_W;
    tft_draw_string(rx, y, val_col, value);
}

// ============================================================================
// Console Mode View (Xbox Passthrough)
// ============================================================================

static void draw_console_view(const tft_stats_t *stats) {
    int y = MARGIN;

    // Header in Xbox green
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_GREEN, "CONSOLE MODE");
    y += LINE_H + SEP_GAP;
    hline(y, COL_DIM_LINE);
    y += SEP_GAP + 2;

    // Auth status
    tft_draw_string(MARGIN, y, COL_GRAY, "Auth:");
    tft_draw_string(MARGIN + 6 * FONT_W, y,
                    stats->console_auth_complete ? COL_GREEN : COL_YELLOW,
                    stats->console_auth_complete ? "OK" : "...");
    y += LINE_H;

    // Buttons (hex display)
    tft_draw_string(MARGIN, y, COL_GRAY, "Btn:");
    char btn_buf[8];
    btn_buf[0] = '0'; btn_buf[1] = 'x';
    // Simple hex format for 16-bit value
    static const char hex_chars[] = "0123456789ABCDEF";
    btn_buf[2] = hex_chars[(stats->gamepad_buttons >> 12) & 0xF];
    btn_buf[3] = hex_chars[(stats->gamepad_buttons >> 8) & 0xF];
    btn_buf[4] = hex_chars[(stats->gamepad_buttons >> 4) & 0xF];
    btn_buf[5] = hex_chars[stats->gamepad_buttons & 0xF];
    btn_buf[6] = '\0';
    tft_draw_string(MARGIN + 5 * FONT_W, y, COL_WHITE, btn_buf);
    y += LINE_H;

    // Left stick
    tft_draw_string(MARGIN, y, COL_GRAY, "LS:");
    char stick_buf[16];
    snprintf(stick_buf, sizeof(stick_buf), "%6d %6d",
             stats->gamepad_sticks[0], stats->gamepad_sticks[1]);
    tft_draw_string(MARGIN + 4 * FONT_W, y, COL_CYAN, stick_buf);
    y += LINE_H;

    // Right stick
    tft_draw_string(MARGIN, y, COL_GRAY, "RS:");
    snprintf(stick_buf, sizeof(stick_buf), "%6d %6d",
             stats->gamepad_sticks[2], stats->gamepad_sticks[3]);
    tft_draw_string(MARGIN + 4 * FONT_W, y, COL_CYAN, stick_buf);
    y += LINE_H;

    // Triggers
    tft_draw_string(MARGIN, y, COL_GRAY, "LT:");
    char trig_buf[8];
    snprintf(trig_buf, sizeof(trig_buf), "%4u", stats->gamepad_triggers[0]);
    tft_draw_string(MARGIN + 4 * FONT_W, y, COL_WHITE, trig_buf);
    tft_draw_string(MARGIN + 9 * FONT_W, y, COL_GRAY, "RT:");
    snprintf(trig_buf, sizeof(trig_buf), "%4u", stats->gamepad_triggers[1]);
    tft_draw_string(MARGIN + 13 * FONT_W, y, COL_WHITE, trig_buf);
    y += LINE_H;
}

static void draw_stats(const tft_stats_t *stats) {
    int y = MARGIN;
    
    // === HEADER ===
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_CYAN, "KMBox Bridge");
    y += LINE_H + SEP_GAP;
    hline(y, COL_DIM_LINE);
    y += SEP_GAP + 2;
    
    // === CONNECTION STATUS (with indicator dots) ===
    {
        uint8_t cdc_col = stats->cdc_connected ? COL_GREEN : COL_RED;
        uint8_t km_col  = stats->kmbox_connected ? COL_GREEN : COL_RED;

        // Status dot (filled 5x5 square) + label
        box(MARGIN, y + 5, 5, 5, cdc_col);
        tft_draw_string(MARGIN + 8, y, COL_GRAY, "Host");
        tft_draw_string(MARGIN + 8 + 5 * FONT_W, y, cdc_col,
                        stats->cdc_connected ? "OK" : "--");

#if (TFT_RAW_WIDTH >= 240)
        int km_x = MARGIN + 14 * FONT_W;
        box(km_x, y + 5, 5, 5, km_col);
        tft_draw_string(km_x + 8, y, COL_GRAY, "KMBox");
        tft_draw_string(km_x + 8 + 6 * FONT_W, y, km_col,
                        stats->kmbox_connected ? "OK" : "--");
#else
        int km_x = MARGIN + 9 * FONT_W;
        box(km_x, y + 5, 5, 5, km_col);
        tft_draw_string(km_x + 8, y, COL_GRAY, "KM");
        tft_draw_string(km_x + 8 + 3 * FONT_W, y, km_col,
                        stats->kmbox_connected ? "OK" : "--");
#endif
        y += LINE_H;
    }
    
    // === API + Uptime row ===
    {
        const char *mode_names[] = { "KMBox", "Makcu", "Ferrum" };
        const char *mode_str = (stats->api_mode < 3) ? mode_names[stats->api_mode] : "???";
        tft_draw_string(MARGIN, y, COL_GRAY, "API");
        tft_draw_string(MARGIN + 4 * FONT_W, y, COL_WHITE, mode_str);
        // Uptime right-aligned
        int ux = TFT_WIDTH - MARGIN - fmt_uptime.len * FONT_W;
        tft_draw_string(ux, y, COL_CYAN, fmt_uptime.str);
        y += LINE_H;
    }
    
    // === HUMANIZATION SECTION ===
    if (fmt_hmode.len) {
        y += SECTION_GAP;
        y = draw_section_header(y, "Humanize");
        
        // Mode + detail flags
        tft_draw_string(MARGIN, y, COL_GRAY, "Mode");
        tft_draw_string(MARGIN + 5 * FONT_W, y,
                        hmode_color(stats->humanization_mode), fmt_hmode.str);
        if (fmt_hdetail.len) {
            int dx = TFT_WIDTH - MARGIN - fmt_hdetail.len * FONT_W;
            tft_draw_string(dx, y, COL_GRAY, fmt_hdetail.str);
        }
        y += LINE_H;
        
        // Queue bar + injection count (with visual bar)
        if (fmt_queuebar.len) {
            tft_draw_string(MARGIN, y, COL_GRAY, "Queue");

            uint8_t q_col = COL_GREEN;
            if (stats->queue_depth > stats->queue_capacity * 3 / 4) q_col = COL_RED;
            else if (stats->queue_depth > stats->queue_capacity / 2) q_col = COL_YELLOW;

            // Visual progress bar
            int bar_x = MARGIN + 6 * FONT_W;
            int bar_w = TFT_WIDTH - bar_x - MARGIN;
            if (fmt_injcount.len) bar_w -= (fmt_injcount.len + 1) * FONT_W;
            int bar_h = FONT_H - 4;
            int bar_y = y + 2;
            box(bar_x, bar_y, bar_w, bar_h, COL_DARK);
            if (stats->queue_capacity > 0) {
                int fill = (int)((float)stats->queue_depth / stats->queue_capacity * bar_w);
                if (fill > bar_w) fill = bar_w;
                if (fill > 0) box(bar_x, bar_y, fill, bar_h, q_col);
            }
            // Overlay text on bar
            int txt_x = bar_x + (bar_w - fmt_queuebar.len * FONT_W) / 2;
            tft_draw_string(txt_x, y, COL_WHITE, fmt_queuebar.str);

            if (fmt_injcount.len) {
                int ix = TFT_WIDTH - MARGIN - fmt_injcount.len * FONT_W;
                tft_draw_string(ix, y, COL_GRAY, fmt_injcount.str);
            }
            y += LINE_H;
        }
        
        // Queue overflows (if any)
        if (stats->queue_overflows > 0) {
            char ovf_buf[16]; char *op = ovf_buf;
            const char *s = "Overflow ";
            while (*s) *op++ = *s++;
            op = u32_to_str(op, stats->queue_overflows);
            *op = '\0';
            tft_draw_string(MARGIN, y, COL_RED, ovf_buf);
            y += LINE_H;
        }
    }
    
    // === DATA SECTION ===
    y += SECTION_GAP;
    y = draw_section_header(y, "Data");
    
    // TX row
    {
        bool tx_active = (stats->tx_bytes != last_tx_bytes);
        uint8_t tx_col = tx_active ? COL_GREEN : (stats->tx_rate_bps == 0 ? COL_RED : COL_WHITE);
        tft_draw_string(MARGIN, y, COL_GRAY, "TX");
        tft_draw_string(MARGIN + 3 * FONT_W, y, tx_col, fmt_tx_rate.str);
        if (fmt_tx_peak.len) {
            int px = TFT_WIDTH - MARGIN - fmt_tx_peak.len * FONT_W;
            tft_draw_string(px, y, COL_DARK, fmt_tx_peak.str);
        }
        y += LINE_H;
    }
    
    // RX row
    {
        bool rx_active = (stats->rx_bytes != last_rx_bytes);
        uint8_t rx_col = rx_active ? COL_GREEN : (stats->rx_rate_bps == 0 ? COL_RED : COL_WHITE);
        tft_draw_string(MARGIN, y, COL_GRAY, "RX");
        tft_draw_string(MARGIN + 3 * FONT_W, y, rx_col, fmt_rx_rate.str);
        if (fmt_rx_buf.len) {
            int bx = TFT_WIDTH - MARGIN - fmt_rx_buf.len * FONT_W;
            tft_draw_string(bx, y, COL_YELLOW, fmt_rx_buf.str);
        } else if (fmt_rx_peak.len) {
            int px = TFT_WIDTH - MARGIN - fmt_rx_peak.len * FONT_W;
            tft_draw_string(px, y, COL_DARK, fmt_rx_peak.str);
        }
        y += LINE_H;
    }
    
    // UART baud + command rate
    {
        tft_draw_string(MARGIN, y, COL_GRAY, "Baud");
        tft_draw_string(MARGIN + 5 * FONT_W, y, COL_GREEN, fmt_baud.str);
        if (fmt_cmd_rate.len) {
            int cx = TFT_WIDTH - MARGIN - fmt_cmd_rate.len * FONT_W;
            tft_draw_string(cx, y, COL_GREEN, fmt_cmd_rate.str);
        }
        y += LINE_H;
    }
    
    // Mouse moves + button presses
    {
        uint8_t mv_col = (stats->mouse_moves > 0) ? COL_WHITE : COL_DARK;
        tft_draw_string(MARGIN, y, COL_GRAY, "Mouse");
        tft_draw_string(MARGIN + 6 * FONT_W, y, mv_col, fmt_moves.str);
        if (fmt_buttons.len) {
            tft_draw_string(TFT_WIDTH - MARGIN - (fmt_buttons.len + 5) * FONT_W, y, COL_GRAY, "Btns");
            int bx = TFT_WIDTH - MARGIN - fmt_buttons.len * FONT_W;
            tft_draw_string(bx, y, COL_YELLOW, fmt_buttons.str);
        }
        y += LINE_H;
    }
    
    // Errors (only when present)
    if (fmt_errors.len) {
        tft_draw_string(MARGIN, y, COL_RED, fmt_errors.str);
        y += LINE_H;
    }
    
    // === LATENCY SECTION ===
    if (fmt_lat_avg.len) {
        y += SECTION_GAP;
        y = draw_section_header(y, "Latency");
        
        draw_row_lr(y, "Avg", COL_GRAY, fmt_lat_avg.str, fmt_lat_avg.len,
                    latency_color(stats->latency_avg_us));
        y += LINE_H;

#if (TFT_RAW_WIDTH >= 240)
        if (fmt_lat_range.len) {
            draw_row_lr(y, "Range", COL_GRAY, fmt_lat_range.str, fmt_lat_range.len,
                        latency_color(stats->latency_max_us));
            y += LINE_H;
        }
#endif
        if (fmt_lat_jitter.len) {
            uint8_t jit_col = stats->latency_jitter_us < 50 ? COL_GREEN :
                              stats->latency_jitter_us < 150 ? COL_YELLOW : COL_RED;
            draw_row_lr(y, "Jitter", COL_GRAY, fmt_lat_jitter.str, fmt_lat_jitter.len, jit_col);
            y += LINE_H;
        }
    }
    
    // === SYSTEM SECTION ===
    {
        y += SECTION_GAP;
        y = draw_section_header(y, "System");
        
        // CPU + Device ID
        tft_draw_string(MARGIN, y, COL_GRAY, "CPU");
        tft_draw_string(MARGIN + 4 * FONT_W, y, COL_WHITE, fmt_cpu.str);
        if (fmt_vid_pid.len) {
            int vx = TFT_WIDTH - MARGIN - fmt_vid_pid.len * FONT_W;
            tft_draw_string(vx, y, COL_CYAN, fmt_vid_pid.str);
        }
        y += LINE_H;
        
        // Product name (if available)
        if (stats->device_product[0]) {
#if (TFT_RAW_WIDTH >= 240)
            char prod[30];
            int i;
            for (i = 0; i < 29 && stats->device_product[i]; i++)
                prod[i] = stats->device_product[i];
#else
            char prod[16];
            int i;
            for (i = 0; i < 15 && stats->device_product[i]; i++)
                prod[i] = stats->device_product[i];
#endif
            prod[i] = '\0';
            tft_draw_string(MARGIN, y, COL_DARK, prod);
            y += LINE_H;
        }
        
        // Temperatures — with mini visual bar
        if (fmt_br_temp.len || fmt_km_temp.len) {
            tft_draw_string(MARGIN, y, COL_GRAY, "Temp");
            int tx = MARGIN + 5 * FONT_W;
            if (fmt_br_temp.len) {
                uint8_t tcol = temp_color(stats->bridge_temperature_c);
                tft_draw_string(tx, y, tcol, fmt_br_temp.str);
                // Mini bar: 2px tall under the text, width proportional to temp (0-100C)
                int bar_w = (int)(stats->bridge_temperature_c * (fmt_br_temp.len * FONT_W) / 100.0f);
                if (bar_w < 0) bar_w = 0;
                if (bar_w > fmt_br_temp.len * FONT_W) bar_w = fmt_br_temp.len * FONT_W;
                box(tx, y + FONT_H, bar_w, 2, tcol);
                tx += (fmt_br_temp.len + 1) * FONT_W;
            }
            if (fmt_km_temp.len) {
                uint8_t tcol = temp_color(stats->kmbox_temperature_c);
                tft_draw_string(tx, y, tcol, fmt_km_temp.str);
                int bar_w = (int)(stats->kmbox_temperature_c * (fmt_km_temp.len * FONT_W) / 100.0f);
                if (bar_w < 0) bar_w = 0;
                if (bar_w > fmt_km_temp.len * FONT_W) bar_w = fmt_km_temp.len * FONT_W;
                box(tx, y + FONT_H, bar_w, 2, tcol);
            }
        }
    }
    
#if TOUCH_ENABLED
    // === TOUCH ZONE HINTS (right edge, centered in each zone) ===
    // No hlines — they overlap content rows. Small right-edge labels only.
    {
        int zone_x = TFT_WIDTH - MARGIN - FONT_W;  // Single-char width from right edge
        // Zone 1 (top):    y 0 .. TOUCH_ZONE_TOP_END  → center vertically
        int z1_y = (TOUCH_ZONE_TOP_END) / 2 - FONT_H / 2;
        // Zone 2 (middle): y TOUCH_ZONE_TOP_END+1 .. TOUCH_ZONE_MID_END
        int z2_y = (TOUCH_ZONE_TOP_END + 1 + TOUCH_ZONE_MID_END) / 2 - FONT_H / 2;
        // Zone 3 (bottom): y TOUCH_ZONE_MID_END+1 .. TFT_HEIGHT-1
        int z3_y = (TOUCH_ZONE_MID_END + 1 + TFT_HEIGHT) / 2 - FONT_H / 2;
        tft_draw_string(zone_x, z1_y, COL_DIM_LINE, "V");
        tft_draw_string(zone_x, z2_y, COL_DIM_LINE, "A");
        tft_draw_string(zone_x, z3_y, COL_DIM_LINE, "H");
    }
#endif
}

// ============================================================================
// Gauge View - Large Visual Indicators
// ============================================================================

// Convert angle in degrees to LUT index (step 2°, wrapping).
// Input angle can be negative.
static inline int angle_to_idx(int deg) {
    int idx = ((deg % 360) + 360) / 2 % SINCOS_LUT_SIZE;
    return idx;
}

// Draw a Bresenham circle ring directly into the framebuffer
static void draw_circle_ring(int cx, int cy, int r_inner, int r_outer, uint8_t color) {
    int r2_inner = r_inner * r_inner;
    int r2_outer = r_outer * r_outer;

    for (int dy = -r_outer; dy <= r_outer; dy++) {
        int py = cy + dy;
        if (py < 0 || py >= TFT_HEIGHT) continue;

        int dy2 = dy * dy;
        int xo2 = r2_outer - dy2;
        if (xo2 < 0) continue;

        int xo = fast_isqrt(xo2);

        int xi = 0;
        int xi2 = r2_inner - dy2;
        if (xi2 > 0) {
            xi = fast_isqrt(xi2) + 1; // inner boundary exclusive
        }

        uint8_t *row = &tft_input[py * TFT_WIDTH];

        // Left arc segment
        int x0 = cx - xo;
        int x1 = cx - xi;
        if (x0 < 0) x0 = 0;
        if (x1 > TFT_WIDTH) x1 = TFT_WIDTH;
        if (x0 < x1) memset(&row[x0], color, x1 - x0);

        // Right arc segment
        x0 = cx + xi;
        x1 = cx + xo + 1;
        if (x0 < 0) x0 = 0;
        if (x1 > TFT_WIDTH) x1 = TFT_WIDTH;
        if (x0 < x1) memset(&row[x0], color, x1 - x0);
    }
}

// Draw a filled arc ring between r_inner..r_outer, from start_deg to end_deg
// Uses angle test per-pixel within bounding ring — clean filled arcs
static void draw_arc_ring(int cx, int cy, int r_inner, int r_outer,
                          int start_deg, int end_deg, uint8_t color) {
    int r2_inner = r_inner * r_inner;
    int r2_outer = r_outer * r_outer;

    // Precompute angle boundary vectors (Q15) for start/end
    int si_start = angle_to_idx(start_deg);
    int si_end   = angle_to_idx(end_deg);
    int16_t sx = lut_cos(si_start), sy = lut_sin(si_start);
    int16_t ex = lut_cos(si_end),   ey = lut_sin(si_end);

    for (int dy = -r_outer; dy <= r_outer; dy++) {
        int py = cy + dy;
        if (py < 0 || py >= TFT_HEIGHT) continue;

        int dy2 = dy * dy;
        int xo2 = r2_outer - dy2;
        if (xo2 < 0) continue;

        int xo = fast_isqrt(xo2);
        int xi = 0;
        int xi2 = r2_inner - dy2;
        if (xi2 > 0) xi = fast_isqrt(xi2) + 1;

        uint8_t *row = &tft_input[py * TFT_WIDTH];

        for (int dx = -xo; dx <= xo; dx++) {
            // Skip inner hollow
            if (dx > -xi && dx < xi) { dx = xi - 1; continue; }

            int px = cx + dx;
            if (px < 0 || px >= TFT_WIDTH) continue;

            // Angle test: is (dx,dy) between start_deg and end_deg?
            // Cross product sign test against boundary vectors
            // cross(start_vec, point) >= 0 AND cross(point, end_vec) >= 0
            // For arcs < 180°, both must be true
            // For arcs >= 180°, either can be true
            int32_t cross_start = (int32_t)sx * dy - (int32_t)sy * dx;
            int32_t cross_end   = (int32_t)dx * ey - (int32_t)dy * ex;

            int span = end_deg - start_deg;
            if (span < 0) span += 360;
            bool inside;
            if (span <= 180) {
                inside = (cross_start >= 0) && (cross_end >= 0);
            } else {
                inside = (cross_start >= 0) || (cross_end >= 0);
            }

            if (inside) {
                row[px] = color;
            }
        }
    }
}

static void draw_circular_gauge(int cx, int cy, int radius, float value, float max_value,
                                const char* label, uint8_t color) {
    // Draw gauge background ring (track)
    draw_circle_ring(cx, cy, radius - 8, radius, COL_DARK);

    // Draw value arc (filled thick ring sector)
    float percentage = (max_value > 0.0f) ? (value / max_value) : 0.0f;
    if (percentage > 1.0f) percentage = 1.0f;
    if (percentage < 0.0f) percentage = 0.0f;

    if (percentage > 0.01f) {
        int start_deg = -135 + 360; // Normalize to positive: 225°
        int end_deg = start_deg + (int)(percentage * 270.0f);
        draw_arc_ring(cx, cy, radius - 7, radius - 1,
                      start_deg, end_deg, color);
    }

    // Tick marks at 0%, 50%, 100% positions on the outer edge
    for (int i = 0; i <= 2; i++) {
        int tick_deg = -135 + i * 135;
        int ai = angle_to_idx(tick_deg);
        int16_t s = lut_sin(ai);
        int16_t c = lut_cos(ai);
        for (int r = radius + 1; r <= radius + 3; r++) {
            int tx = cx + (r * c + 16384) / 32768;
            int ty = cy + (r * s + 16384) / 32768;
            if ((unsigned)tx < (unsigned)TFT_WIDTH && (unsigned)ty < (unsigned)TFT_HEIGHT)
                tft_input[ty * TFT_WIDTH + tx] = COL_GRAY;
        }
    }

    // Draw value text in center (larger visual emphasis)
    char value_str[16];
    char *p = u32_to_str(value_str, (uint32_t)(value + 0.5f));
    *p = '\0';
    int text_len = (int)(p - value_str);
    tft_draw_string(cx - (text_len * FONT_W) / 2, cy - FONT_H / 2, COL_WHITE, value_str);

    // Draw label below center
    int label_len = 0;
    const char *lp = label;
    while (*lp++) label_len++;
    tft_draw_string(cx - (label_len * FONT_W) / 2, cy + FONT_H / 2 + 2, COL_GRAY, label);
}

// Draw a circular gauge with a unit suffix next to the value (e.g. "42C")
static void draw_circular_gauge_unit(int cx, int cy, int radius, float value, float max_value,
                                     const char* label, const char* unit, uint8_t color) {
    // Draw gauge background ring (track)
    draw_circle_ring(cx, cy, radius - 8, radius, COL_DARK);

    // Draw value arc
    float percentage = (max_value > 0.0f) ? (value / max_value) : 0.0f;
    if (percentage > 1.0f) percentage = 1.0f;
    if (percentage < 0.0f) percentage = 0.0f;

    if (percentage > 0.01f) {
        int start_deg = -135 + 360;
        int end_deg = start_deg + (int)(percentage * 270.0f);
        draw_arc_ring(cx, cy, radius - 7, radius - 1,
                      start_deg, end_deg, color);
    }

    // Tick marks at 0%, 50%, 100%
    for (int i = 0; i <= 2; i++) {
        int tick_deg = -135 + i * 135;
        int ai = angle_to_idx(tick_deg);
        int16_t s = lut_sin(ai);
        int16_t c = lut_cos(ai);
        for (int r = radius + 1; r <= radius + 3; r++) {
            int tx = cx + (r * c + 16384) / 32768;
            int ty = cy + (r * s + 16384) / 32768;
            if ((unsigned)tx < (unsigned)TFT_WIDTH && (unsigned)ty < (unsigned)TFT_HEIGHT)
                tft_input[ty * TFT_WIDTH + tx] = COL_GRAY;
        }
    }

    // Value text with unit suffix: e.g. "42C"
    char value_str[16];
    char *p = u32_to_str(value_str, (uint32_t)(value + 0.5f));
    const char *up = unit;
    while (*up) *p++ = *up++;
    *p = '\0';
    int text_len = (int)(p - value_str);
    tft_draw_string(cx - (text_len * FONT_W) / 2, cy - FONT_H / 2, COL_WHITE, value_str);

    // Label below center
    int label_len = 0;
    const char *lp = label;
    while (*lp++) label_len++;
    tft_draw_string(cx - (label_len * FONT_W) / 2, cy + FONT_H / 2 + 2, COL_GRAY, label);
}

static void draw_bar_gauge(int x, int y, int width, int height, float value, float max_value,
                           const char* label, uint8_t color) {
    // Draw border
    box(x, y, width, 2, COL_DARK);
    box(x, y + height - 2, width, 2, COL_DARK);
    box(x, y, 2, height, COL_DARK);
    box(x + width - 2, y, 2, height, COL_DARK);
    
    // Draw filled portion
    float percentage = value / max_value;
    if (percentage > 1.0f) percentage = 1.0f;
    int fill_width = (int)((width - 8) * percentage);
    if (fill_width > 0) {
        box(x + 4, y + 4, fill_width, height - 8, color);
    }
    
    // Draw value text (lean formatting)
    char value_str[16];
    char *p = u32_to_str(value_str, (uint32_t)(value + 0.5f));
    *p = '\0';
    int text_len = (int)(p - value_str);
    tft_draw_string(x + width / 2 - (text_len * FONT_W) / 2, 
                    y + height / 2 - FONT_H / 2, COL_WHITE, value_str);
    
    // Draw label above
    tft_draw_string(x + 4, y - LINE_H, COL_GRAY, label);
}

static void draw_gauge_view(const tft_stats_t *stats) {
    int y = MARGIN;

    // === HEADER with connection status inline ===
    {
        uint8_t cdc_col = stats->cdc_connected ? COL_GREEN : COL_RED;
        uint8_t km_col  = stats->kmbox_connected ? COL_GREEN : COL_RED;

        // Left: connection dots
        box(MARGIN, y + 5, 5, 5, cdc_col);
        box(MARGIN + 8, y + 5, 5, 5, km_col);

        // Center: title
        tft_draw_string_center(TFT_WIDTH / 2, y, COL_CYAN, "KMBox Gauges");

        // Right: uptime
        int ux = TFT_WIDTH - MARGIN - fmt_uptime.len * FONT_W;
        tft_draw_string(ux, y, COL_DARK, fmt_uptime.str);
    }
    y += LINE_H + SEP_GAP;
    hline(y, COL_DARK);
    y += SEP_GAP + 4;

#if (TFT_RAW_WIDTH >= 240)
    // ── Large display (ILI9341 240x320): 3 rows of 2 circular gauges ──
    //   Row 1: Latency + Cmd/s      (performance)
    //   Row 2: Bridge Temp + KMBox Temp  (thermal)
    //   Row 3: TX Rate + RX Rate    (throughput)
    int gauge_radius = 42;
    int col_cx_l = TFT_WIDTH / 4;          // Left column center X
    int col_cx_r = TFT_WIDTH * 3 / 4;      // Right column center X
    int row_h = 100;                        // Vertical spacing per row

    // ── Row 1: Latency + Command Rate ──
    {
        int row_cy = y + gauge_radius;

        // Latency gauge — always shown, color by threshold
        uint8_t lat_col = latency_color(stats->latency_avg_us);
        draw_circular_gauge(col_cx_l, row_cy, gauge_radius,
                            (float)stats->latency_avg_us, 1000.0f, "Lat us", lat_col);

        // Command rate gauge — always shown
        uint8_t cmd_col = (stats->commands_per_sec > 500) ? COL_YELLOW :
                          (stats->commands_per_sec > 0)   ? COL_GREEN  : COL_DARK;
        draw_circular_gauge(col_cx_r, row_cy, gauge_radius,
                            (float)stats->commands_per_sec, 1000.0f, "Cmd/s", cmd_col);

        y += row_h;
    }

    // ── Row 2: Temperature Gauges ──
    {
        int row_cy = y + gauge_radius;

        // Bridge temperature gauge (0-100C range)
        if (TEMP_VALID(stats->bridge_temperature_c)) {
            draw_circular_gauge_unit(col_cx_l, row_cy, gauge_radius,
                                     stats->bridge_temperature_c, 100.0f,
                                     "Bridge", "C",
                                     temp_color(stats->bridge_temperature_c));
        } else {
            // Show empty gauge with "--" placeholder
            draw_circle_ring(col_cx_l, row_cy, gauge_radius - 8, gauge_radius, COL_DARK);
            tft_draw_string(col_cx_l - FONT_W, row_cy - FONT_H / 2, COL_DARK, "--");
            tft_draw_string(col_cx_l - 3 * FONT_W, row_cy + FONT_H / 2 + 2, COL_GRAY, "Bridge");
        }

        // KMBox temperature gauge
        if (stats->kmbox_connected && TEMP_VALID(stats->kmbox_temperature_c)) {
            draw_circular_gauge_unit(col_cx_r, row_cy, gauge_radius,
                                     stats->kmbox_temperature_c, 100.0f,
                                     "KMBox", "C",
                                     temp_color(stats->kmbox_temperature_c));
        } else {
            draw_circle_ring(col_cx_r, row_cy, gauge_radius - 8, gauge_radius, COL_DARK);
            tft_draw_string(col_cx_r - FONT_W, row_cy - FONT_H / 2, COL_DARK, "--");
            tft_draw_string(col_cx_r - 3 * FONT_W / 2, row_cy + FONT_H / 2 + 2, COL_GRAY, "KMBox");
        }

        y += row_h;
    }

    // ── Row 3: TX + RX throughput ──
    {
        int row_cy = y + gauge_radius;
        float tx_kbps = stats->tx_rate_bps / 1000.0f;
        float rx_kbps = stats->rx_rate_bps / 1000.0f;

        uint8_t tx_col = (tx_kbps > 50.0f) ? COL_YELLOW :
                         (tx_kbps > 0.1f)   ? COL_CYAN   : COL_DARK;
        uint8_t rx_col = (rx_kbps > 50.0f) ? COL_YELLOW :
                         (rx_kbps > 0.1f)   ? COL_CYAN   : COL_DARK;

        draw_circular_gauge(col_cx_l, row_cy, gauge_radius,
                            tx_kbps, 100.0f, "TX KB/s", tx_col);
        draw_circular_gauge(col_cx_r, row_cy, gauge_radius,
                            rx_kbps, 100.0f, "RX KB/s", rx_col);
    }

#else
    // ── Small display (ST7735 128x160): horizontal bar gauges ──
    int bar_height = 20;
    int bar_width = TFT_WIDTH - MARGIN * 2;

    // Latency bar (always shown)
    {
        uint8_t lat_col = latency_color(stats->latency_avg_us);
        draw_bar_gauge(MARGIN, y, bar_width, bar_height,
                       (float)stats->latency_avg_us, 1000.0f, "Latency us", lat_col);
        y += bar_height + LINE_H + 2;
    }

    // Command rate bar (always shown)
    {
        uint8_t cmd_col = (stats->commands_per_sec > 0) ? COL_GREEN : COL_DARK;
        draw_bar_gauge(MARGIN, y, bar_width, bar_height,
                       (float)stats->commands_per_sec, 500.0f, "Cmd/s", cmd_col);
        y += bar_height + LINE_H + 2;
    }

    // Temperature bar (bridge)
    if (TEMP_VALID(stats->bridge_temperature_c)) {
        draw_bar_gauge(MARGIN, y, bar_width, bar_height,
                       stats->bridge_temperature_c, 100.0f, "Temp C",
                       temp_color(stats->bridge_temperature_c));
    } else {
        draw_bar_gauge(MARGIN, y, bar_width, bar_height, 0.0f, 100.0f, "Temp C", COL_DARK);
    }
#endif

#if TOUCH_ENABLED
    // Touch zone hints (right edge, centered in each zone)
    {
        int zone_x = TFT_WIDTH - MARGIN - FONT_W;
        int z1_y = (TOUCH_ZONE_TOP_END) / 2 - FONT_H / 2;
        int z2_y = (TOUCH_ZONE_TOP_END + 1 + TOUCH_ZONE_MID_END) / 2 - FONT_H / 2;
        int z3_y = (TOUCH_ZONE_MID_END + 1 + TFT_HEIGHT) / 2 - FONT_H / 2;
        tft_draw_string(zone_x, z1_y, COL_DIM_LINE, "V");
        tft_draw_string(zone_x, z2_y, COL_DIM_LINE, "A");
        tft_draw_string(zone_x, z3_y, COL_DIM_LINE, "H");
    }
#endif
}

// ============================================================================
// Menu View — Humanization Settings
// ============================================================================

static void draw_menu_row(int y, int row_idx, const char *label,
                          const char *value, uint8_t val_color,
                          bool show_arrows) {
    // Highlight row briefly after a tap
    uint8_t bg = COL_BG;
    if (menu_highlight_item == row_idx) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - menu_highlight_time < MENU_HIGHLIGHT_MS) {
            bg = COL_DARK;
        } else {
            menu_highlight_item = -1;
        }
    }

    // Row background
    if (bg != COL_BG) {
        box(MENU_MARGIN, y, TFT_WIDTH - 2 * MENU_MARGIN, MENU_ROW_H - 2, bg);
    }

    // Separator line at top of row
    hline(y, COL_DIM_LINE);

    int text_y = y + (MENU_ROW_H - FONT_H) / 2;  // Vertically centered text

    // Label on left
    tft_draw_string(MENU_MARGIN + 4, text_y, COL_GRAY, label);

    // Value on right
    int val_len = 0;
    const char *vp = value;
    while (*vp++) val_len++;
    int val_x = TFT_WIDTH - MENU_MARGIN - (val_len * FONT_W) - 4;

    if (show_arrows) {
        // Draw < > arrows around value
        tft_draw_string(val_x - 2 * FONT_W, text_y, COL_CYAN, "<");
        tft_draw_string(val_x, text_y, val_color, value);
        tft_draw_string(val_x + val_len * FONT_W + FONT_W, text_y, COL_CYAN, ">");
    } else {
        tft_draw_string(val_x, text_y, val_color, value);
    }
}

static void draw_menu_view(const tft_stats_t *stats) {
    // Sync local menu values from stats on first entry
    if (!menu_values_initialized && stats->humanization_valid) {
        menu_humanization_mode = stats->humanization_mode;
        menu_inject_mode = stats->inject_mode;
        menu_max_per_frame = stats->max_per_frame;
        menu_velocity_matching = stats->velocity_matching;
        menu_jitter_enabled = stats->jitter_enabled;
        menu_values_initialized = true;
    }

    int y = MARGIN;

    // === HEADER ===
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_CYAN, "Humanization");
    y += LINE_H;
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_DARK, "Tap to adjust");
    y += LINE_H + SEP_GAP;
    hline(y, COL_CYAN);
    y = MENU_HEADER_H;

    // === MENU ROWS ===
    // Row 0: Humanization Mode (Off / Micro / Full)
    {
        static const char *mode_names[] = { "Off", "Micro", "Full" };
        const char *val = (menu_humanization_mode < 3) ? mode_names[menu_humanization_mode] : "?";
        uint8_t col = hmode_color(menu_humanization_mode);
        draw_menu_row(y, 0, "Mode", val, col, true);
        y += MENU_ROW_H;
    }

    // Row 1: Inject Mode (Immediate / Smooth / VelMatch / Micro)
    {
        static const char *inject_names[] = { "Immed", "Smooth", "VelMat", "Micro" };
        const char *val = (menu_inject_mode < 4) ? inject_names[menu_inject_mode] : "?";
        draw_menu_row(y, 1, "Inject", val, COL_WHITE, true);
        y += MENU_ROW_H;
    }

    // Row 2: Max Per Frame (1-32)
    {
        char mpf_str[8];
        char *p = u32_to_str(mpf_str, menu_max_per_frame);
        *p++ = ' '; *p++ = 'p'; *p++ = 'x'; *p = '\0';
        uint8_t col = COL_WHITE;
        if (menu_max_per_frame <= 4) col = COL_YELLOW;
        else if (menu_max_per_frame >= 24) col = COL_RED;
        draw_menu_row(y, 2, "MaxPF", mpf_str, col, true);
        y += MENU_ROW_H;
    }

    // Row 3: Velocity Matching (On / Off)
    {
        const char *val = menu_velocity_matching ? "On" : "Off";
        uint8_t col = menu_velocity_matching ? COL_GREEN : COL_RED;
        draw_menu_row(y, 3, "VelMatch", val, col, false);
        y += MENU_ROW_H;
    }

    // Row 4: Jitter/Tremor (On / Off)
    {
        const char *val = menu_jitter_enabled ? "On" : "Off";
        uint8_t col = menu_jitter_enabled ? COL_GREEN : COL_RED;
        draw_menu_row(y, 4, "Jitter", val, col, false);
        y += MENU_ROW_H;
    }

    // Separator after last item
    hline(y, COL_DIM_LINE);
    y += SECTION_GAP + 4;

    // === LIVE STATUS (read-only, from stats) ===
    tft_draw_string(MENU_MARGIN, y, COL_CYAN, "Live Status");
    y += LINE_H + 2;

    // Queue depth bar
    if (stats->humanization_valid) {
        tft_draw_string(MENU_MARGIN, y, COL_GRAY, "Queue");
        // Draw visual bar
        int bar_x = MENU_MARGIN + 6 * FONT_W + 4;
        int bar_w = TFT_WIDTH - bar_x - MENU_MARGIN - 40;
        int bar_h = FONT_H - 2;
        box(bar_x, y + 1, bar_w, bar_h, COL_DARK);
        if (stats->queue_capacity > 0) {
            int fill = (int)((float)stats->queue_depth / stats->queue_capacity * bar_w);
            if (fill > bar_w) fill = bar_w;
            uint8_t bar_col = COL_GREEN;
            if (stats->queue_depth > stats->queue_capacity * 3 / 4) bar_col = COL_RED;
            else if (stats->queue_depth > stats->queue_capacity / 2) bar_col = COL_YELLOW;
            if (fill > 0) box(bar_x, y + 1, fill, bar_h, bar_col);
        }
        // Numeric value
        char qstr[8];
        char *qp = u32_to_str(qstr, stats->queue_depth);
        *qp++ = '/';
        qp = u32_to_str(qp, stats->queue_capacity);
        *qp = '\0';
        tft_draw_string(TFT_WIDTH - MENU_MARGIN - 5 * FONT_W, y, COL_WHITE, qstr);
        y += LINE_H + 2;
    }

    // Total injected
    if (stats->total_injected > 0) {
        tft_draw_string(MENU_MARGIN, y, COL_GRAY, "Injected");
        char inj_str[12];
        char *ip = u32_to_str(inj_str, stats->total_injected);
        *ip = '\0';
        int ix = TFT_WIDTH - MENU_MARGIN - (int)(ip - inj_str) * FONT_W;
        tft_draw_string(ix, y, COL_WHITE, inj_str);
        y += LINE_H + 2;
    }

    // Queue overflows
    if (stats->queue_overflows > 0) {
        tft_draw_string(MENU_MARGIN, y, COL_GRAY, "Overflow");
        char ovf_str[12];
        char *op = u32_to_str(ovf_str, stats->queue_overflows);
        *op = '\0';
        int ox = TFT_WIDTH - MENU_MARGIN - (int)(op - ovf_str) * FONT_W;
        tft_draw_string(ox, y, COL_RED, ovf_str);
        y += LINE_H + 2;
    }

    // Connection + temp at bottom
    y = TFT_HEIGHT - LINE_H * 2 - MARGIN;
    hline(y, COL_DIM_LINE);
    y += 4;
    {
        uint8_t km_col = stats->kmbox_connected ? COL_GREEN : COL_RED;
        tft_draw_string(MENU_MARGIN, y, COL_GRAY, "KMBox");
        tft_draw_string(MENU_MARGIN + 6 * FONT_W, y, km_col,
                        stats->kmbox_connected ? "OK" : "--");
        // Uptime right-aligned
        int ux = TFT_WIDTH - MARGIN - fmt_uptime.len * FONT_W;
        tft_draw_string(ux, y, COL_CYAN, fmt_uptime.str);
        y += LINE_H;
    }
    if (fmt_br_temp.len) {
        tft_draw_string(MENU_MARGIN, y, COL_GRAY, "Temp");
        tft_draw_string(MENU_MARGIN + 5 * FONT_W, y, 
                        temp_color(stats->bridge_temperature_c), fmt_br_temp.str);
    }
}

// ============================================================================
// Public API
// ============================================================================

// Keep alarm IRQ work bounded. Formatting and drawing run in the main loop.
static bool tft_render_timer_callback(repeating_timer_t *rt) {
    (void)rt;
    render_due = true;
    return true;
}

bool tft_display_init(void) {
    if (initialized) return true;
    
    picotft_init();
    tft_fill(COL_BG);
    tft_swap_sync();
    
    // PWM backlight
    gpio_set_function(TFT_BL_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(TFT_BL_PIN), 255);
    pwm_set_enabled(slice, true);
    
#if TOUCH_ENABLED
    // Initialize FT6206 capacitive touch controller over I2C
    // SDA = Arduino A4 (GPIO20), SCL = Arduino A5 (GPIO21)
    if (ft6206_init(i2c0, BRIDGE_TOUCH_SDA_PIN, BRIDGE_TOUCH_SCL_PIN)) {
        ft6206_set_callback(touch_event_callback);
        printf("[TFT] FT6206 capacitive touch initialized (SDA=%d SCL=%d)\n",
               BRIDGE_TOUCH_SDA_PIN, BRIDGE_TOUCH_SCL_PIN);
    } else {
        printf("[TFT] FT6206 touch controller not detected\n");
    }
#endif
    
    initialized = true;
    
    // Start background render timer (100ms = 10 FPS)
    // Negative interval means the timer fires every 100ms regardless of callback duration.
    add_repeating_timer_ms(-UPDATE_INTERVAL_MS, tft_render_timer_callback, NULL, &tft_render_timer);
    
    return true;
}

// ============================================================================
// Main-loop rendering, scheduled by the lightweight alarm callback.
// ============================================================================

static void tft_render_frame(void) {
    if (!initialized) return;
    
    // Skip if the previous frame hasn't been flushed yet
    if (frame_ready) return;
    
    // Stats submission and rendering both run on the main loop.
    if (!stats_pending) return;
    tft_stats_t local_stats = shared_stats;  // Snapshot
    stats_pending = false;

    // Skip render if stats haven't meaningfully changed (saves CPU in steady state)
    uint32_t h = stats_quick_hash(&local_stats);
    if (h == last_stats_hash && menu_highlight_item < 0) return;
    last_stats_hash = h;

    // Render into the back buffer
    tft_fill(COL_BG);
    format_stats(&local_stats);
    
    if (local_stats.console_mode) {
        draw_console_view(&local_stats);
    } else if (current_view == TFT_VIEW_MENU) {
        draw_menu_view(&local_stats);
    } else if (current_view == TFT_VIEW_GAUGES) {
        draw_gauge_view(&local_stats);
    } else {
        draw_stats(&local_stats);
    }

    // Track activity for next frame
    last_tx_bytes = local_stats.tx_bytes;
    last_rx_bytes = local_stats.rx_bytes;
    
    // Signal main loop that a frame is ready for DMA
    frame_ready = true;
    return;
}

// ============================================================================
// Public API — lightweight calls for the main loop
// ============================================================================

void tft_display_submit_stats(const tft_stats_t *stats) {
    // Keep the latest snapshot for the next main-loop render.
    shared_stats = *stats;
    stats_pending = true;
}

bool tft_display_flush(void) {
    if (!initialized) return false;
    if (transfer_pending) {
        if (!tft_sync_poll()) return false;
        transfer_pending = false;
        frame_ready = false;
        return true;
    }
    if (render_due && !frame_ready) {
        render_due = false;
        tft_render_frame();
    }
    if (!frame_ready) return false;
    tft_swap_buffers();
    transfer_pending = tft_sync_start();
    return false;
}

void tft_display_update(const tft_stats_t *stats) {
    // Convenience wrapper: submit + flush in one call.
    // For best performance, use submit_stats/flush separately so the main
    // loop can do other work between submitting stats and flushing DMA.
    tft_display_submit_stats(stats);
    tft_display_flush();
}

void tft_display_refresh(const tft_stats_t *stats) {
    if (!initialized) return;
    
    // Force an immediate synchronous render (bypasses timer).
    tft_fill(COL_BG);
    format_stats(stats);
    
    if (stats->console_mode) {
        draw_console_view(stats);
    } else if (current_view == TFT_VIEW_MENU) {
        draw_menu_view(stats);
    } else if (current_view == TFT_VIEW_GAUGES) {
        draw_gauge_view(stats);
    } else {
        draw_stats(stats);
    }
    
    tft_swap_sync();
    
    last_tx_bytes = stats->tx_bytes;
    last_rx_bytes = stats->rx_bytes;
    frame_ready = false;
    transfer_pending = false;
}

void tft_display_splash(void) {
    if (!initialized) return;

    tft_fill(COL_BG);

    int cx = TFT_WIDTH / 2;
    int cy = TFT_HEIGHT / 2;

    // Outer frame with double border
    int bx = 8, by = cy - LINE_H * 3;
    int bw = TFT_WIDTH - 16, bh = LINE_H * 5 + 12;
    box(bx, by, bw, 2, COL_CYAN);                     // top
    box(bx, by + bh - 2, bw, 2, COL_CYAN);            // bottom
    box(bx, by, 2, bh, COL_CYAN);                      // left
    box(bx + bw - 2, by, 2, bh, COL_CYAN);             // right
    // Inner fill
    box(bx + 4, by + 4, bw - 8, bh - 8, COL_DARK);

    int y = by + 8;
    tft_draw_string_center(cx, y, COL_CYAN, "KMBox Bridge");
    y += LINE_H + 2;

    // Thin accent line
    int line_x = cx - 40;
    int line_w = 80;
    if (line_x < bx + 6) { line_x = bx + 6; line_w = bw - 12; }
    box(line_x, y, line_w, 1, COL_CYAN);
    y += 6;

    tft_draw_string_center(cx, y, COL_WHITE, "Autopilot");
    y += LINE_H + LINE_H;
    tft_draw_string_center(cx, y, COL_GREEN, "Starting...");

    tft_swap_sync();
}

void tft_display_error(const char *msg) {
    if (!initialized) return;
    
    tft_fill(COL_BG);
    
    int y = TFT_HEIGHT / 2 - LINE_H;
    box(5, y - 4, TFT_WIDTH - 10, LINE_H * 2 + 8, COL_RED);
    
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_WHITE, "ERROR");
    y += LINE_H;
    tft_draw_string_center(TFT_WIDTH / 2, y, COL_WHITE, msg);
    
    tft_swap_sync();
}

void tft_display_backlight(uint8_t brightness) {
    if (!initialized) return;
    uint slice = pwm_gpio_to_slice_num(TFT_BL_PIN);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(TFT_BL_PIN), brightness);
}

tft_view_mode_t tft_display_get_view(void) {
    return current_view;
}

void tft_display_set_view(tft_view_mode_t mode) {
    current_view = mode;
}

void tft_display_toggle_view(void) {
    switch (current_view) {
        case TFT_VIEW_DETAILED:
            current_view = TFT_VIEW_GAUGES;
            break;
        case TFT_VIEW_GAUGES:
#if TOUCH_ENABLED
            current_view = TFT_VIEW_MENU;
            menu_values_initialized = false;  // Re-sync from live stats
            break;
        case TFT_VIEW_MENU:
#endif
            current_view = TFT_VIEW_DETAILED;
            break;
        default:
            current_view = TFT_VIEW_DETAILED;
            break;
    }
}

void tft_display_handle_touch(void) {
#if TOUCH_ENABLED
    // Poll FT6206 for touch events (handles debouncing internally)
    ft6206_poll();
    
    // Process zone-based touch flags
    if (touch_view_requested) {
        touch_view_requested = false;
        tft_display_toggle_view();
    }
    
    if (touch_api_requested) {
        touch_api_requested = false;
        // Signal API mode cycle request (checked by bridge main loop)
        extern volatile bool api_cycle_requested;
        api_cycle_requested = true;
    }
    
    if (touch_human_requested) {
        touch_human_requested = false;
        // Signal humanization cycle request (checked by bridge main loop)
        extern volatile bool humanization_cycle_requested;
        humanization_cycle_requested = true;
    }
#endif
}