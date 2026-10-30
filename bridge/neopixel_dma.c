/**
 * @file neopixel_dma.c
 * @brief Zero-CPU NeoPixel (WS2812) driver for KMBox Bridge
 *
 * All color computation (breathing, rainbow, activity flash) happens inside a
 * 30 Hz repeating-timer ISR.  The ISR writes a pre-shifted GRB word into a
 * shadow register, then kicks a 1-word DMA transfer into the PIO TX FIFO.
 *
 * The main loop only ever does a volatile store to request a status change —
 * zero PIO / DMA / floating-point work on the hot path.
 */

#include "neopixel_dma.h"
#include "config.h"
#include "led_color.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "ws2812.pio.h"

#include <string.h>

// ── Internal constants ──────────────────────────────────────────────

#define NEO_REFRESH_MS      33      // ~30 Hz
#define NEO_ACTIVITY_MS     100     // Activity flash duration (ms)
#define NEO_BREATHING_MIN   40      // Minimum brightness (0-255)
#define NEO_BREATHING_MAX   255     // Maximum brightness (0-255)
#define NEO_BREATHING_STEP  3       // Brightness change per tick (~30 Hz)
#define NEO_RAINBOW_STEP    2       // Hue increment per tick (0-359)

// ── Status → color table ────────────────────────────────────────────

typedef struct {
    uint8_t r, g, b;
    neo_effect_t default_effect;
} neo_status_cfg_t;

static const neo_status_cfg_t s_status_table[NEO_STATUS_COUNT] = {
    [NEO_STATUS_BOOTING]       = { .r =   0, .g =   0, .b = 255, .default_effect = NEO_EFFECT_BREATHING },
    [NEO_STATUS_IDLE]          = { .r = 255, .g =   0, .b =   0, .default_effect = NEO_EFFECT_BREATHING },
    [NEO_STATUS_CDC_CONNECTED] = { .r =   0, .g = 255, .b = 255, .default_effect = NEO_EFFECT_NONE      },
    [NEO_STATUS_TRACKING]      = { .r =   0, .g = 255, .b =   0, .default_effect = NEO_EFFECT_NONE      },
    [NEO_STATUS_DISABLED]      = { .r = 255, .g = 255, .b =   0, .default_effect = NEO_EFFECT_NONE      },
    [NEO_STATUS_ERROR]         = { .r = 255, .g =   0, .b =   0, .default_effect = NEO_EFFECT_BREATHING },
};

// ── Module state (accessed from ISR + main — volatile where needed) ─

static PIO          s_pio;
static uint         s_sm;
static uint         s_offset;
static int          s_dma_chan = -1;
static struct repeating_timer s_timer;

// Shadow register: pre-shifted GRB word ready for PIO.
// Written by the timer ISR, read by DMA.
static volatile uint32_t s_shadow_grb __attribute__((aligned(4))) = 0;

// Requested state (written by main, read by ISR)
static volatile neo_status_t s_status       = NEO_STATUS_BOOTING;
static volatile neo_effect_t s_effect_override = NEO_EFFECT_NONE;
static volatile bool         s_effect_overridden = false;

// One-shot RGB override (set by neopixel_dma_set_rgb / activity flash)
static volatile bool    s_rgb_override    = false;
static volatile uint8_t s_ovr_r, s_ovr_g, s_ovr_b;

// Activity flash
static volatile bool     s_activity_active = false;
static volatile uint32_t s_activity_end_us = 0;
static volatile uint8_t  s_act_r, s_act_g, s_act_b;

// ISR-local effect state (not shared — only touched inside the callback)
static uint8_t  s_breath_brightness = NEO_BREATHING_MAX;
static int8_t   s_breath_dir        = -NEO_BREATHING_STEP;
static uint16_t s_rainbow_hue       = 0;

// ── Helpers ─────────────────────────────────────────────────────────

/** Pack RGB into PIO-ready GRB<<8 format. */
static inline uint32_t rgb_to_pio(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    return grb << 8;  // WS2812 PIO expects data in top 24 bits
}

// HSV→RGB and brightness scaling provided by shared lib/led-utils/led_color.h
// Local aliases for shorter call sites
#define apply_brightness  led_apply_brightness_u8
#define hsv_to_rgb        led_hsv_to_rgb

// ── Timer ISR (~30 Hz) ──────────────────────────────────────────────

static bool neo_timer_callback(struct repeating_timer *t) {
    (void)t;
    uint32_t now_us = time_us_32();

    // ── 1. Activity flash (highest priority) ────────────────────────
    if (s_activity_active) {
        if ((int32_t)(now_us - s_activity_end_us) >= 0) {
            s_activity_active = false;  // expired
        } else {
            s_shadow_grb = rgb_to_pio(s_act_r, s_act_g, s_act_b);
            goto dma_kick;
        }
    }

    // ── 2. One-shot RGB override ────────────────────────────────────
    if (s_rgb_override) {
        s_rgb_override = false;
        s_shadow_grb = rgb_to_pio(s_ovr_r, s_ovr_g, s_ovr_b);
        goto dma_kick;
    }

    // ── 3. Status-driven color + effect ─────────────────────────────
    {
        neo_status_t st = s_status;
        if (st >= NEO_STATUS_COUNT) st = NEO_STATUS_ERROR;

        const neo_status_cfg_t *cfg = &s_status_table[st];
        uint8_t r = cfg->r, g = cfg->g, b = cfg->b;

        // Choose active effect (override takes precedence)
        neo_effect_t eff = s_effect_overridden ? s_effect_override : cfg->default_effect;

        switch (eff) {
            case NEO_EFFECT_BREATHING: {
                // Update breathing brightness (triangle wave)
                int16_t next = (int16_t)s_breath_brightness + s_breath_dir;
                if (next <= NEO_BREATHING_MIN) {
                    next = NEO_BREATHING_MIN;
                    s_breath_dir = NEO_BREATHING_STEP;
                } else if (next >= NEO_BREATHING_MAX) {
                    next = NEO_BREATHING_MAX;
                    s_breath_dir = -NEO_BREATHING_STEP;
                }
                s_breath_brightness = (uint8_t)next;
                apply_brightness(&r, &g, &b, s_breath_brightness);
                break;
            }
            case NEO_EFFECT_RAINBOW: {
                s_rainbow_hue = (s_rainbow_hue + NEO_RAINBOW_STEP) % 360;
                hsv_to_rgb(s_rainbow_hue, 255, 200, &r, &g, &b);
                break;
            }
            case NEO_EFFECT_NONE:
            default:
                break;  // solid status color
        }

        s_shadow_grb = rgb_to_pio(r, g, b);
    }

dma_kick:
    // Fire-and-forget DMA: 1 word from shadow register → PIO TX FIFO.
    // CRITICAL: re-arm transfer count to 1 before each trigger — after the
    // previous transfer completes, trans_count is 0 and triggering does nothing.
    if (s_dma_chan >= 0 && !dma_channel_is_busy(s_dma_chan)) {
        dma_channel_set_trans_count(s_dma_chan, 1, false);
        dma_channel_set_read_addr(s_dma_chan, (const volatile void *)&s_shadow_grb, true);
    } else {
        // DMA busy or unavailable — fall back to direct PIO write (still non-blocking)
        if (!pio_sm_is_tx_fifo_full(s_pio, s_sm)) {
            pio_sm_put(s_pio, s_sm, s_shadow_grb);
        }
    }
    return true;  // keep repeating
}

// ── Public API ──────────────────────────────────────────────────────

void neopixel_dma_init(PIO pio, uint pin) {
    s_pio = pio;

    // Load WS2812 PIO program
    s_offset = pio_add_program(s_pio, &ws2812_program);
    s_sm = pio_claim_unused_sm(s_pio, true);
    ws2812_program_init(s_pio, s_sm, s_offset, pin, 800000, false);

    // ── Claim DMA channel ───────────────────────────────────────────
    s_dma_chan = dma_claim_unused_channel(false);
    if (s_dma_chan >= 0) {
        dma_channel_config c = dma_channel_get_default_config(s_dma_chan);
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(s_pio, s_sm, true));

        dma_channel_configure(
            s_dma_chan,
            &c,
            &s_pio->txf[s_sm],                          // write: PIO TX FIFO
            (const volatile void *)&s_shadow_grb,        // read:  shadow register
            1,                                            // 1 word per transfer
            false                                         // don't start yet
        );
    }

    // ── Set initial color (booting blue) ────────────────────────────
    s_shadow_grb = rgb_to_pio(0, 0, 255);
    // Push one frame immediately so the LED lights up before the timer fires
    if (!pio_sm_is_tx_fifo_full(s_pio, s_sm)) {
        pio_sm_put(s_pio, s_sm, s_shadow_grb);
    }

    // ── Start 30 Hz repeating timer (runs from hardware alarm pool) ─
    add_repeating_timer_ms(-NEO_REFRESH_MS, neo_timer_callback, NULL, &s_timer);
}

void neopixel_dma_set_status(neo_status_t status) {
    if (status < NEO_STATUS_COUNT) {
        s_status = status;
    }
}

void neopixel_dma_set_effect(neo_effect_t effect) {
    if (effect == NEO_EFFECT_NONE) {
        s_effect_overridden = false;
    } else {
        s_effect_override = effect;
        s_effect_overridden = true;
    }
}

void neopixel_dma_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    s_ovr_r = r;
    s_ovr_g = g;
    s_ovr_b = b;
    s_rgb_override = true;
}

void neopixel_dma_activity_flash(uint8_t r, uint8_t g, uint8_t b) {
    neopixel_dma_activity_flash_ms(r, g, b, NEO_ACTIVITY_MS);
}

void neopixel_dma_activity_flash_ms(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms) {
    s_act_r = r;
    s_act_g = g;
    s_act_b = b;
    s_activity_end_us = time_us_32() + (duration_ms * 1000);
    s_activity_active = true;
}

void neopixel_dma_get_hw(PIO *out_pio, uint *out_sm) {
    if (out_pio) *out_pio = s_pio;
    if (out_sm)  *out_sm  = s_sm;
}
