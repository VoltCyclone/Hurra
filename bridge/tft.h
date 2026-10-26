/*
 * Copyright (C) 2022 Jan Hamal Dvořák <mordae@anilinux.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#pragma once
#define TFT_DRIVER_ST7735 1
#define TFT_DRIVER_ILI9225 2
#define TFT_DRIVER_ILI9341 3

#include <pico/stdlib.h>
#include "tft_config.h"  // Include our configuration

#if !defined(TFT_SCALE)
#define TFT_SCALE 1
#elif (TFT_SCALE != 1) && (TFT_SCALE != 2) && (TFT_SCALE != 4)
#error TFT_SCALE must be 1, 2, or 4
#endif

#if !defined(TFT_SWAP_XY)
#define TFT_SWAP_XY 0
#endif

#if !defined(TFT_VSYNC)
#define TFT_VSYNC 0
#endif

#if TFT_DRIVER == TFT_DRIVER_ST7735
#define TFT_RAW_WIDTH 128
#define TFT_RAW_HEIGHT 160
// ST7735 driver enabled for KMBox Bridge
#elif TFT_DRIVER == TFT_DRIVER_ILI9225
#define TFT_RAW_WIDTH 176
#define TFT_RAW_HEIGHT 220
#error "ILI9225 driver is currently broken"
#elif TFT_DRIVER == TFT_DRIVER_ILI9341
#define TFT_RAW_WIDTH 240
#define TFT_RAW_HEIGHT 320
#endif

#if TFT_SWAP_XY
#define TFT_WIDTH (int)(TFT_RAW_HEIGHT / TFT_SCALE)
#define TFT_HEIGHT (int)(TFT_RAW_WIDTH / TFT_SCALE)
#else
#define TFT_WIDTH (int)(TFT_RAW_WIDTH / TFT_SCALE)
#define TFT_HEIGHT (int)(TFT_RAW_HEIGHT / TFT_SCALE)
#endif

#define TFT_RIGHT (TFT_WIDTH - 1)
#define TFT_BOTTOM (TFT_HEIGHT - 1)

/*
 * Modified latin 16x8 bitmap font.
 * Populate it with tft_load_font().
 */
extern uint8_t tft_font[256 * 16];

/*
 * Color palette mapping into RGB565.
 */
extern uint16_t tft_palette[256];

/*
 * Initialize the screen.
 * Renamed to picotft_init() to avoid conflict with tft_display.h wrapper
 */
void picotft_init(void);

/*
 * Swap buffers to allow for a background sync.
 */
void tft_swap_buffers(void);

/*
 * Start display synchronization cycle.
 * Blocks until it's done so the you do not clobber the back buffer.
 */
void tft_sync(void);
// Cooperative row-DMA refresh (software LUT path). Poll once per main-loop pass.
bool tft_sync_start(void);
bool tft_sync_poll(void);

/* Shortcut to swap buffers and sync at once. */
void tft_swap_sync(void);

/*
 * Here goes whatever should be on the screen.
 * First come the Y rows, then the X columns.
 */
extern uint8_t *tft_input;

/* Clipping rectangle. */
extern int tft_clip_x0;
extern int tft_clip_y0;
extern int tft_clip_x1;
extern int tft_clip_y1;

/*
 * Set area to restrict painting to.
 * Unlike with tft_draw_rect, the x1, y1 bounds are non-inclusive.
 * Clipping applies to screen coordinates and ignores origin.
 */
inline static void __unused tft_set_clip(int x0, int y0, int x1, int y1)
{
	tft_clip_x0 = x0 < 0 ? 0 : x0;
	tft_clip_y0 = y0 < 0 ? 0 : y0;
	tft_clip_x1 = x1 > TFT_WIDTH ? TFT_WIDTH : x1;
	tft_clip_y1 = y1 > TFT_HEIGHT ? TFT_HEIGHT : y1;
}

/* Clear clipping to include whole display. */
inline static void __unused tft_clear_clip(void)
{
	tft_clip_x0 = 0;
	tft_clip_y0 = 0;
	tft_clip_x1 = TFT_WIDTH;
	tft_clip_y1 = TFT_HEIGHT;
}

/* Color a single pixel. */
inline static void __unused tft_draw_pixel_absolute(int ax, int ay, int color)
{
	if ((ax < tft_clip_x0) || (ax >= tft_clip_x1))
		return;

	if ((ay < tft_clip_y0) || (ay >= tft_clip_y1))
		return;

#if TFT_SWAP_XY
	int i = ax * TFT_HEIGHT + ay;
#else
	int i = ay * TFT_WIDTH + ax;
#endif
	tft_input[i] = color;
}

/* Origin for relative drawing. */
extern int tft_origin_x;
extern int tft_origin_y;

/* Set origin for relative drawing. */
inline static void __unused tft_set_origin(int x, int y)
{
	tft_origin_x = x;
	tft_origin_y = y;
}

/* Color a single pixel, relative to origin. */
inline static void __unused tft_draw_pixel(int x, int y, int color)
{
	x -= tft_origin_x;
	y -= tft_origin_y;
	tft_draw_pixel_absolute(x, y, color);
}

#define rgb565(r, g, b) ((((r) & 31) << 11) | (((g) & 63) << 5) | ((b) & 31))
#define rgb332(r, g, b) ((((r) & 7) << 5) | (((g) & 7) << 2) | ((b) & 3))

#define rgb565_just_red(x) ((x) & 0xf800)
#define rgb565_just_green(x) ((x) & 0x07e0)
#define rgb565_just_blue(x) ((x) & 0x1f)

#define rgb565_red(x) ((rgb565_just_red((x)) >> 8) | (rgb565_just_red((x)) >> 13))
#define rgb565_green(x) ((rgb565_just_green((x)) >> 3) | (rgb565_just_green((x)) >> 9))
#define rgb565_blue(x) ((rgb565_just_blue((x)) << 3) | (rgb565_just_blue((x)) >> 2))

#define rgb332_just_red(x) ((x) & 0xe0)
#define rgb332_just_green(x) ((x) & 0x1c)
#define rgb332_just_blue(x) ((x) & 0x03)

#define rgb332_red(x) \
	(rgb332_just_red((x)) | (rgb332_just_red((x)) >> 3) | (rgb332_just_red((x)) >> 6))
#define rgb332_green(x) \
	((rgb332_just_green((x)) << 3) | rgb332_just_green((x)) | (rgb332_just_green((x)) >> 3))
#define rgb332_blue(x)                                                 \
	((rgb332_just_blue((x)) << 6) | (rgb332_just_blue((x)) << 4) | \
	 (rgb332_just_blue((x)) << 2) | rgb332_just_blue((x)))

#define rgb_to_rgb565(r, g, b) ((((r) >> 3) << 11) | (((g) >> 2) << 5) | ((b) >> 3))
#define rgb_to_rgb332(r, g, b) ((((r) >> 5) << 5) | (((g) >> 5) << 2) | ((b) >> 6))

#define rgb332_to_rgb565(x) rgb_to_rgb565(rgb332_red((x)), rgb332_green((x)), rgb332_blue((x)))
#define rgb565_to_rgb332(x) rgb_to_rgb332(rgb565_red((x)), rgb565_green((x)), rgb565_blue((x)))

/*
 * Color a whole rect of pixels.
 */
void tft_draw_rect(int x0, int y0, int x1, int y1, int color);

/*
 * Paint the whole screen with a single color.
 */
void tft_fill(int color);

/*
 * Draw a simple sprite, starting from top-left coordinates.
 * Transparency indicates what color to treat as transparent.
 */
void tft_draw_sprite(int x, int y, int w, int h, const uint8_t *data, int trsp);

/*
 * Draw a flipped sprite, starting from top-left coordinates.
 * Transparency indicates what color to treat as transparent.
 * Combination of flips and swap allows for 90° rotations.
 */
void tft_draw_sprite_flipped(int x, int y, int w, int h, const uint8_t *data, int trsp, bool flip_x,
			     bool flip_y, bool swap_xy);

/*
 * Draw given glyph at specified coordinates.
 * The coordinates indicate bottom left of the glyph.
 */
void tft_draw_glyph(int x, int y, int color, char c);

/*
 * Draw given string at specified coordinates.
 *
 * The coordinates indicate bottom left of the string for the tft_draw_string,
 * bottom right for the tft_draw_string_right and bottom center for the
 * tft_draw_string_center.
 */
void tft_draw_string(int x, int y, int color, const char *str);
void tft_draw_string_right(int x, int y, int color, const char *str);
void tft_draw_string_center(int x, int y, int color, const char *str);

/*
 * Write directly into a control register.
 */
void tft_control(uint8_t reg, const uint8_t *bstr, int len);

#if defined(TFT_MISO_PIN)
/*
 * Read directly from a control register.
 */
void tft_read(uint8_t reg, uint8_t *bstr, int len, int delay_us);
#endif

/*
 * Weak symbol that can be replaced with a version that yields to
 * other tasks while the DMA transfer is pending.
 */
void tft_dma_channel_wait_for_finish_blocking(int dma_ch);
