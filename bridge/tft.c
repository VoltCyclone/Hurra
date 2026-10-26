/*
 * Copyright Jan Hamal Dvořák <mordae@anilinux.org>
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

#include <pico/stdlib.h>

#include <hardware/spi.h>
#include <hardware/dma.h>

#include "tft.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(TFT_RST_DELAY)
#define TFT_RST_DELAY 50
#endif

#if !defined(TFT_HW_ACCEL)
#define TFT_HW_ACCEL 0
#endif

#if !defined(TFT_PIO)
#define TFT_PIO pio0
#endif

#if !defined(__weak)
#define __weak __attribute__((__weak__))
#endif

#if TFT_HW_ACCEL
#include "lut.pio.h"
#endif

/*
 * We are using double buffering.
 *
 * One buffer is being written to by the client.
 * The other buffer is being transmitted.
 *
 * After every cycle the buffers are rotated.
 */
static uint8_t buffer[2][TFT_HEIGHT * TFT_WIDTH];

#if TFT_HW_ACCEL
#include "tft_dma_row_script.h"
#else
/*
 * Per-row transfer buffers. These are also two.
 * We are writing into one while the other is read by DMA.
 */
static uint16_t txbuf[2][TFT_RAW_WIDTH];
#endif

/* Currently inactive buffer that is to be sent to the display. */
uint8_t *tft_committed;

/* Current active buffer that is to be written into. */
uint8_t *tft_input;

/* Clipping rectangle. */
int tft_clip_x0 = 0;
int tft_clip_y0 = 0;
int tft_clip_x1 = TFT_WIDTH;
int tft_clip_y1 = TFT_HEIGHT;

/* Origin for relative drawing. */
int tft_origin_x = 0;
int tft_origin_y = 0;

/* Font data. */
extern uint8_t tft_font[256 * 16];

/* Palette colors. */
uint16_t __attribute__((__aligned__(512))) tft_palette[256] = {
	0x0000, 0x1082, 0x2104, 0x31a6, 0x4228, 0x52aa, 0x632c, 0x73ae, 0x8c51, 0x9cd3, 0xad55,
	0xbdd7, 0xce59, 0xdefb, 0xef7d, 0xffff, 0x20a2, 0x20e2, 0x2102, 0x1902, 0x1102, 0x1102,
	0x1103, 0x1104, 0x10c4, 0x10a4, 0x1084, 0x1884, 0x2084, 0x2083, 0x2083, 0x2082, 0x2081,
	0x20c1, 0x2101, 0x1101, 0x0901, 0x0902, 0x0903, 0x0904, 0x08c4, 0x0884, 0x0844, 0x1044,
	0x1844, 0x2043, 0x2042, 0x2041, 0x2060, 0x20c0, 0x1900, 0x1100, 0x0900, 0x0101, 0x0102,
	0x0104, 0x00a4, 0x0044, 0x0004, 0x1004, 0x1804, 0x2003, 0x2002, 0x2000, 0x41a6, 0x41e6,
	0x4206, 0x3a06, 0x3206, 0x3206, 0x3207, 0x3208, 0x31c8, 0x31a8, 0x3188, 0x3988, 0x3988,
	0x4187, 0x4187, 0x4186, 0x4164, 0x41c4, 0x3a04, 0x3204, 0x2204, 0x2205, 0x2206, 0x2208,
	0x21a8, 0x2148, 0x2108, 0x3108, 0x3908, 0x4107, 0x4105, 0x4104, 0x4102, 0x41a2, 0x3a02,
	0x2a02, 0x1a02, 0x1203, 0x1205, 0x1208, 0x1168, 0x10e8, 0x1888, 0x2888, 0x3888, 0x4086,
	0x4084, 0x4082, 0x40c0, 0x4180, 0x3a00, 0x2200, 0x0a00, 0x0202, 0x0205, 0x0208, 0x0148,
	0x0088, 0x0808, 0x2008, 0x3808, 0x4006, 0x4003, 0x4000, 0x834c, 0x83ac, 0x7c0c, 0x740c,
	0x640c, 0x640d, 0x640e, 0x640f, 0x63b0, 0x6350, 0x6310, 0x7310, 0x7b10, 0x830f, 0x830d,
	0x830c, 0x82a8, 0x8368, 0x7c08, 0x6408, 0x4c08, 0x440a, 0x440c, 0x440f, 0x4350, 0x4290,
	0x4a10, 0x6210, 0x7210, 0x820e, 0x820b, 0x8208, 0x8204, 0x8324, 0x7404, 0x5404, 0x3404,
	0x2407, 0x240b, 0x240f, 0x22f0, 0x21d0, 0x2910, 0x5110, 0x7110, 0x810d, 0x8108, 0x8104,
	0x8160, 0x82e0, 0x7400, 0x4400, 0x1400, 0x0404, 0x0409, 0x040f, 0x0290, 0x0110, 0x1010,
	0x4010, 0x6810, 0x800c, 0x8006, 0x8000, 0xfeb7, 0xff77, 0xf7f7, 0xdff7, 0xc7f7, 0xbff9,
	0xbffc, 0xbfff, 0xbf3f, 0xbe7f, 0xc5ff, 0xddff, 0xf5ff, 0xfdfd, 0xfdfa, 0xfdf7, 0xfd70,
	0xfef0, 0xeff0, 0xbff0, 0x97f0, 0x87f3, 0x87f9, 0x87ff, 0x869f, 0x851f, 0x8c1f, 0xbc1f,
	0xec1f, 0xfc1b, 0xfc16, 0xfc10, 0xfc28, 0xfe48, 0xe7e8, 0xa7e8, 0x5fe8, 0x47ed, 0x47f6,
	0x47ff, 0x45df, 0x439f, 0x521f, 0x9a1f, 0xe21f, 0xfa1a, 0xfa11, 0xfa08, 0xfae0, 0xfdc0,
	0xe7e0, 0x87e0, 0x27e0, 0x07e7, 0x07f3, 0x07ff, 0x051f, 0x023f, 0x181f, 0x781f, 0xd81f,
	0xf818, 0xf80c, 0xf800,
};

/* DMA channel to use for transmit. */
static int dma_ch_spi;

#if TFT_HW_ACCEL
/* PIO SM for LUT & horizontal scaling. */
static int sm_lut = -1;

/* DMA channel for moving 8b samples into the PIO SM. */
static int dma_ch_pio_in;

/* DMA channel for moving 16b samples out of PIO SM. */
static int dma_ch_pio_out;

/* DMA channel to drive the row-by-row output. */
static int dma_ch_rows;
#endif

/* Implemented by the driver. */
extern void tft_preflight(void);
extern void tft_begin_sync(void);

inline static void select_register(void)
{
	gpio_put(TFT_RS_PIN, 0);
}

inline static void select_data(void)
{
	gpio_put(TFT_RS_PIN, 1);
}

inline static void select_chip(bool cs)
{
	gpio_put(TFT_CS_PIN, !cs);
}

__weak void tft_dma_channel_wait_for_finish_blocking(int dma_ch_spi)
{
	dma_channel_wait_for_finish_blocking(dma_ch_spi);
}

static void __unused write_buffer_dma(void *bstr, size_t len)
{
	tft_dma_channel_wait_for_finish_blocking(dma_ch_spi);
	dma_channel_transfer_from_buffer_now(dma_ch_spi, bstr, len);
}

void tft_control(uint8_t reg, const uint8_t *bstr, int len)
{
	if (len < 0 || (len > 0 && !bstr)) return;
	// Register commands and parameters are byte streams, even though pixels
	// use 16-bit DMA. This also handles odd parameter counts without padding.
	spi_set_format(TFT_SPI_DEV, 8, 0, 0, SPI_MSB_FIRST);
	select_register();
	select_chip(true);
	spi_write_blocking(TFT_SPI_DEV, &reg, 1);
	select_data();
	if (len > 0) spi_write_blocking(TFT_SPI_DEV, bstr, (size_t)len);
	select_chip(false);
	spi_set_format(TFT_SPI_DEV, 16, 0, 0, SPI_MSB_FIRST);
}

#if defined(TFT_MISO_PIN)
void tft_read(uint8_t reg, uint8_t *bstr, int len, int delay_us)
{
	if (len < 0 || (len > 0 && !bstr)) return;
	spi_set_format(TFT_SPI_DEV, 8, 0, 0, SPI_MSB_FIRST);
	select_register();
	select_chip(true);
	spi_write_blocking(TFT_SPI_DEV, &reg, 1);
	if (delay_us >= 0) sleep_us(delay_us);
	select_data();
	if (len > 0) spi_read_blocking(TFT_SPI_DEV, 0, bstr, (size_t)len);
	select_chip(false);
	spi_set_format(TFT_SPI_DEV, 16, 0, 0, SPI_MSB_FIRST);
}
#endif

void picotft_init(void)
{
	int framebuf_len = TFT_WIDTH * TFT_HEIGHT;
	int txbuf_len = TFT_WIDTH * 2 * TFT_SCALE;

	printf("tft: allocate fb1=%i, fb2=%i, tx1=%i, tx2=%i\n", framebuf_len, framebuf_len,
	       txbuf_len, txbuf_len);

	tft_input = buffer[0];
	tft_committed = buffer[1];

	unsigned rate = spi_init(TFT_SPI_DEV, TFT_BAUDRATE);
	spi_set_format(TFT_SPI_DEV, 16, 0, 0, SPI_MSB_FIRST);

	printf("tft: spi 16b rate=%u\n", rate);

#if defined(TFT_MISO_PIN)
	printf("tft: pins cs=%i, sck=%i, mosi=%i, miso=%i, rs=%i, rst=%i\n", TFT_CS_PIN,
	       TFT_SCK_PIN, TFT_MOSI_PIN, TFT_MISO_PIN, TFT_RS_PIN, TFT_RST_PIN);
#else
	printf("tft: pins cs=%i, sck=%i, mosi=%i, rs=%i, rst=%i\n", TFT_CS_PIN, TFT_SCK_PIN,
	       TFT_MOSI_PIN, TFT_RS_PIN, TFT_RST_PIN);
#endif

	//gpio_set_function(TFT_CS_PIN, GPIO_FUNC_SPI);
	gpio_init(TFT_CS_PIN);
	gpio_set_dir(TFT_CS_PIN, GPIO_OUT);
	gpio_put(TFT_CS_PIN, 1);

	gpio_set_function(TFT_SCK_PIN, GPIO_FUNC_SPI);
	gpio_set_function(TFT_MOSI_PIN, GPIO_FUNC_SPI);

#if defined(TFT_MISO_PIN)
	gpio_set_function(TFT_MISO_PIN, GPIO_FUNC_SPI);
#endif

	gpio_init(TFT_RST_PIN);
	gpio_set_dir(TFT_RST_PIN, GPIO_OUT);

	gpio_init(TFT_RS_PIN);
	gpio_set_dir(TFT_RS_PIN, GPIO_OUT);

	dma_ch_spi = dma_claim_unused_channel(true);
	printf("tft: claimed dma%i for spi\n", dma_ch_spi);

	dma_channel_config dma_conf;

#if TFT_HW_ACCEL
	dma_ch_rows = dma_claim_unused_channel(true);
	printf("tft: claimed dma%i for rows\n", dma_ch_rows);

	dma_ch_pio_in = dma_claim_unused_channel(true);
	printf("tft: claimed dma%i for pio_in\n", dma_ch_pio_in);

	dma_ch_pio_out = dma_claim_unused_channel(true);
	printf("tft: claimed dma%i for pio_out\n", dma_ch_pio_out);

	sm_lut = pio_claim_unused_sm(TFT_PIO, true);
	printf("tft: claimed pio%i.%i\n", TFT_PIO == pio0 ? 0 : 1, sm_lut);

	struct pio_lut_config lut_cfg = {
		.origin = -1,
		.pio = TFT_PIO,
		.sm = sm_lut,
		.base_addr = (uint32_t)tft_palette,
		.scale = TFT_SCALE,
	};

	int offset = pio_lut_init(&lut_cfg);

	if (offset < 0)
		panic("tft: pio_lut_init failed");

	printf("tft: pio_lut offset=%i\n", offset);

	pio_sm_set_enabled(TFT_PIO, sm_lut, true);

	dma_conf = dma_channel_get_default_config(dma_ch_spi);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_16);
	channel_config_set_dreq(&dma_conf, spi_get_dreq(TFT_SPI_DEV, true));
	channel_config_set_chain_to(&dma_conf, dma_ch_pio_out);
	channel_config_set_irq_quiet(&dma_conf, true);
	dma_channel_configure(dma_ch_spi, &dma_conf, &spi_get_hw(TFT_SPI_DEV)->dr, NULL, 1, false);

	dma_conf = dma_channel_get_default_config(dma_ch_pio_out);
	channel_config_set_read_increment(&dma_conf, false);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_dreq(&dma_conf, pio_get_dreq(TFT_PIO, sm_lut, false));
	channel_config_set_irq_quiet(&dma_conf, true);
	dma_channel_configure(dma_ch_pio_out, &dma_conf, &dma_hw->ch[dma_ch_spi].al3_read_addr_trig,
			      &TFT_PIO->rxf[sm_lut], 1, true);

	dma_conf = dma_channel_get_default_config(dma_ch_pio_in);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_8);
	channel_config_set_dreq(&dma_conf, pio_get_dreq(TFT_PIO, sm_lut, true));
	channel_config_set_chain_to(&dma_conf, dma_ch_rows);
	channel_config_set_irq_quiet(&dma_conf, true);
	dma_channel_configure(dma_ch_pio_in, &dma_conf, &TFT_PIO->txf[sm_lut], NULL,
			      TFT_RAW_WIDTH / TFT_SCALE, false);

	dma_conf = dma_channel_get_default_config(dma_ch_rows);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_32);
	channel_config_set_irq_quiet(&dma_conf, true);
	dma_channel_configure(dma_ch_rows, &dma_conf, &dma_hw->ch[dma_ch_pio_in].al3_read_addr_trig,
			      NULL, 1, false);
#else
	dma_conf = dma_channel_get_default_config(dma_ch_spi);
	channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_16);
	channel_config_set_read_increment(&dma_conf, true);
	channel_config_set_write_increment(&dma_conf, false);
	channel_config_set_dreq(&dma_conf, spi_get_dreq(TFT_SPI_DEV, true));
	dma_channel_configure(dma_ch_spi, &dma_conf, &spi_get_hw(TFT_SPI_DEV)->dr, NULL, 0, false);
#endif

	printf("tft: begin reset & preflight...\n");

	gpio_put(TFT_RST_PIN, 0);
	sleep_ms(TFT_RST_DELAY);

	gpio_put(TFT_RST_PIN, 1);
	sleep_ms(TFT_RST_DELAY);

	tft_preflight();
}

void tft_swap_buffers(void)
{
	uint8_t *tmp = tft_committed;
	tft_committed = tft_input;
	tft_input = tmp;
}

static bool async_active;
#if !TFT_HW_ACCEL
static int async_row;
static int async_repeat;
#endif

bool tft_sync_start(void)
{
	if (async_active) return false;
#if TFT_HW_ACCEL
	// The optional chained PIO path retains its synchronous implementation.
	tft_sync();
#else
	tft_begin_sync();
	select_chip(true);
	async_row = 0;
	async_repeat = 0;
	async_active = true;
#endif
	return true;
}

bool tft_sync_poll(void)
{
#if !TFT_HW_ACCEL
	if (!async_active) return true;
	if (dma_channel_is_busy(dma_ch_spi)) return false;
	if (async_row == TFT_RAW_HEIGHT / TFT_SCALE) {
		// DMA completion can precede the final bits leaving the SPI shifter.
		if (spi_is_busy(TFT_SPI_DEV)) return false;
		select_chip(false);
		async_active = false;
		return true;
	}
	if (async_repeat == 0) {
		const uint8_t *src = tft_committed + async_row * (TFT_RAW_WIDTH / TFT_SCALE);
		uint16_t *dst = txbuf[0];
		for (int x = 0; x < TFT_RAW_WIDTH / TFT_SCALE; x++) {
			uint16_t color = tft_palette[src[x]];
			for (int n = 0; n < TFT_SCALE; n++) *dst++ = color;
		}
	}
	dma_channel_transfer_from_buffer_now(dma_ch_spi, txbuf[0], TFT_RAW_WIDTH);
	if (++async_repeat == TFT_SCALE) {
		async_repeat = 0;
		async_row++;
	}
	return false;
#else
	return true;
#endif
}

void tft_sync(void)
{
	while (!tft_sync_poll()) tight_loop_contents();
#if TFT_HW_ACCEL
	tft_begin_sync();
	select_chip(true);

	/* Use DMA script stored in flash. */
	const void **script = dma_row_script[tft_committed == buffer[1]];
	script += (XIP_NOCACHE_NOALLOC_BASE - XIP_BASE) / sizeof(*script);
	dma_channel_transfer_from_buffer_now(dma_ch_rows, script, 1);

	/* Busy-wait until the first row transfer starts. */
	while (!dma_hw->ch[dma_ch_pio_in].read_addr)
		tight_loop_contents();

	/* Yield until all rows have been enqueued. */
	while (dma_hw->ch[dma_ch_pio_in].read_addr)
		tft_dma_channel_wait_for_finish_blocking(dma_ch_pio_in);

	/* Busy wait until all the PIO outputs have been consumed. */
	while (dma_debug_hw->ch[dma_ch_pio_out].dbg_ctdreq)
		tight_loop_contents();
#else
	tft_begin_sync();
	select_chip(true);

	const uint8_t *inptr = tft_committed;

	for (int y = 0; y < TFT_RAW_HEIGHT / TFT_SCALE; y++) {
		uint16_t *buf = txbuf[y & 1];
		uint16_t *bufptr = buf;

		for (int x = 0; x < TFT_RAW_WIDTH / TFT_SCALE; x++) {
			uint16_t color = tft_palette[*inptr++];

			for (int i = 0; i < TFT_SCALE; i++)
				*bufptr++ = color;
		}

		for (int i = 0; i < TFT_SCALE; i++)
			write_buffer_dma(buf, TFT_RAW_WIDTH);
	}

	tft_dma_channel_wait_for_finish_blocking(dma_ch_spi);
#endif

	/* Busy wait until SPI finishes as well. */
	while (spi_is_busy(TFT_SPI_DEV))
		tight_loop_contents();

	select_chip(false);
}

void tft_swap_sync(void)
{
	while (!tft_sync_poll()) tight_loop_contents();
	tft_swap_buffers();
	tft_sync();
}

void tft_draw_rect(int x0, int y0, int x1, int y1, int color)
{
	if (x0 > x1) {
		int tmp = x0;
		x0 = x1;
		x1 = tmp;
	}

	if (y0 > y1) {
		int tmp = y0;
		y0 = y1;
		y1 = tmp;
	}

#if TFT_SWAP_XY
	for (int x = x0; x <= x1; x++)
		for (int y = y0; y <= y1; y++)
			tft_draw_pixel(x, y, color);
#else
	for (int y = y0; y <= y1; y++)
		for (int x = x0; x <= x1; x++)
			tft_draw_pixel(x, y, color);
#endif
}

void tft_fill(int color)
{
	memset(tft_input, color, TFT_WIDTH * TFT_HEIGHT);
}

void tft_draw_sprite(int x, int y, int w, int h, const uint8_t *data, int trsp)
{
	tft_draw_sprite_flipped(x, y, w, h, data, trsp, false, false, false);
}

void tft_draw_sprite_flipped(int x, int y, int w, int h, const uint8_t *data, int trsp, bool flip_x,
			     bool flip_y, bool swap_xy)
{
#define loop                                   \
	for (int sy = 0; sy < h; sy++)         \
		for (int sx = 0; sx < w; sx++) \
			if ((c = data[sy * w + sx]) != trsp)

	int c;
	int b = h - 1;
	int r = w - 1;

	if (flip_y && flip_x && swap_xy) {
		loop tft_draw_pixel(x + b - sy, y + r - sx, c);
	} else if (flip_y && flip_x && !swap_xy) {
		loop tft_draw_pixel(x + r - sx, y + b - sy, c);
	} else if (flip_y && !flip_x && swap_xy) {
		loop tft_draw_pixel(x + b - sy, y + sx, c);
	} else if (flip_y && !flip_x && !swap_xy) {
		loop tft_draw_pixel(x + sx, y + b - sy, c);
	} else if (!flip_y && flip_x && swap_xy) {
		loop tft_draw_pixel(x + sy, y + r - sx, c);
	} else if (!flip_y && flip_x && !swap_xy) {
		loop tft_draw_pixel(x + r - sx, y + sy, c);
	} else if (!flip_y && !flip_x && swap_xy) {
		loop tft_draw_pixel(x + sy, y + sx, c);
	} else {
		loop tft_draw_pixel(x + sx, y + sy, c);
	}

#undef loop
}

void tft_draw_glyph(int x, int y, int color, char c)
{
	/* Apply origin offset (matches tft_draw_pixel convention). */
	x -= tft_origin_x;
	y -= tft_origin_y;

	/* Fast path: fully on-screen — write directly into framebuffer. */
	if (x >= 0 && x + 8 <= TFT_WIDTH && y >= 0 && y + 16 <= TFT_HEIGHT) {
		const uint8_t *glyph = tft_font + (unsigned char)c * 16;
		uint8_t *row = &tft_input[y * TFT_WIDTH + x];
		for (int gy = 0; gy < 16; gy++) {
			uint8_t bits = glyph[gy];
			if (bits) {
				if (bits & 0x80) row[0] = color;
				if (bits & 0x40) row[1] = color;
				if (bits & 0x20) row[2] = color;
				if (bits & 0x10) row[3] = color;
				if (bits & 0x08) row[4] = color;
				if (bits & 0x04) row[5] = color;
				if (bits & 0x02) row[6] = color;
				if (bits & 0x01) row[7] = color;
			}
			row += TFT_WIDTH;
		}
		return;
	}

	/* Slow path: partially clipped — per-pixel with bounds checks. */
	const uint8_t *glyph = tft_font + (unsigned char)c * 16;
	for (int gy = 0; gy < 16; gy++) {
		uint8_t bits = glyph[gy];
		if (!bits) continue;
		int py = y + gy;
		if (py < tft_clip_y0 || py >= tft_clip_y1) continue;
		for (int gx = 0; gx < 8; gx++) {
			if ((bits >> (7 - gx)) & 1) {
				int px = x + gx;
				if (px >= tft_clip_x0 && px < tft_clip_x1)
					tft_input[py * TFT_WIDTH + px] = color;
			}
		}
	}
}

void tft_draw_string(int x, int y, int color, const char *str)
{
	for (; *str; str++, x += 8)
		tft_draw_glyph(x, y, color, *str);
}

void tft_draw_string_right(int x, int y, int color, const char *str)
{
	int len = strlen(str);
	x -= len * 8;
	for (; *str; str++, x += 8)
		tft_draw_glyph(x, y, color, *str);
}

void tft_draw_string_center(int x, int y, int color, const char *str)
{
	int len = strlen(str);
	x -= len * 4;
	for (; *str; str++, x += 8)
		tft_draw_glyph(x, y, color, *str);
}
