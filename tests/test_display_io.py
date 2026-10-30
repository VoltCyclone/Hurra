"""Check exact TFT register byte counts and LED-off/rollover behavior."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def function(file, name):
    source = (ROOT / file).read_text()
    match = re.search(r'^(?:static )?(?:bool|void) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define TFT_SPI_DEV 0
#define SPI_MSB_FIRST 0
static unsigned bits = 16;
static bool selected, data_mode;
static uint8_t written[128];
static size_t written_len;
static void select_register(void) {data_mode = false;}
static void select_data(void) {data_mode = true;}
static void select_chip(bool on) {selected = on;}
static void spi_set_format(int spi, unsigned n, int p, int h, int order) {
    assert(spi == 0 && p == 0 && h == 0 && order == 0 && !selected); bits = n;
}
static void spi_write_blocking(int spi, const uint8_t *src, size_t n) {
    assert(spi == 0 && bits == 8 && selected);
    memcpy(written + written_len, src, n); written_len += n;
}
static void spi_read_blocking(int spi, uint8_t fill, uint8_t *dst, size_t n) {
    assert(spi == 0 && fill == 0 && bits == 8 && selected && data_mode);
    for (size_t i = 0; i < n; i++) dst[i] = (uint8_t)(i + 1);
}
static void sleep_us(int delay) {(void)delay;}
struct repeating_timer {int unused;};
static uint32_t now_us, s_led_activity_expire_us;
static volatile uint32_t s_led_shadow_grb;
static bool s_led_activity_pending, dma_busy;
static int s_led_dma_chan = 1;
static unsigned pushes;
static uint32_t time_us_32(void) {return now_us;}
static bool dma_channel_is_busy(int ch) {assert(ch == 1); return dma_busy;}
static void dma_channel_set_trans_count(int ch, unsigned n, bool trigger) {
    assert(ch == 1 && n == 1 && !trigger);
}
static void dma_channel_set_read_addr(int ch, const volatile void *src, bool trigger) {
    assert(ch == 1 && src == &s_led_shadow_grb && trigger); pushes++;
}
'''
test = r'''
int main(void) {
    const unsigned sizes[] = {0, 1, 2, 3, 15};
    for (unsigned k = 0; k < sizeof(sizes)/sizeof(sizes[0]); k++) {
        unsigned n = sizes[k];
        uint8_t *src = n ? malloc(n) : NULL;
        for (unsigned i = 0; i < n; i++) src[i] = (uint8_t)(i + 1);
        written_len = 0;
        tft_control(0x26, src, (int)n);
        assert(bits == 16 && !selected && written_len == n + 1 && written[0] == 0x26);
        if (n) assert(memcmp(written + 1, src, n) == 0);
        written_len = 0;
        tft_read(0x45, src, (int)n, 3);
        assert(bits == 16 && !selected && written_len == 1);
        for (unsigned i = 0; i < n; i++) assert(src[i] == i + 1);
        free(src);
    }
    // Zero must be transmitted to turn a previously lit LED off.
    s_led_shadow_grb = 0;
    assert(led_refresh_timer_callback(NULL) && pushes == 1);
    dma_busy = true; led_refresh_timer_callback(NULL); assert(pushes == 1);
    s_led_activity_pending = true;
    now_us = UINT32_MAX - 10; s_led_activity_expire_us = 20;
    led_refresh_timer_callback(NULL); assert(s_led_activity_pending);
    now_us = 21;
    led_refresh_timer_callback(NULL); assert(!s_led_activity_pending);
}
'''
with tempfile.TemporaryDirectory() as temp:
    source = Path(temp) / 'test.c'
    binary = Path(temp) / 'test'
    source.write_text('#include <stdlib.h>\n' + prefix + function('bridge/tft.c', 'tft_control')
                      + function('bridge/tft.c', 'tft_read')
                      + function('led_control.c', 'led_refresh_timer_callback') + test)
    subprocess.run(['cc', '-std=c11', '-fsanitize=address,undefined', str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    print('TFT byte transactions and LED rollover/off: passed')

    async_prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define TFT_HW_ACCEL 0
#define TFT_RAW_WIDTH 4
#define TFT_RAW_HEIGHT 4
#define TFT_SPI_DEV 0
static int dma_ch_spi = 1;
static bool async_active, busy, shifter_busy, selected;
static int async_row, async_repeat;
static uint8_t frame[16];
static uint8_t *tft_committed = frame;
static uint16_t tft_palette[16], txbuf[2][4], sent[16];
static const uint16_t *pending;
static size_t sent_count;
static void tft_sync(void) {assert(0);}
static void tft_begin_sync(void) {assert(!selected && !busy);}
static void select_chip(bool on) {selected = on;}
static bool dma_channel_is_busy(int ch) {assert(ch == 1); return busy;}
static bool spi_is_busy(int spi) {assert(spi == 0); return shifter_busy;}
static void dma_channel_transfer_from_buffer_now(int ch, const void *src, size_t n) {
    assert(ch == 1 && n == 4 && !busy && selected);
    pending = src; busy = true;
}
'''
    async_test = r'''
int main(void) {
    for (unsigned i = 0; i < 16; i++) {frame[i] = (uint8_t)i; tft_palette[i] = (uint16_t)(100 + i);}
    assert(tft_sync_poll());
    assert(tft_sync_start() && !tft_sync_start());
    for (unsigned row = 0; row < 4; row++) {
        assert(!tft_sync_poll() && busy);
        uint16_t snapshot[4]; memcpy(snapshot, pending, sizeof(snapshot));
        assert(!tft_sync_poll()); // Busy DMA must keep the staging row unchanged.
        assert(memcmp(snapshot, pending, sizeof(snapshot)) == 0);
        memcpy(sent + sent_count, pending, sizeof(snapshot)); sent_count += 4;
        busy = false;
    }
    shifter_busy = true;
    assert(!tft_sync_poll() && selected); // Keep CS until the last bits leave SPI.
    shifter_busy = false;
    assert(tft_sync_poll() && !selected && !async_active);
    assert(sent_count == 16);
    for (unsigned y = 0; y < 4; y++)
        for (unsigned x = 0; x < 4; x++)
            assert(sent[y * 4 + x] == tft_palette[frame[(y / TFT_SCALE) * (4 / TFT_SCALE) + x / TFT_SCALE]]);
}
'''
    source.write_text(async_prefix + function('bridge/tft.c', 'tft_sync_start')
                      + function('bridge/tft.c', 'tft_sync_poll') + async_test)
    for scale in (1, 2):
        subprocess.run(['cc', '-std=c11', '-fsanitize=address,undefined', f'-DTFT_SCALE={scale}',
                        str(source), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    print('Cooperative TFT DMA, framebuffer scaling, and SPI completion: passed')

    flush_prefix = r'''
#include <assert.h>
#include <stdbool.h>
typedef struct {int unused;} repeating_timer_t;
static bool initialized = true, frame_ready, render_due, transfer_pending;
static bool dma_active, dma_done;
static unsigned renders, swaps;
static void tft_render_frame(void) {assert(!dma_active); renders++; frame_ready = true;}
static void tft_swap_buffers(void) {assert(!dma_active); swaps++;}
static bool tft_sync_start(void) {assert(!dma_active); dma_active = true; dma_done = false; return true;}
static bool tft_sync_poll(void) {
    if (!dma_done) return false;
    dma_active = false; return true;
}
'''
    flush_test = r'''
int main(void) {
    assert(tft_render_timer_callback(0) && render_due && renders == 0);
    assert(!tft_display_flush() && renders == 1 && swaps == 1 && transfer_pending);
    assert(tft_render_timer_callback(0));
    assert(!tft_display_flush() && renders == 1 && swaps == 1 && frame_ready);
    dma_done = true;
    assert(tft_display_flush() && !transfer_pending && !frame_ready);
    assert(!tft_display_flush() && renders == 2 && swaps == 2);
}
'''
    source.write_text(flush_prefix + function('bridge/tft_display.c', 'tft_render_timer_callback')
                      + function('bridge/tft_display.c', 'tft_display_flush') + flush_test)
    subprocess.run(['cc', '-std=c11', '-fsanitize=address,undefined', str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    print('Display IRQ scheduling and framebuffer ownership: passed')
