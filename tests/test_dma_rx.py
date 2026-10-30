"""Model RP2350 self-trigger DMA, including laps and overwrites during copying."""
from pathlib import Path
import re
import subprocess
import tempfile
from dma_rx_mock import write_headers

ROOT = Path(__file__).resolve().parents[1]


def function(name):
    source = (ROOT / 'bridge/hw_uart.c').read_text()
    match = re.search(r'^(?:bool|void|size_t|int) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
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
static void *copy_with_dma(void *dest, const void *src, size_t n);
#define memcpy copy_with_dma
#include "dma_uart.h"
#undef memcpy
static uint8_t rx_buffer[64] __attribute__((aligned(64)));
static dma_rx_state_t rx_state;
static int rx_dma_chan = 1;
static uint32_t rx_reported_loss_epoch, total_rx_bytes;
static uint64_t model_bytes;
static unsigned model_start, overwrite_on_copy;
static void registers(void) {
    mock_dma.transfer_count = (1u << 28) | (DMA_RX_TRANSFER_COUNT - (model_bytes & DMA_RX_TRANSFER_MASK));
    mock_dma.write_addr = (uintptr_t)(rx_buffer + ((model_start + model_bytes) & 63));
}
static void write_bytes(unsigned n, uint8_t first) {
    for (unsigned i = 0; i < n; i++) rx_buffer[(model_start + model_bytes++) & 63] = (uint8_t)(first + i);
    registers();
}
static void *copy_with_dma(void *dest, const void *src, size_t n) {
    memcpy(dest, src, n);
    if (overwrite_on_copy) {
        unsigned count = overwrite_on_copy; overwrite_on_copy = 0;
        write_bytes(count, 0x80);
    }
    return dest;
}
static void reset(unsigned read_pos, unsigned write_pos) {
    mock_us = 1000; mock_time_step = 0; mock_time_hook = NULL; model_bytes = 0; model_start = write_pos;
    overwrite_on_copy = 0; rx_reported_loss_epoch = total_rx_bytes = 0;
    memset(rx_buffer, 0, sizeof(rx_buffer));
    registers();
    dma_rx_init(&rx_state, 63, read_pos, write_pos, 3000000, mock_us);
}
static unsigned time_calls;
static void advance_during_snapshot(void) {
    if (++time_calls == 2) {
        mock_time_hook = NULL;
        write_bytes(1, 77);
    }
}
size_t hw_uart_rx_count(void);
size_t hw_uart_read(uint8_t *buf, size_t maxlen);
'''
test = r'''
int main(void) {
    uint8_t out[64];
    reset(0, 0);
    assert(hw_uart_rx_count() == 0);
    write_bytes(63, 1); assert(hw_uart_rx_count() == 63 && rx_state.overruns == 0);
    assert(hw_uart_read(out, 63) == 63);
    for (unsigned i = 0; i < 63; i++) assert(out[i] == i + 1);
    assert(hw_uart_getc() == -1);

    reset(0, 0); write_bytes(64, 1);
    assert(hw_uart_rx_count() == 0 && rx_state.overruns == 1 && rx_state.resync);
    assert(rx_state.discarded_bytes == 64 && hw_uart_rx_take_loss());
    assert(!hw_uart_rx_take_loss());
    assert(hw_uart_rx_count() == 0 && rx_state.overruns == 1);
    write_bytes(10, 1); assert(hw_uart_rx_count() == 0 && rx_state.resync);
    assert(rx_state.discarded_bytes == 74);
    mock_us += DMA_RX_IDLE_US;
    assert(hw_uart_rx_count() == 0 && !rx_state.resync);
    write_bytes(8, 10); assert(hw_uart_read(out, 8) == 8);
    for (unsigned i = 0; i < 8; i++) assert(out[i] == 10 + i);

    reset(0, 0); write_bytes(64 * 3, 1); // Same write pointer, three complete laps.
    assert(hw_uart_read(out, 8) == 0 && rx_state.overruns == 1);
    assert(rx_state.discarded_bytes == 192);

    reset(60, 60); write_bytes(8, 40);
    assert(hw_uart_read(out, 8) == 8 && !hw_uart_rx_take_loss());
    for (unsigned i = 0; i < 8; i++) assert(out[i] == 40 + i);

    // Preserve unread polled bytes when DMA starts at a nonzero ring position.
    reset(50, 10); write_bytes(2, 70);
    assert(hw_uart_rx_count() == 26 && rx_state.overruns == 0);
    assert(hw_uart_read(out, 26) == 26 && out[24] == 70 && out[25] == 71);

    // Hardware counter reload and software byte-total rollover happen together.
    reset(0, 0); model_bytes = DMA_RX_TRANSFER_COUNT - 3;
    rx_state.remaining = 3; rx_state.produced = rx_state.consumed = UINT32_MAX - 2;
    registers(); write_bytes(6, 20);
    assert(hw_uart_rx_count() == 6 && rx_state.produced == 3 && rx_state.overruns == 0);
    assert(hw_uart_read(out, 6) == 6);
    for (unsigned i = 0; i < 6; i++) assert(out[i] == 20 + i);

    reset(0, 0); write_bytes(8, 1); overwrite_on_copy = 64;
    assert(hw_uart_read(out, 8) == 0 && rx_state.resync && hw_uart_rx_take_loss());
    assert(rx_state.discarded_bytes == 72); // Never deliver a copy crossed by a lap.

    reset(0, 0); write_bytes(8, 1); overwrite_on_copy = 3;
    assert(hw_uart_read(out, 8) == 8 && rx_state.overruns == 0);
    for (unsigned i = 0; i < 8; i++) assert(out[i] == i + 1);
    assert(hw_uart_read(out, 3) == 3 && out[0] == 0x80 && out[2] == 0x82);

    reset(0, 0); time_calls = 0; mock_time_hook = advance_during_snapshot;
    assert(hw_uart_getc() == 77 && rx_state.overruns == 0); // Retry a changing register snapshot.

    reset(0, 0);
    mock_dma.transfer_count--; // Count/address disagree while a bus write is pending.
    assert(hw_uart_rx_count() == 0 && !rx_state.sample_valid);
    assert(rx_state.produced == 0 && rx_state.overruns == 0);
    rx_buffer[0] = 0x55; model_bytes = 1; registers();
    assert(hw_uart_getc() == 0x55);

    // Too much elapsed time makes the modulo count ambiguous: fail closed.
    reset(0, 0); model_bytes = DMA_RX_TRANSFER_COUNT + 5; registers();
    mock_us += rx_state.max_poll_gap_us;
    assert(hw_uart_rx_count() == 0 && rx_state.ambiguous_polls == 1 && rx_state.resync);
    uint64_t discarded; uint32_t ambiguous, overflows;
    hw_uart_get_rx_loss_stats(&discarded, &ambiguous);
    hw_uart_get_stats(NULL, NULL, &overflows);
    assert(discarded == 5 && ambiguous == 1 && overflows == 1);
    mock_us += DMA_RX_IDLE_US;
    assert(hw_uart_rx_count() == 0 && !rx_state.resync);
    write_bytes(8, 60); assert(hw_uart_read(out, 8) == 8);
    for (unsigned i = 0; i < 8; i++) assert(out[i] == 60 + i);

    // A scheduling pause inside the sampling operation must also fail closed.
    reset(0, 0); mock_time_step = rx_state.max_poll_gap_us + 1;
    assert(hw_uart_rx_count() == 0 && rx_state.ambiguous_polls == 1);

    // An idle DMA counter is insufficient if bytes remain in the UART FIFO.
    reset(0, 0); write_bytes(64, 0); hw_uart_rx_count();
    volatile uint32_t fifo_status = 0;
    rx_state.fifo_status = &fifo_status;
    mock_us += DMA_RX_IDLE_US * 2;
    assert(hw_uart_rx_count() == 0 && rx_state.resync);
    fifo_status = UART_UARTFR_RXFE_BITS;
    assert(hw_uart_rx_count() == 0 && rx_state.resync);
    mock_us += DMA_RX_IDLE_US;
    assert(hw_uart_rx_count() == 0 && !rx_state.resync);
}
'''
with tempfile.TemporaryDirectory() as temp:
    temp = Path(temp)
    write_headers(temp)
    unit = temp / 'test.c'
    binary = temp / 'test'
    # get_stats also exposes the existing TX statistic.
    unit.write_text(prefix + '\nstatic uint32_t total_tx_bytes;\n' + '\n'.join(function(f) for f in [
        'hw_uart_rx_available', 'hw_uart_rx_count', 'hw_uart_getc', 'hw_uart_read',
        'hw_uart_rx_take_loss', 'hw_uart_get_rx_loss_stats', 'hw_uart_get_stats']) + test)
    subprocess.run(['cc', '-std=c11', '-fsanitize=address,undefined', f'-I{temp}',
                    f'-I{ROOT / "lib/dma-uart/include"}', str(unit), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    print('DMA RX laps, count rollover, concurrent overwrite, recovery, and bridge statistics: passed')
