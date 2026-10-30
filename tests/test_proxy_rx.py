"""Check polled RX, ring handoff, overlong text, and text/binary boundaries."""
from pathlib import Path
import re
import subprocess
import tempfile
from dma_rx_mock import write_headers

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'kmbox_serial_handler.c').read_text()


def function(name):
    match = re.search(r'^(?:static )?(?:__force_inline )?(?:bool|void|uint\d+_t) '
                      + name + r'\([^;]*?\)\s*\{', source, re.M)
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
#include <stddef.h>
#include <string.h>
#include "fast_protocol.h"
#include "dma_uart.h"
#define __force_inline inline
#define UART_RX_BUFFER_SIZE 128
#define UART_RX_BUFFER_MASK 127
#define KMBOX_CMD_BUFFER_SIZE 64
#define KMBOX_UART 0
#define BRIDGE_SYNC_BYTE 0xBD
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE] __attribute__((aligned(UART_RX_BUFFER_SIZE)));
static uint32_t uart_rx_read_pos, uart_rx_poll_write_pos;
static int uart_rx_dma_chan = -1;
static bool text_discarding;
static dma_rx_state_t uart_rx_state;
static uint32_t uart_rx_reported_epoch;
static bool uart_rx_loss_pending;
static bool g_xbox_mode;
static int g_timed_move_alarm_id = -1;
static struct {bool pending, fired;} pending_timed_move;
static unsigned released, cleared, cancelled, xbox_cleared;
static void kmbox_set_injected_buttons(uint8_t mask) {assert(mask == 0); released++;}
static void smooth_clear_queue(void) {cleared++;}
static void cancel_alarm(int id) {assert(id == 7); cancelled++;}
static void xbox_inject_clear(void) {xbox_cleared++;}
static uint8_t fifo[512];
static size_t fifo_pos, fifo_len;
static unsigned commands;
static char last_command[64];
static bool uart_is_readable(int uart) {assert(uart == 0); return fifo_pos < fifo_len;}
static int uart_getc(int uart) {assert(uart == 0 && fifo_pos < fifo_len); return fifo[fifo_pos++];}
static void mark_activity(uint32_t now_ms) {assert(now_ms == 100);}
static bool handle_text_command(const char *line, size_t len, uint32_t now_ms) {
    (void)line; (void)len; assert(now_ms == 100); return false;
}
static void kmbox_process_serial_line(const char *line, size_t len, const char *term, unsigned n, uint32_t now_ms) {
    assert(n == 1 && *term == '\n' && now_ms == 100 && len < sizeof(last_command));
    memcpy(last_command, line, len + 1); commands++;
}
'''
bitmap = re.search(r'static const uint32_t g_fast_cmd_bitmap\[8\] = \{.*?\n\};', source, re.S)[0]
test = r'''
static void reset(const void *input, size_t len) {
    memcpy(fifo, input, len); fifo_pos = 0; fifo_len = len;
    uart_rx_read_pos = uart_rx_poll_write_pos = 0;
    uart_rx_dma_chan = -1; text_discarding = false; commands = 0;
    uart_rx_reported_epoch = 0; uart_rx_loss_pending = false; mock_us = 1000;
}
int main(void) {
    const uint8_t mixed[] = {'M','1',',','2','\n', 1,10,13,0,1,2,3,4, 'P','\n'};
    reset(mixed, sizeof(mixed));
    assert(process_text_commands(100, 64) == 1 && commands == 1);
    assert(strcmp(last_command, "M1,2") == 0 && rx_available() == 10);
    for (unsigned i = 0; i < 8; i++) assert(rx_peek_at(i) == mixed[5 + i]);
    rx_consume(8);
    assert(process_text_commands(100, 64) == 1 && strcmp(last_command, "P") == 0);

    uint8_t oversized[203]; memset(oversized, 'X', sizeof(oversized));
    oversized[127] = 1; oversized[200] = '\n'; oversized[201] = 'P'; oversized[202] = '\n';
    reset(oversized, sizeof(oversized));
    process_text_commands(100, 64); assert(text_discarding && commands == 0);
    process_text_commands(100, 64);
    assert(!text_discarding && commands == 1 && strcmp(last_command, "P") == 0);

    uint8_t maximum[64]; memset(maximum, 'A', 63); maximum[63] = '\n';
    reset(maximum, sizeof(maximum)); process_text_commands(100, 64);
    assert(commands == 1 && strlen(last_command) == 63);
    reset("P\nP\n", 4);
    assert(process_text_commands(100, 1) == 1 && rx_available() == 2);

    // A full polled ring stays nonempty and leaves excess bytes in the FIFO.
    uint8_t burst[150]; for (unsigned i = 0; i < 150; i++) burst[i] = (uint8_t)i;
    reset(burst, sizeof(burst));
    assert(rx_available() == 127 && fifo_pos == 127);
    assert(rx_available() == 127 && fifo_pos == 127);
    rx_consume(100); assert(rx_available() == 50 && fifo_pos == 150);
    for (unsigned i = 0; i < 50; i++) assert(rx_peek_at(i) == (uint8_t)(100 + i));
    // DMA begins at the polled producer position, retaining unread ring data.
    uint32_t pos = uart_rx_poll_write_pos;
    uart_rx_buffer[pos] = 0xAA; uart_rx_buffer[(pos + 1) & 127] = 0xBB;
    dma_rx_init(&uart_rx_state, 127, uart_rx_read_pos, pos, 3000000, mock_us);
    mock_dma.write_addr = (uintptr_t)(uart_rx_buffer + ((pos + 2) & 127));
    mock_dma.transfer_count = (1u << 28) | (DMA_RX_TRANSFER_COUNT - 2);
    uart_rx_dma_chan = 1;
    assert(rx_available() == 52 && rx_peek_at(50) == 0xAA && rx_peek_at(51) == 0xBB);
    uint8_t copy[52];
    assert(rx_copy_peek(copy, sizeof(copy)) && copy[50] == 0xAA && copy[51] == 0xBB);
    text_discarding = true;
    // Another complete lap has the same address but a different transfer count.
    mock_dma.transfer_count -= 128;
    assert(rx_available() == 0 && uart_rx_state.overruns == 1);
    assert(uart_rx_loss_pending && !text_discarding && uart_rx_state.discarded_bytes == 180);
    assert(!rx_copy_peek(copy, 1));
    g_xbox_mode = true; g_timed_move_alarm_id = 7;
    pending_timed_move.pending = pending_timed_move.fired = true;
    rx_handle_loss();
    assert(!uart_rx_loss_pending && released == 1 && cleared == 1 && cancelled == 1 && xbox_cleared == 1);
    assert(g_timed_move_alarm_id == -1 && !pending_timed_move.pending && !pending_timed_move.fired);
    rx_handle_loss(); assert(released == 1 && cleared == 1);
    uint32_t overruns, ambiguous; uint64_t dropped;
    kmbox_get_rx_loss_stats(&overruns, &dropped, &ambiguous);
    assert(overruns == 1 && dropped == 180 && ambiguous == 0);
    mock_us += DMA_RX_IDLE_US;
    assert(rx_available() == 0 && !uart_rx_state.resync);
}
'''
with tempfile.TemporaryDirectory() as temp:
    write_headers(temp)
    unit = Path(temp) / 'test.c'
    binary = Path(temp) / 'test'
    unit.write_text(prefix + bitmap + '\n'.join(function(f) for f in [
        'rx_get_write_pos', 'rx_sync_state', 'rx_available', 'rx_peek_at', 'rx_copy_peek', 'rx_consume', 'is_fast_cmd_start',
        'parse_text_line', 'process_text_commands', 'rx_handle_loss', 'kmbox_get_rx_loss_stats']) + test)
    subprocess.run(['cc', '-std=c11', '-fsanitize=address,undefined', f'-I{temp}',
                    f'-I{ROOT / "lib/dma-uart/include"}',
                    f'-I{ROOT / "lib/fast-protocol/include"}', f'-I{ROOT / "lib/hid-defs/include"}',
                    str(unit), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    print('Proxy polled RX, ring handoff, oversized lines, and text/binary boundaries: passed')
