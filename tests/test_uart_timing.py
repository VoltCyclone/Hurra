"""Run UART TX logic with deferred DMA and check production clock expressions."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
uart = (ROOT / 'bridge/hw_uart.c').read_text()
serial = (ROOT / 'kmbox_serial_handler.c').read_text()


def function(source, name):
    match = re.search(r'^(?:bool|void) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


task = serial[serial.index('void kmbox_serial_task(void)'):]
task_clock = re.search(r'uint32_t now_ms = (.*);', task)[1]
click_clock = re.search(r'uint32_t click_now = (.*);', serial)[1]
ppm = re.search(r'int32_t baud_error_ppm = (.*);', uart)[1]
prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define HW_UART 0
#define HW_UART_TX_BUFFER_SIZE 512
static int tx_dma_chan = 1;
static uint8_t tx_buffer[512], output[2048];
static size_t total_tx_bytes, output_len, dma_len;
static const uint8_t *dma_source;
static bool busy;
static bool dma_channel_is_busy(int ch) {assert(ch == 1); return busy;}
static void dma_channel_set_read_addr(int ch, const void *p, bool trigger) {
    assert(ch == 1 && !trigger && !busy); dma_source = p;
}
static void dma_channel_set_trans_count(int ch, size_t n, bool trigger) {
    assert(ch == 1 && trigger && !busy); dma_len = n; busy = true;
}
static void tight_loop_contents(void) {
    assert(busy);
    memcpy(output + output_len, dma_source, dma_len);
    output_len += dma_len; busy = false;
}
static void uart_putc_raw(int uart, uint8_t c) {
    assert(uart == 0 && !busy); output[output_len++] = c;
}
static void uart_write_blocking(int uart, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) uart_putc_raw(uart, data[i]);
}
static uint64_t clock_us;
static uint64_t get_absolute_time(void) {return clock_us;}
static uint32_t to_ms_since_boot(uint64_t t) {return (uint32_t)(t / 1000);}
static uint32_t time_us_32(void) {return (uint32_t)clock_us;}
bool hw_uart_tx_ready(void);
void hw_uart_tx_wait(void);
'''
clocks = f'''
static uint32_t task_ms(void) {{ uint32_t now_us = time_us_32(); (void)now_us; return {task_clock}; }}
static uint32_t click_ms(void) {{ return {click_clock}; }}
static int32_t error_ppm(uint32_t actual_baud, uint32_t baud) {{ return {ppm}; }}
'''
test = r'''
int main(void) {
    uint8_t first[] = {1, 2, 3}, second[] = {4, 5};
    assert(hw_uart_tx_ready());
    assert(hw_uart_send(first, sizeof(first)));
    assert(!hw_uart_tx_ready());
    assert(!hw_uart_send(second, sizeof(second)));
    assert(memcmp(dma_source, first, sizeof(first)) == 0);
    hw_uart_puts("P"); hw_uart_putc('\n');
    assert(output_len == 5 && memcmp(output, "\1\2\3P\n", 5) == 0);
    assert(hw_uart_send(second, sizeof(second)));
    hw_uart_tx_wait(); assert(hw_uart_tx_ready());
    assert(memcmp(output + 5, second, sizeof(second)) == 0);
    assert(total_tx_bytes == 7);
    assert(!hw_uart_send(NULL, 1));
    tx_dma_chan = -1;
    assert(hw_uart_send(first, sizeof(first)));
    assert(memcmp(output + 7, first, sizeof(first)) == 0);

    // Cross the 32-bit microsecond rollover without premature click expiry.
    clock_us = (uint64_t)UINT32_MAX - 20000;
    uint32_t start = click_ms();
    clock_us += 40000;
    assert(task_ms() - start == 40 && click_ms() == task_ms());
    clock_us += 100000;
    assert(task_ms() - start == 140);
    // The millisecond clock itself wraps normally after ~49 days.
    clock_us = ((uint64_t)UINT32_MAX - 20) * 1000;
    start = click_ms(); clock_us += 40000;
    assert(task_ms() - start == 40);
    assert(error_ppm(2900000, 3000000) == -33333);
    assert(error_ppm(3100000, 3000000) == 33333);
}
'''
with tempfile.TemporaryDirectory() as temp:
    source = Path(temp) / 'test.c'
    binary = Path(temp) / 'test'
    source.write_text(prefix + clocks + '\n'.join(function(uart, f) for f in
        ['hw_uart_send', 'hw_uart_putc', 'hw_uart_puts', 'hw_uart_tx_ready', 'hw_uart_tx_wait']) + test)
    subprocess.run(['cc', '-std=c11', '-fsanitize=address,undefined', str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    print('UART DMA ordering, baud arithmetic, and timer rollover: passed')

    (Path(temp) / 'hardware').mkdir()
    (Path(temp) / 'hardware/sync.h').write_text('''
#include <stdint.h>
typedef int spin_lock_t;
static spin_lock_t lock;
static int spin_lock_claim_unused(bool required) {(void)required; return 24;}
static spin_lock_t *spin_lock_init(unsigned n) {(void)n; return &lock;}
static uint32_t spin_lock_blocking(spin_lock_t *p) {(void)p; return 0;}
static void spin_unlock(spin_lock_t *p, uint32_t v) {(void)p; (void)v;}
''')
    source.write_text(r'''
#include <assert.h>
#include "kmbox_commands.c"
int main(void) {
    kmbox_commands_init();
    int parsed_x, parsed_y, parsed_button;
    assert(kmbox_parse_move_command("km.move( -32768 , +32767 ) ", &parsed_x, &parsed_y));
    assert(parsed_x == -32768 && parsed_y == 32767);
    assert(!kmbox_parse_move_command("km.move(999999999999999999999,0)", &parsed_x, &parsed_y));
    assert(!kmbox_parse_move_command("km.move(1,2", &parsed_x, &parsed_y));
    assert(!kmbox_parse_move_command("km.move(1,2)junk", &parsed_x, &parsed_y));
    for (unsigned i = 0; i < KMBOX_BUTTON_COUNT; i++) {
        char command[24]; snprintf(command, sizeof(command), "km.click(%u)", i);
        assert(kmbox_parse_click_command(command, &parsed_button) && parsed_button == (int)i);
    }
    assert(!kmbox_parse_click_command("km.click(-1)", &parsed_button));
    assert(!kmbox_parse_click_command("km.click(5)", &parsed_button));
    assert(!kmbox_parse_click_command("km.click(999999999999999999)", &parsed_button));
    assert(!kmbox_parse_click_command("km.click(0", &parsed_button));
    kmbox_update_physical_buttons(1);
    kmbox_update_states(100);
    kmbox_set_injected_buttons(2);
    assert(kmbox_get_current_buttons() == 3 && kmbox_has_forced_buttons());
    uint8_t buttons;
    int8_t x8, y8, wheel, pan;
    int16_t x16, y16;
    kmbox_get_mouse_report(&buttons, &x8, &y8, &wheel, &pan);
    assert(buttons == 3);
    kmbox_get_mouse_report_16(&buttons, &x16, &y16, &wheel, &pan);
    assert(buttons == 3);
    kmbox_set_injected_buttons(0);
    assert(kmbox_get_current_buttons() == 1 && !kmbox_has_forced_buttons());
    assert(kmbox_try_drain_mouse_16(3, &buttons, &x16, &y16, &wheel, &pan));
    assert(buttons == 1); // Synthetic release must preserve the physical left hold.
    kmbox_update_physical_buttons(0);
    kmbox_update_states(101);
    assert(kmbox_get_current_buttons() == 0);
    kmbox_commands_init();
    for (unsigned i = 0; i < KMBOX_BUTTON_COUNT; i++) {
        uint32_t start = UINT32_MAX - 30;
        kmbox_start_button_click((kmbox_button_t)i, start);
        button_state_t *b = &g_kmbox_state.buttons[i];
        uint32_t release = b->click_release_start, end = b->click_end_time;
        kmbox_update_states(start + 10);
        assert(b->is_clicking && b->is_pressed);
        kmbox_update_states(release);
        assert(b->is_clicking && !b->is_pressed);
        kmbox_update_states(end);
        assert(!b->is_clicking && !b->is_forced);

        g_kmbox_state.physical_buttons = (uint8_t)(1u << i);
        set_button_state((kmbox_button_t)i, true, start);
        set_button_state((kmbox_button_t)i, false, start);
        end = b->release_time;
        kmbox_update_states(start + 10);
        assert(b->is_forced && !b->is_pressed);
        kmbox_update_states(end);
        assert(!b->is_forced && b->is_pressed);
        g_kmbox_state.physical_buttons = 0;
    }
}
''')
    subprocess.run(['cc', '-std=c11', '-fsanitize=address,undefined', f'-I{temp}',
                    f'-I{ROOT / "lib/kmbox-commands"}', f'-I{ROOT / "lib/hid-defs/include"}',
                    str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    print('All five click/release state machines across millisecond rollover: passed')
