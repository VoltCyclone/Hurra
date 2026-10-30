"""Check fixed-size bridge commands and Feather replies split across DMA reads."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def function(file, name):
    source = (ROOT / file).read_text()
    match = re.search(r'^static void ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


serial = (ROOT / 'kmbox_serial_handler.c').read_text()
bitmap = re.search(r'static const uint32_t g_fast_cmd_bitmap\[8\] = \{.*?\n\};', serial, re.S)[0]
click_case = serial[serial.index('        case FAST_CMD_MOUSE_CLICK: {'):
                    serial.index('        case FAST_CMD_KEY_COMBO: {')]
prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "fast_protocol.h"
#include "kmbox_commands.h"
static uint8_t incoming[128], forwarded[256];
static size_t incoming_len, forwarded_len;
static uint32_t now = 1000, kmbox_last_rx_time;
static bool loss_pending;
static bool hw_uart_rx_take_loss(void) {bool lost = loss_pending; loss_pending = false; return lost;}
enum {BRIDGE_DISCONNECTED, BRIDGE_CONNECTED};
static int bridge_conn_state;
static uint64_t get_absolute_time(void) {return now;}
static uint32_t to_ms_since_boot(uint64_t t) {return (uint32_t)t;}
static size_t hw_uart_read(uint8_t *dst, size_t capacity) {
    assert(capacity >= incoming_len);
    memcpy(dst, incoming, incoming_len);
    size_t n = incoming_len; incoming_len = 0; return n;
}
static bool tud_cdc_connected(void) {return true;}
static void tud_cdc_write(const void *p, size_t n) {
    memcpy(forwarded + forwarded_len, p, n); forwarded_len += n;
}
static void tud_cdc_write_str(const char *p) {tud_cdc_write(p, strlen(p));}
static void tud_cdc_write_flush(void) {}
static unsigned fast_cmd_count;
static kmbox_button_t clicked;
static void kmbox_start_button_click(kmbox_button_t button, uint32_t ms) {
    assert(ms == now); clicked = button;
}
'''
prefix = prefix.replace('static void kmbox_start_button_click', 'void kmbox_start_button_click')
click_handler = 'static bool process_click(const uint8_t *pkt) {switch(pkt[0]) {\n' + click_case + 'default: return false;}}\n'
test = r'''
static void feed(const void *p, size_t n) {
    memcpy(incoming, p, n); incoming_len = n; uart_rx_task();
}
int main(void) {
    uint8_t packet[8];
    assert(fast_build_ping(packet) == 8 && packet[0] == FAST_CMD_PING);
    assert(fast_build_set_human(packet, 2) == 8 && packet[1] == 2);
    assert(fast_build_smooth_config(packet, 16, 1) == 8 && packet[0] == 8 && packet[2] == 1);
    const unsigned commands[] = {FAST_CMD_PING, FAST_CMD_SET_HUMAN, FAST_CMD_INFO_REQUEST,
                                 FAST_CMD_SMOOTH_CONFIG, FAST_CMD_MOUSE_MOVE};
    for (unsigned i = 0; i < sizeof(commands)/sizeof(commands[0]); i++)
        assert((g_fast_cmd_bitmap[commands[i] >> 5] >> (commands[i] & 31)) & 1);
    assert(!((g_fast_cmd_bitmap['\r' >> 5] >> ('\r' & 31)) & 1));
    for (unsigned i = 0; i < KMBOX_BUTTON_COUNT; i++) {
        fast_build_mouse_click(packet, (uint8_t)(1u << i), 1);
        assert(process_click(packet) && clicked == (kmbox_button_t)i);
    }
    fast_build_mouse_click(packet, 0xFF, 1);
    assert(!process_click(packet));
    const uint8_t reply[] = {FAST_CMD_RESPONSE, 0, 0x0A, 0x0D, 'k', 'm', '.', 0};
    for (size_t split = 1; split < sizeof(reply); split++) {
        bridge_conn_state = BRIDGE_DISCONNECTED;
        feed(reply, split);
        assert(bridge_conn_state == BRIDGE_DISCONNECTED);
        feed(reply + split, sizeof(reply) - split);
        assert(bridge_conn_state == BRIDGE_CONNECTED && kmbox_last_rx_time == now);
    }
    assert(forwarded_len == 0);
    feed("KMBOX_", 6); assert(forwarded_len == 0);
    feed("PONG\r\n", 6);
    assert(forwarded_len == 12 && memcmp(forwarded, "KMBOX_PONG\r\n", 12) == 0);
    // An abandoned partial response must not eat the next complete reply.
    feed(reply, 3); now += 101;
    bridge_conn_state = BRIDGE_DISCONNECTED;
    feed(reply, sizeof(reply)); assert(bridge_conn_state == BRIDGE_CONNECTED);
    feed(reply, 3); loss_pending = true;
    feed("", 0); // Even an empty read must reset partial state after DMA loss.
    bridge_conn_state = BRIDGE_DISCONNECTED;
    feed(reply, sizeof(reply)); assert(bridge_conn_state == BRIDGE_CONNECTED);
}
'''
with tempfile.TemporaryDirectory() as temp:
    source = Path(temp) / 'test.c'
    binary = Path(temp) / 'test'
    source.write_text(prefix + bitmap + click_handler + function('bridge/main_feather.c', 'uart_rx_task') + test)
    subprocess.run(['cc', '-std=c11', '-fsanitize=address,undefined',
                    f'-I{ROOT / "lib/fast-protocol/include"}', f'-I{ROOT / "lib/hid-defs/include"}',
                    f'-I{ROOT / "lib/kmbox-commands"}',
                    str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    print('Fixed-size bridge protocol and fragmented replies: passed')

    metro_state = r'''
#define STATUS_BUFFER_SIZE 128
enum {KMBOX_CONNECTED = 1};
static char status_buffer[STATUS_BUFFER_SIZE], last_status[STATUS_BUFFER_SIZE];
static unsigned status_buffer_idx, status_count, kmbox_response_count, kmbox_state;
static unsigned kmbox_humanization_mode, kmbox_inject_mode, kmbox_max_per_frame;
static unsigned kmbox_queue_depth, kmbox_queue_capacity, kmbox_total_injected, kmbox_queue_overflows;
static unsigned kmbox_gamepad_buttons;
static int16_t kmbox_gamepad_sticks[4];
static float kmbox_temperature_c;
static bool kmbox_humanization_valid, kmbox_jitter_enabled, kmbox_velocity_matching;
static bool kmbox_console_mode, kmbox_console_auth_complete;
static void process_status_message(const char *msg, size_t len) {
    assert(len < sizeof(last_status)); memcpy(last_status, msg, len + 1); status_count++;
}
'''
    metro_test = r'''
static void feed(const void *p, size_t n) {
    memcpy(incoming, p, n); incoming_len = n; uart_rx_task();
}
int main(void) {
    const uint8_t reply[] = {FAST_CMD_RESPONSE, 0, 10, 13, 'k', 'm', '.', 0};
    feed(reply, 3); loss_pending = true; feed("", 0);
    feed(reply, sizeof(reply));
    assert(kmbox_response_count == 1 && forwarded_len == 8 && memcmp(forwarded, reply, 8) == 0);
    feed("OLD", 3); loss_pending = true; feed("", 0);
    feed("PONG\r\n", 6);
    assert(status_count == 1 && strcmp(last_status, "PONG") == 0);
    const uint8_t info[] = {FAST_CMD_INFO, 2, 1, 16, 4, 250, 0, 3};
    feed(info, 4); loss_pending = true; feed("", 0); feed(info, sizeof(info));
    assert(kmbox_humanization_valid && kmbox_humanization_mode == 2 && kmbox_max_per_frame == 16);
    assert(kmbox_temperature_c == 25.0f);
}
'''
    source.write_text(prefix + metro_state + function('bridge/main.c', 'uart_rx_task') + metro_test)
    subprocess.run(['cc', '-std=c11', '-fsanitize=address,undefined',
                    f'-I{ROOT / "lib/fast-protocol/include"}', f'-I{ROOT / "lib/hid-defs/include"}',
                    f'-I{ROOT / "lib/kmbox-commands"}', str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    print('Metro binary/text parser reset after DMA loss: passed')
