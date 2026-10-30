"""Compile selected production functions against a deferred USB-transfer mock.

Run with python3 tests/test_xbox_transfers.py (requires a C compiler).
The mock deliberately retains buffer pointers instead of copying payloads.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def function(file, name):
    source = (ROOT / file).read_text()
    match = re.search(r"^(?:static )?(?:bool|void|uint16_t const \*)\s*(?:__not_in_flash_func\()?"
                      + name + r"\)?\([^;]*?\)\s*\{", source, re.M)
    assert match, name
    end = match.end()
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


gip = re.sub(r'^#include "pico/sync.h"$', '', (ROOT / 'xbox_gip.h').read_text(), flags=re.M)
prefix = r'''
#include <assert.h>
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
typedef int spin_lock_t;
static spin_lock_t locks[32];
static unsigned claim_count;
static int spin_lock_claim_unused(bool required) { assert(required); return 24 + claim_count++; }
static spin_lock_t *spin_lock_init(unsigned n) { assert(n < 32); return &locks[n]; }
static spin_lock_t *spin_lock_instance(unsigned n) {return spin_lock_init(n);}
static uint32_t spin_lock_blocking(spin_lock_t *p) { (void)p; return 0; }
static void spin_unlock(spin_lock_t *p, uint32_t s) { (void)p; (void)s; }
#define __not_in_flash_func(x) x
'''
mock = r'''
static bool busy, reject;
static uint8_t *retained;
static unsigned submitted;
static uint32_t now;
static bool usbh_edpt_busy(uint8_t d, uint8_t e) { (void)d; (void)e; return busy; }
static bool usbh_edpt_xfer(uint8_t d, uint8_t e, uint8_t *p, uint16_t n) {
    (void)d; (void)e; assert(!busy); assert(n > 0);
    if (reject) return false;
    retained = p; busy = true; submitted++; return true;
}
static uint32_t get_absolute_time(void) { return now; }
static uint32_t to_ms_since_boot(uint32_t t) { return t; }
static bool xbox_mounted = true, power_on_pending;
static uint8_t xbox_ep_out = 1, xbox_dev_addr = 1, gip_seq_host;
static uint8_t xfer_buf_out[GIP_MAX_PACKET_SIZE], pending_out_len;
static uint32_t last_keepalive_ms;
static gip_packet_queue_t device_to_host_queue;
'''
host_test = r'''
int main(void) {
    gip_queue_init(&device_to_host_queue, 1);
    uint8_t a[] = {6, 0, 1, 0}, b[] = {6, 0, 2, 0};
    assert(gip_queue_push(&device_to_host_queue, a, sizeof(a)));
    assert(gip_queue_push(&device_to_host_queue, b, sizeof(b)));
    xbox_host_task(); assert(submitted == 1 && retained[2] == 1);
    xbox_host_task(); assert(submitted == 1 && retained[2] == 1);
    busy = false; reject = true;
    xbox_host_task(); assert(pending_out_len == 4 && submitted == 1);
    reject = false; xbox_host_task();
    assert(submitted == 2 && retained[2] == 2 && pending_out_len == 0);
    busy = false; assert(xbox_host_send_power_on());
    assert(retained == xfer_buf_out && retained[0] == GIP_CMD_POWER);
    busy = false; assert(xbox_host_send_keepalive());
    assert(retained == xfer_buf_out && retained[0] == GIP_CMD_STATUS);
}
'''
device_state = r'''
#define TUSB_DESC_STRING 3
static uint16_t xbox_desc_str[33];
static const char *xbox_string_desc[] = {"", "Microsoft", "Xbox Wireless Controller", "123"};
'''
device_test = r'''
int main(void) {
    const uint16_t *d = xbox_get_string_descriptor(0, 0);
    assert(d[0] == 0x0304 && d[1] == 0x0409);
    d = xbox_get_string_descriptor(1, 0);
    assert(d[0] == 0x0314 && d[1] == 'M');
    assert(xbox_get_string_descriptor(4, 0) == NULL);
}
'''
proxy_state = r'''
#define usbd_edpt_busy usbh_edpt_busy
#define usbd_edpt_xfer usbh_edpt_xfer
volatile bool g_xbox_mode = true;
static bool xbox_dev_mounted = true, announce_sent;
static uint8_t xbox_dev_ep_in = 0x81;
static uint8_t proxy_in_buf[GIP_MAX_PACKET_SIZE], pending_in_len;
static gip_packet_queue_t h2d;
static gip_packet_queue_t *xbox_host_get_h2d_queue(void) { return &h2d; }
static void xbox_build_and_send_report(void) { assert(!busy); }
'''
proxy_test = r'''
int main(void) {
    gip_queue_init(&h2d, 1);
    reject = true; xbox_device_task(); assert(!announce_sent);
    reject = false; xbox_device_task();
    assert(announce_sent && retained[0] == GIP_CMD_ANNOUNCE);
    uint8_t a[] = {6, 0, 9, 0};
    assert(gip_queue_push(&h2d, a, sizeof(a)));
    // Input report helper itself guards a busy endpoint in production.
    busy = false; reject = true; xbox_device_task();
    assert(pending_in_len == 4);
    reject = false; xbox_device_task();
    assert(pending_in_len == 0 && retained == proxy_in_buf && retained[2] == 9);
}
'''
ownership_state = r'''
static uint8_t pending_out_len;
static bool power_on_pending;
static xbox_gamepad_state_t gamepad_state;
static gip_packet_queue_t host_to_device_queue, device_to_host_queue;
static int gamepad_lock_num = -1, h2d_lock_num = -1, d2h_lock_num = -1;
'''
ownership_test = r'''
int main(void) {
    assert(xbox_host_driver_init());
    assert(claim_count == 3);
    assert(gamepad_state.lock != host_to_device_queue.lock);
    assert(host_to_device_queue.lock != device_to_host_queue.lock);
    assert(gamepad_state.lock != device_to_host_queue.lock);
    assert(xbox_host_driver_init());
    assert(claim_count == 3);
}
'''
with tempfile.TemporaryDirectory() as temp:
    for name, code in {
        'ownership': prefix + gip + ownership_state + function('xbox_host.c', 'xbox_host_driver_init') + ownership_test,
        'host': prefix + gip + mock + '\n'.join(function('xbox_host.c', f) for f in
            ['xbox_host_send_power_on', 'xbox_host_send_keepalive', 'xbox_host_task']) + host_test,
        'strings': prefix + device_state + function('xbox_device.c', 'xbox_get_string_descriptor') + device_test,
        'proxy': prefix + gip + mock + proxy_state + function('xbox_device.c', 'xbox_send_announce')
            + function('xbox_device.c', 'xbox_device_task') + proxy_test,
    }.items():
        source = Path(temp) / (name + '.c')
        binary = Path(temp) / name
        source.write_text(code)
        subprocess.run(['cc', '-std=c11', '-fsanitize=address,undefined', str(source), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        print(f'{name}: passed')
