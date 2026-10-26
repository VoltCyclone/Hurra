"""Exercise the proxy's production TX queue with deferred DMA and two producers."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
assert 'pico_enable_stdio_uart(PIOKMbox 0)' in (ROOT / 'CMakeLists.txt').read_text()
source = (ROOT / 'kmbox_serial_handler.c').read_text()
queue_state = source[source.index('#define UART_TX_RECORD_SIZE'):source.index('// DMA-based RX:')]


def function(name):
    match = re.search(r'^(?:static )?(?:bool|void|uint32_t)\s+(?:__not_in_flash_func\()?'
                      + name + r'\)?\([^;]*?\)\s*\{', source, re.M)
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
#include <pthread.h>
#include <sched.h>
#include "hardware/sync.h"
#include "kmbox_commands.c"
#define __not_in_flash_func(x) x
static void __dmb(void) {}
#define KMBOX_UART 0
static uint8_t output[65536];
static size_t output_len, dma_len;
static const uint8_t *dma_source;
static bool busy, tx_irq_enabled;
static unsigned fifo_budget;
static bool dma_channel_is_busy(int ch) {assert(ch == 1); return busy;}
static void dma_channel_set_read_addr(int ch, const void *p, bool trigger) {
    assert(ch == 1 && !busy && !trigger); dma_source = p;
}
static void dma_channel_set_trans_count(int ch, size_t n, bool trigger) {
    assert(ch == 1 && !busy && trigger && n > 0); dma_len = n; busy = true;
}
static void uart_set_irq_enables(int uart, bool rx, bool tx) {
    assert(uart == 0 && !rx); tx_irq_enabled = tx;
}
static bool uart_is_writable(int uart) {assert(uart == 0); return fifo_budget > 0;}
static void uart_putc_raw(int uart, uint8_t c) {
    assert(uart == 0 && fifo_budget > 0); fifo_budget--; output[output_len++] = c;
}
static void tx_service_locked(void);
'''
test = r'''
static void complete_dma(void) {
    uint32_t save = spin_lock_blocking(tx_lock);
    if (busy) {
        assert(output_len + dma_len <= sizeof(output));
        memcpy(output + output_len, dma_source, dma_len);
        output_len += dma_len; busy = false;
    }
    tx_service_locked();
    spin_unlock(tx_lock, save);
}
static void drain(void) {
    while (tx_head != tx_tail) complete_dma();
}
static void reset(void) {
    assert(!busy);
    tx_head = tx_tail = 0; tx_offset = 0; tx_dma_active = false;
    g_uart_tx_dropped = 0; output_len = 0; uart_tx_dma_chan = 1;
}
static unsigned finished;
#define COUNT 1000
static void *producer(void *arg) {
    uint8_t record[8]; memset(record, (int)(uintptr_t)arg, sizeof(record));
    for (unsigned i = 0; i < COUNT; i++)
        while (!uart_send_bytes(record, sizeof(record))) sched_yield();
    __atomic_add_fetch(&finished, 1, __ATOMIC_RELEASE);
    return NULL;
}
int main(void) {
    tx_lock = spin_lock_init((unsigned)spin_lock_claim_unused(true));
    reset();
    kmbox_send_response("HELLO");
    const uint8_t binary[] = {0xFD, 0, 1, 2, 3, 4, 5, 6};
    assert(uart_send_bytes(binary, sizeof(binary)));
    kmbox_commands_init();
    kmbox_set_output_callback(command_reply_write);
    kmbox_process_serial_line("km.monitor()", 12, "\n", 1, 100);
    drain();
    assert(output_len == 7 + 8 + 7);
    assert(memcmp(output, "HELLO\r\n", 7) == 0);
    assert(memcmp(output + 7, binary, 8) == 0);
    assert(memcmp(output + 15, "0\r\n>>> ", 7) == 0);

    reset();
    for (unsigned i = 0; i < UART_TX_RECORD_COUNT - 1; i++) kmbox_send_response("A");
    assert(!uart_send_bytes(binary, sizeof(binary)));
    assert(kmbox_get_tx_dropped_bytes() == 8);
    drain(); assert(output_len == 7 * 3);
    for (unsigned i = 0; i < 7; i++) assert(memcmp(output + i * 3, "A\r\n", 3) == 0);

    reset();
    uint8_t oversized[UART_TX_RECORD_SIZE + 1]; memset(oversized, 'X', sizeof(oversized));
    assert(!uart_send_bytes(oversized, sizeof(oversized)));
    assert(tx_head == tx_tail && !busy && output_len == 0);
    assert(kmbox_get_tx_dropped_bytes() == sizeof(oversized));
    // Account for the suffix before admitting a text response.
    oversized[UART_TX_RECORD_SIZE - 1] = 0;
    kmbox_send_response((const char *)oversized);
    assert(tx_head == tx_tail && !busy && output_len == 0);
    assert(kmbox_get_tx_dropped_bytes() == sizeof(oversized) * 2);

    // IRQ-to-DMA handoff resumes an already partially transmitted record.
    reset(); uart_tx_dma_chan = -1;
    assert(uart_send_bytes((const uint8_t *)"ABCD", 4));
    assert(tx_irq_enabled);
    fifo_budget = 2; on_uart_irq(); assert(tx_offset == 2 && output_len == 2);
    uart_tx_dma_chan = 1; tx_service(); drain();
    assert(output_len == 4 && memcmp(output, "ABCD", 4) == 0);

    reset();
    pthread_t a, b;
    assert(pthread_create(&a, NULL, producer, (void *)(uintptr_t)0xAA) == 0);
    assert(pthread_create(&b, NULL, producer, (void *)(uintptr_t)0xBB) == 0);
    while (__atomic_load_n(&finished, __ATOMIC_ACQUIRE) != 2) {complete_dma(); sched_yield();}
    assert(pthread_join(a, NULL) == 0 && pthread_join(b, NULL) == 0);
    drain();
    assert(output_len == COUNT * 2 * 8);
    unsigned a_count = 0, b_count = 0;
    for (size_t i = 0; i < output_len; i += 8) {
        assert(output[i] == 0xAA || output[i] == 0xBB);
        for (unsigned j = 1; j < 8; j++) assert(output[i + j] == output[i]);
        if (output[i] == 0xAA) a_count++; else b_count++;
    }
    assert(a_count == COUNT && b_count == COUNT);
}
'''
with tempfile.TemporaryDirectory() as temp:
    temp = Path(temp)
    (temp / 'hardware').mkdir()
    (temp / 'hardware/sync.h').write_text('''
#pragma once
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
typedef pthread_mutex_t spin_lock_t;
static spin_lock_t locks[32];
static int next_lock = 24;
static int spin_lock_claim_unused(bool required) {assert(required); return next_lock++;}
static spin_lock_t *spin_lock_init(unsigned n) {
    assert(n < 32); pthread_mutex_init(&locks[n], NULL); return &locks[n];
}
static uint32_t spin_lock_blocking(spin_lock_t *p) {pthread_mutex_lock(p); return 0;}
static void spin_unlock(spin_lock_t *p, uint32_t save) {(void)save; pthread_mutex_unlock(p);}
''')
    unit = temp / 'test.c'
    binary = temp / 'test'
    unit.write_text(prefix + queue_state + '\n'.join(function(f) for f in [
        'tx_service_locked', 'tx_service', 'on_uart_irq', 'uart_enqueue_record',
        'uart_send_bytes', 'command_reply_write', 'kmbox_send_response', 'kmbox_get_tx_dropped_bytes']) + test)
    subprocess.run(['cc', '-std=c11', '-pthread', '-fsanitize=address,undefined', f'-I{temp}',
                    f'-I{ROOT / "lib/kmbox-commands"}', f'-I{ROOT / "lib/hid-defs/include"}',
                    str(unit), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    print('Proxy reply routing, record integrity, IRQ/DMA handoff, and concurrent producers: passed')
