"""Host regression checks for bridge parsing and translated queue bounds."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
TEST = r'''
#include <assert.h>
#include <limits.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <setjmp.h>
#include "core1_translator.c"
#include "input_classifier.h"

static jmp_buf idle_exit;
static unsigned idle_calls;
static void test_wfe(void) {
    if (idle_calls++ == 0) {
        uint8_t move[] = {7, 0, 9, 0};
        assert(core1_queue_makcu_frame(MAKCU_CMD_MOVE, move, sizeof(move)));
        return;
    }
    longjmp(idle_exit, 1);
}

#define STRESS_COUNT 100000
static void *consume_frames(void *unused) {
    (void)unused;
    for (unsigned i = 0; i < STRESS_COUNT; i++) {
        while (!translate_one_frame()) sched_yield();
        uint8_t packet[8];
        assert(core1_dequeue_kmbox_command(packet));
        assert(packet[1] == (uint8_t)i && packet[2] == (uint8_t)(i >> 8));
        assert(packet[3] == (uint8_t)~i && packet[4] == (uint8_t)(~i >> 8));
    }
    return NULL;
}

static void *produce_packets(void *unused) {
    (void)unused;
    for (unsigned i = 0; i < STRESS_COUNT; i++) {
        translated_cmd_t result = {.length = 8, .result = TRANSLATE_OK};
        memcpy(result.buffer, &i, sizeof(i));
        memcpy(result.buffer + 4, &i, sizeof(i));
        while (!queue_translated_packet(&result)) sched_yield();
    }
    return NULL;
}

int main(void) {
    ferrum_translated_t out;
    const char raw[] = {'k','m','.','m','o','v','e','(','1',',','2',')'};
    assert(ferrum_translate_line(raw, sizeof(raw), &out));
    assert(out.length == 8 && out.buffer[1] == 1 && out.buffer[3] == 2);
    // A truncated span must not read the rest of its backing string.
    assert(!ferrum_translate_line("km.move(1,2)", 7, &out));
    const char *bad[] = {
        "km.move(99999999999999999999999999999,0)",
        "km.move(-2147483649,0)", "km.move(2147483648,0)",
        "km.move(-2147483648,0)", "km.move(32768,0)",
        "km.move(0,-32769)", "km.wheel(128)", "km.wheel(-129)"
    };
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); i++)
        assert(!ferrum_translate_line(bad[i], strlen(bad[i]), &out));
    assert(ferrum_translate_line("km.move(-32768,32767)", 21, &out));
    input_classifier_reset();
    input_classifier_record_move(INT16_MIN, 0);
    assert(ic_total_magnitude == 32768);
    input_classifier_record_move(INT16_MIN, INT16_MIN);
    assert(ic_total_magnitude == 98304);

    translated_cmd_t result;
    makcu_translate_command(MAKCU_CMD_VERSION, NULL, 0, &result);
    assert(result.length > 8);
    assert(!queue_translated_packet(&result));
    assert(!core1_has_kmbox_commands());
    assert(!core1_queue_makcu_frame(MAKCU_CMD_MOVE, NULL, 4));
    uint8_t move[] = {1, 0, 2, 0};
    makcu_translate_command(MAKCU_CMD_MOVE, move, sizeof(move), &result);
    assert(queue_translated_packet(&result));
    uint8_t packet[8];
    assert(core1_dequeue_kmbox_command(packet));
    assert(memcmp(packet, result.buffer, 8) == 0);
    assert(!core1_dequeue_kmbox_command(packet));

    // Exercise the actual idle loop: wake, consume, then return to waiting.
    if (setjmp(idle_exit) == 0) core1_main();
    assert(idle_calls == 2 && core1_dequeue_kmbox_command(packet));
    assert(packet[1] == 7 && packet[3] == 9);

    pthread_t thread;
    assert(pthread_create(&thread, NULL, consume_frames, NULL) == 0);
    for (unsigned i = 0; i < STRESS_COUNT; i++) {
        uint8_t move[] = {(uint8_t)i, (uint8_t)(i >> 8),
                          (uint8_t)~i, (uint8_t)(~i >> 8)};
        while (!core1_queue_makcu_frame(MAKCU_CMD_MOVE, move, sizeof(move))) sched_yield();
    }
    assert(pthread_join(thread, NULL) == 0);

    assert(pthread_create(&thread, NULL, produce_packets, NULL) == 0);
    for (unsigned i = 0; i < STRESS_COUNT; i++) {
        while (!core1_dequeue_kmbox_command(packet)) sched_yield();
        unsigned first, second;
        memcpy(&first, packet, sizeof(first));
        memcpy(&second, packet + 4, sizeof(second));
        assert(first == i && second == i);
    }
    assert(pthread_join(thread, NULL) == 0);
    return 0;
}
'''
with tempfile.TemporaryDirectory() as temp:
    temp = Path(temp)
    (temp / 'pico').mkdir()
    (temp / 'pico/stdlib.h').write_text('''
#include <stdint.h>
#define __not_in_flash_func(x) x
static inline void __dmb(void) {}
static inline void __sev(void) {}
static void test_wfe(void);
static inline void __wfe(void) {test_wfe();}
static inline void __dsb(void) {}
''')
    (temp / 'pico/sync.h').write_text('')
    (temp / 'pico/multicore.h').write_text('static inline void multicore_launch_core1(void (*f)(void)) {(void)f;}')
    (temp / 'test.c').write_text(TEST)
    includes = [temp, ROOT / 'bridge', ROOT / 'lib/fast-protocol/include', ROOT / 'lib/hid-defs/include']
    subprocess.run(['cc', '-std=c11', '-pthread', '-fsanitize=address,undefined',
                    *[f'-I{p}' for p in includes], str(temp / 'test.c'),
                    str(ROOT / 'bridge/ferrum_translator.c'),
                    str(ROOT / 'bridge/makcu_translator.c'), '-o', str(temp / 'test')], check=True)
    subprocess.run([str(temp / 'test')], check=True)
    print('bridge parsers and queue bounds: passed')
