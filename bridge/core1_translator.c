/**
 * Core1 Protocol Translator Implementation
 */

#include "core1_translator.h"
#include "makcu_translator.h"
#include "ferrum_translator.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include <string.h>
#include <stdio.h>

// Lockless ring buffers for inter-core communication
static queued_makcu_frame_t makcu_queue[MAKCU_FRAME_QUEUE_SIZE] __attribute__((aligned(4)));
static queued_kmbox_cmd_t kmbox_queue[KMBOX_CMD_QUEUE_SIZE] __attribute__((aligned(4)));

// SPSC indices: acquire/release orders slot data and prevents early slot reuse.
static uint8_t makcu_head = 0;
static uint8_t makcu_tail = 0;
static uint8_t kmbox_head = 0;
static uint8_t kmbox_tail = 0;

// Statistics
static volatile uint32_t frames_queued = 0;
static volatile uint32_t cmds_produced = 0;
static volatile uint32_t queue_overflows = 0;

// Only fixed-size UART commands belong in this queue. Text responses (such
// as VERSION) must not be copied into packet slots or forwarded to the UART.
static bool queue_translated_packet(const translated_cmd_t *result) {
    if (result->result != TRANSLATE_OK ||
        result->length != sizeof(kmbox_queue[0].packet)) return false;
    uint8_t head = __atomic_load_n(&kmbox_head, __ATOMIC_RELAXED);
    uint8_t next = (head + 1) & (KMBOX_CMD_QUEUE_SIZE - 1);
    if (next == __atomic_load_n(&kmbox_tail, __ATOMIC_ACQUIRE)) {
        __atomic_add_fetch(&queue_overflows, 1, __ATOMIC_RELAXED);
        return false;
    }
    memcpy(kmbox_queue[head].packet, result->buffer, result->length);
    __atomic_store_n(&kmbox_head, next, __ATOMIC_RELEASE);
    __atomic_add_fetch(&cmds_produced, 1, __ATOMIC_RELAXED);
    return true;
}

// Consume one input frame. Kept separate from idle waiting so wake-ups always
// recheck the queue, including events received just before WFE.
static bool translate_one_frame(void) {
    uint8_t tail = __atomic_load_n(&makcu_tail, __ATOMIC_RELAXED);
    uint8_t head = __atomic_load_n(&makcu_head, __ATOMIC_ACQUIRE);
    if (head == tail) return false;

    queued_makcu_frame_t *frame = &makcu_queue[tail];
    translated_cmd_t result;
    makcu_translate_command(frame->cmd, frame->payload, frame->payload_len, &result);
    queue_translated_packet(&result);
    __atomic_store_n(&makcu_tail, (tail + 1) & (MAKCU_FRAME_QUEUE_SIZE - 1), __ATOMIC_RELEASE);
    return true;
}

static void __not_in_flash_func(core1_main)(void) {
    while (1) {
        if (!translate_one_frame()) {
            __wfe();  // Matches the producer's SEV; a pending event prevents sleep.
        }
    }
}

// Initialize Core1 translator
void core1_translator_init(void) {
    // Reset queues
    makcu_head = 0;
    makcu_tail = 0;
    kmbox_head = 0;
    kmbox_tail = 0;
    
    frames_queued = 0;
    cmds_produced = 0;
    queue_overflows = 0;
    
    // Launch Core1
    multicore_launch_core1(core1_main);
    
    printf("Core1 translator launched\n");
}

// Queue Makcu frame for translation
bool core1_queue_makcu_frame(uint8_t cmd, const uint8_t* payload, uint16_t len) {
    if (len > 256 || (len > 0 && !payload)) {
        return false;  // Payload too large
    }
    
    uint8_t head = __atomic_load_n(&makcu_head, __ATOMIC_RELAXED);
    uint8_t next_head = (head + 1) & (MAKCU_FRAME_QUEUE_SIZE - 1);
    
    if (next_head == __atomic_load_n(&makcu_tail, __ATOMIC_ACQUIRE)) {
        __atomic_add_fetch(&queue_overflows, 1, __ATOMIC_RELAXED);
        return false;  // Queue full
    }
    
    // Copy frame to queue
    queued_makcu_frame_t* frame = &makcu_queue[head];
    frame->cmd = cmd;
    frame->payload_len = len;
    if (payload && len > 0) {
        memcpy(frame->payload, payload, len);
    }
    
    __atomic_store_n(&makcu_head, next_head, __ATOMIC_RELEASE);
    __atomic_add_fetch(&frames_queued, 1, __ATOMIC_RELAXED);
    
    // Wake Core1 if idle
    __dsb();  // Complete publication before waking the other core.
    __sev();
    
    return true;
}

// Check if KMBox commands available
bool core1_has_kmbox_commands(void) {
    return __atomic_load_n(&kmbox_head, __ATOMIC_ACQUIRE) !=
           __atomic_load_n(&kmbox_tail, __ATOMIC_RELAXED);
}

// Dequeue translated KMBox command
bool core1_dequeue_kmbox_command(uint8_t* packet_out) {
    uint8_t tail = __atomic_load_n(&kmbox_tail, __ATOMIC_RELAXED);
    uint8_t head = __atomic_load_n(&kmbox_head, __ATOMIC_ACQUIRE);
    
    if (head == tail) {
        return false;  // Queue empty
    }
    
    // Copy packet
    memcpy(packet_out, kmbox_queue[tail].packet, 8);
    
    // Advance tail (consume command)
    __atomic_store_n(&kmbox_tail, (tail + 1) & (KMBOX_CMD_QUEUE_SIZE - 1), __ATOMIC_RELEASE);
    
    return true;
}

// Get statistics
void core1_get_stats(uint32_t* out_frames, uint32_t* out_cmds, uint32_t* out_overflows) {
    if (out_frames) *out_frames = __atomic_load_n(&frames_queued, __ATOMIC_RELAXED);
    if (out_cmds) *out_cmds = __atomic_load_n(&cmds_produced, __ATOMIC_RELAXED);
    if (out_overflows) *out_overflows = __atomic_load_n(&queue_overflows, __ATOMIC_RELAXED);
}
