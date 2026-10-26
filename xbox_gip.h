/*
 * Xbox GIP (Game Input Protocol) Definitions
 *
 * Protocol structures and shared state for Xbox controller passthrough.
 * GIP runs over vendor-specific USB class (0xFF/0x47/0xD0) and uses
 * 4-byte headers followed by variable-length payloads.
 */

#ifndef XBOX_GIP_H
#define XBOX_GIP_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/sync.h"

//--------------------------------------------------------------------+
// GIP Command IDs
//--------------------------------------------------------------------+

#define GIP_CMD_ACK          0x01
#define GIP_CMD_ANNOUNCE     0x02
#define GIP_CMD_STATUS       0x03
#define GIP_CMD_IDENTIFY     0x04
#define GIP_CMD_POWER        0x05
#define GIP_CMD_AUTH         0x06
#define GIP_CMD_GUIDE        0x07
#define GIP_CMD_AUDIO_CFG    0x08
#define GIP_CMD_RUMBLE       0x09
#define GIP_CMD_LED          0x0A
#define GIP_CMD_SERIAL_NUM   0x1E
#define GIP_CMD_INPUT        0x20
#define GIP_CMD_AUDIO_DATA   0x60

//--------------------------------------------------------------------+
// GIP Header Flags
//--------------------------------------------------------------------+

#define GIP_FLAG_NEED_ACK    0x10   // Sender requests ACK
#define GIP_FLAG_INTERNAL    0x20   // Internal/system packet
#define GIP_FLAG_CHUNK_START 0x40   // First chunk of multi-chunk
#define GIP_FLAG_CHUNK       0x80   // Continuation chunk

//--------------------------------------------------------------------+
// GIP Header Structure (4 bytes)
//--------------------------------------------------------------------+

typedef struct __attribute__((packed)) {
    uint8_t command;    // GIP command ID
    uint8_t client;     // Client ID (device index) + flags
    uint8_t sequence;   // Sequence number
    uint8_t length;     // Payload length (bytes following header)
} gip_header_t;

_Static_assert(sizeof(gip_header_t) == 4, "GIP header must be 4 bytes");

//--------------------------------------------------------------------+
// Xbox Input Report (follows GIP header with command 0x20)
//--------------------------------------------------------------------+

typedef struct __attribute__((packed)) {
    uint16_t buttons;       // Button bitfield
    uint16_t trigger_left;  // Left trigger (0-1023)
    uint16_t trigger_right; // Right trigger (0-1023)
    int16_t  stick_lx;      // Left stick X (-32768 to 32767)
    int16_t  stick_ly;      // Left stick Y (-32768 to 32767)
    int16_t  stick_rx;      // Right stick X (-32768 to 32767)
    int16_t  stick_ry;      // Right stick Y (-32768 to 32767)
} xbox_input_report_t;

_Static_assert(sizeof(xbox_input_report_t) == 14, "Xbox input report must be 14 bytes");

//--------------------------------------------------------------------+
// Xbox Button Bit Definitions (within xbox_input_report_t.buttons)
//--------------------------------------------------------------------+

#define XBOX_BTN_SYNC       (1u << 0)
#define XBOX_BTN_DUMMY      (1u << 1)   // Reserved
#define XBOX_BTN_MENU       (1u << 2)
#define XBOX_BTN_VIEW       (1u << 3)
#define XBOX_BTN_A          (1u << 4)
#define XBOX_BTN_B          (1u << 5)
#define XBOX_BTN_X          (1u << 6)
#define XBOX_BTN_Y          (1u << 7)
#define XBOX_BTN_DPAD_UP    (1u << 8)
#define XBOX_BTN_DPAD_DOWN  (1u << 9)
#define XBOX_BTN_DPAD_LEFT  (1u << 10)
#define XBOX_BTN_DPAD_RIGHT (1u << 11)
#define XBOX_BTN_LB         (1u << 12)
#define XBOX_BTN_RB         (1u << 13)
#define XBOX_BTN_LS         (1u << 14)  // Left stick press
#define XBOX_BTN_RS         (1u << 15)  // Right stick press

//--------------------------------------------------------------------+
// GIP Packet Queue (ring buffer for cross-core communication)
//--------------------------------------------------------------------+

#define GIP_QUEUE_SIZE       16
#define GIP_MAX_PACKET_SIZE  64

_Static_assert((GIP_QUEUE_SIZE & (GIP_QUEUE_SIZE - 1)) == 0,
               "GIP_QUEUE_SIZE must be a power of 2 for bitmask indexing");

typedef struct {
    uint8_t  data[GIP_MAX_PACKET_SIZE];
    uint8_t  length;
} gip_queue_entry_t;

typedef struct {
    gip_queue_entry_t entries[GIP_QUEUE_SIZE];
    volatile uint8_t  head;
    volatile uint8_t  tail;
    spin_lock_t      *lock;
} gip_packet_queue_t;

static inline void gip_queue_init(gip_packet_queue_t *q, unsigned int spin_lock_num) {
    q->head = 0;
    q->tail = 0;
    q->lock = spin_lock_init(spin_lock_num);
}

static inline bool gip_queue_push(gip_packet_queue_t *q, const uint8_t *data, uint8_t len) {
    if (len > GIP_MAX_PACKET_SIZE) return false;
    // Fast-path: check if full without lock (conservative — may spuriously fail)
    uint8_t next = (q->head + 1) & (GIP_QUEUE_SIZE - 1);
    if (next == q->tail) return false;
    uint32_t save = spin_lock_blocking(q->lock);
    next = (q->head + 1) & (GIP_QUEUE_SIZE - 1);
    if (next == q->tail) {
        spin_unlock(q->lock, save);
        return false;
    }
    memcpy(q->entries[q->head].data, data, len);
    q->entries[q->head].length = len;
    q->head = next;
    spin_unlock(q->lock, save);
    return true;
}

static inline bool gip_queue_pop(gip_packet_queue_t *q, uint8_t *data, uint8_t *len) {
    // Fast-path: lockless empty check avoids spinlock contention on idle queues
    if (q->head == q->tail) return false;
    uint32_t save = spin_lock_blocking(q->lock);
    if (q->head == q->tail) {
        spin_unlock(q->lock, save);
        return false;
    }
    *len = q->entries[q->tail].length;
    memcpy(data, q->entries[q->tail].data, *len);
    q->tail = (q->tail + 1) & (GIP_QUEUE_SIZE - 1);
    spin_unlock(q->lock, save);
    return true;
}

static inline bool gip_queue_empty(gip_packet_queue_t *q) {
    return q->head == q->tail;
}

//--------------------------------------------------------------------+
// Auth Proxy State
//--------------------------------------------------------------------+

typedef enum {
    XBOX_AUTH_IDLE,       // No auth in progress
    XBOX_AUTH_PROXYING,   // Auth challenge/response in transit
    XBOX_AUTH_COMPLETE    // Auth completed successfully
} xbox_auth_state_t;

//--------------------------------------------------------------------+
// Gamepad State Accumulator (shared between cores)
//--------------------------------------------------------------------+

typedef struct {
    // Physical state from controller (written by Core 1 host driver)
    xbox_input_report_t physical;

    // Injection overrides from serial bridge (written by Core 0)
    uint16_t inject_buttons;       // OR'd with physical buttons
    uint16_t inject_trigger_left;  // Replaces physical if inject_mask set
    uint16_t inject_trigger_right;
    int16_t  inject_stick_lx;
    int16_t  inject_stick_ly;
    int16_t  inject_stick_rx;
    int16_t  inject_stick_ry;

    // Bitmask of which injection fields are active
    // Bit 0: buttons, Bit 1: triggers, Bit 2: left stick, Bit 3: right stick
    uint8_t  inject_mask;

    // Sequence counter for outgoing input reports
    uint8_t  output_sequence;

    // Flag indicating new physical data available
    volatile bool physical_updated;

    // Spinlock for cross-core access
    spin_lock_t *lock;
} xbox_gamepad_state_t;

#define XBOX_INJECT_BUTTONS   (1u << 0)
#define XBOX_INJECT_TRIGGERS  (1u << 1)
#define XBOX_INJECT_STICK_L   (1u << 2)
#define XBOX_INJECT_STICK_R   (1u << 3)

//--------------------------------------------------------------------+
// Xbox USB Constants
//--------------------------------------------------------------------+

#define XBOX_VID                  0x045E   // Microsoft
#define XBOX_PID                  0x0B12   // Xbox Wireless Controller (2020)
#define XBOX_ITF_CLASS            0xFF     // Vendor specific
#define XBOX_ITF_SUBCLASS         0x47     // GIP
#define XBOX_ITF_PROTOCOL         0xD0     // GIP protocol

#define GIP_KEEPALIVE_INTERVAL_MS 10000    // 10 seconds
#define GIP_HEADER_SIZE           4
#define GIP_INPUT_REPORT_SIZE     (GIP_HEADER_SIZE + sizeof(xbox_input_report_t))

//--------------------------------------------------------------------+
// Global Xbox Mode Flag
//--------------------------------------------------------------------+

extern volatile bool g_xbox_mode;

#endif // XBOX_GIP_H
