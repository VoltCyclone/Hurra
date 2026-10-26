/**
 * Fast Binary Command Protocol
 *
 * Shared header for the 8-byte fixed-size binary command protocol used
 * between Bridge and KMBox over UART.  Included by both firmware targets.
 *
 * Contains:
 *   - Command IDs (FAST_CMD_*)
 *   - Injection mode constants (INJECT_MODE_*)
 *   - Packed struct typedefs for type-punned parsing
 *   - Inline packet builders
 *   - HID button masks (via hid_defs.h)
 */

#ifndef FAST_PROTOCOL_H
#define FAST_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "hid_defs.h"

// ============================================================================
// Command IDs (8-byte fixed-size packets, Bridge <-> KMBox UART)
// ============================================================================

#define FAST_CMD_MOUSE_MOVE     0x01    // Direct accumulator (buttons + wheel + move)
#define FAST_CMD_MOUSE_CLICK    0x02    // Button click with repeat count
#define FAST_CMD_SMOOTH_MOVE    0x07    // Smooth injection queue (humanized)
#define FAST_CMD_SMOOTH_CONFIG  0x08    // Configure smooth injection
#define FAST_CMD_SMOOTH_CLEAR   0x09    // Clear smooth injection queue
#define FAST_CMD_TIMED_MOVE     0x0A    // Movement with timestamp for sync
#define FAST_CMD_MULTI_MOVE     0x0B    // Multiple movements in one packet
#define FAST_CMD_KEY_COMBO      0x0C    // Keyboard key combination
#define FAST_CMD_KEY_PRESS      0x0C    // Single key press (alias for KEY_COMBO)
#define FAST_CMD_INFO           0x0D    // Request system info
#define FAST_CMD_INFO_EXT       0x0E    // Request extended stats
#define FAST_CMD_SET_HUMAN      0x10    // [cmd][mode][pad*6], mode 0..2
#define FAST_CMD_INFO_REQUEST   0x11    // Text-safe request; response remains FAST_CMD_INFO
#define FAST_CMD_CYCLE_HUMAN    0x0F    // Cycle humanization mode
#define FAST_CMD_XBOX_INPUT     0x20    // Xbox gamepad: buttons + triggers
#define FAST_CMD_XBOX_STICK_L   0x22    // Xbox left stick X/Y
#define FAST_CMD_XBOX_STICK_R   0x23    // Xbox right stick X/Y
#define FAST_CMD_XBOX_RELEASE   0x27    // Xbox clear all injection overrides
#define FAST_CMD_XBOX_STATUS    0x28    // Xbox console mode status report
#define FAST_CMD_SYNC           0xFC    // Clock synchronization
#define FAST_CMD_RESPONSE       0xFD    // Generic response
#define FAST_CMD_PING           0xFE    // Keepalive ping
#define FAST_CMD_PACKET_SIZE    8       // Fixed 8-byte packet size

// ============================================================================
// Injection Modes
// ============================================================================
// On the KMBox side, smooth_injection.h provides inject_mode_t as an enum
// with the same values. It sets _INJECT_MODES_DEFINED to suppress these
// macros so the enum and macros don't collide in the same translation unit.

#ifndef _INJECT_MODES_DEFINED
#define INJECT_MODE_IMMEDIATE        0
#define INJECT_MODE_SMOOTH           1
#define INJECT_MODE_VELOCITY_MATCHED 2
#define INJECT_MODE_MICRO            3
#endif

// ============================================================================
// Packed Struct Typedefs (for type-punned parsing on KMBox side)
// ============================================================================

// Timed move command: 0x0A (for clock-synchronized injection)
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;       // 0x0A
    int16_t  x;         // X movement
    int16_t  y;         // Y movement
    uint16_t time_us;   // Execution time offset (microseconds from sync)
    uint8_t  mode;      // Injection mode
} fast_cmd_timed_t;

_Static_assert(sizeof(fast_cmd_timed_t) == 8, "fast_cmd_timed_t must be 8 bytes");

// Mouse move command: 0x01
typedef struct __attribute__((packed)) {
    uint8_t  cmd;       // 0x01
    int16_t  x, y;      // Movement
    uint8_t  buttons;   // Button state
    int8_t   wheel;     // Wheel movement
    uint8_t  pad[2];    // Padding to 8 bytes
} fast_cmd_move_t;

typedef fast_cmd_move_t fast_cmd_mouse_move_t;  // Alias for compatibility

// Multi-move command: 0x0B
typedef struct __attribute__((packed)) {
    uint8_t  cmd;       // 0x0B
    int8_t   x1, y1;    // First movement
    int8_t   x2, y2;    // Second movement
    int8_t   x3, y3;    // Third movement
    uint8_t  pad;       // Padding to 8 bytes
} fast_cmd_multi_t;

// Click command: 0x02
typedef struct __attribute__((packed)) {
    uint8_t  cmd;       // 0x02
    uint8_t  button;    // Button mask
    uint8_t  count;     // Click count
    uint8_t  pad[5];    // Padding to 8 bytes
} fast_cmd_click_t;

// Key press/combo command: 0x0C
typedef struct __attribute__((packed)) {
    uint8_t  cmd;        // 0x0C
    uint8_t  modifiers;  // Modifier keys
    uint8_t  keycode;    // Primary keycode (for single key)
    uint8_t  keys[5];    // Additional keycodes (for combo)
} fast_cmd_key_t;

typedef fast_cmd_key_t fast_cmd_combo_t;  // Alias for compatibility

// Smooth move command: 0x07
typedef struct __attribute__((packed)) {
    uint8_t  cmd;       // 0x07
    int16_t  x;         // X movement
    int16_t  y;         // Y movement
    uint8_t  mode;      // Injection mode
    uint8_t  pad[2];    // Padding to 8 bytes
} fast_cmd_smooth_t;

// Config command: 0x08
typedef struct __attribute__((packed)) {
    uint8_t  cmd;            // 0x08
    uint8_t  max_per_frame;  // Max pixels per frame
    uint8_t  vel_match;      // Velocity matching enable
    uint8_t  pad[5];         // Padding to 8 bytes
} fast_cmd_config_t;

// Sync command: 0xFC
typedef struct __attribute__((packed)) {
    uint8_t  cmd;        // 0xFC
    uint32_t timestamp;  // PC timestamp
    uint16_t seq_num;    // Sequence number
    uint8_t  pad;        // Padding to 8 bytes
} fast_cmd_sync_t;

// ============================================================================
// Inline Packet Builders
// ============================================================================

/**
 * Build FAST_CMD_SMOOTH_MOVE (0x07) packet.
 * KMBox routes this through smooth_inject_movement() which applies
 * humanization (easing, subdivision, tremor, overshoot) automatically.
 */
static inline size_t fast_build_smooth_move(uint8_t *buf, int16_t x, int16_t y, uint8_t mode) {
    buf[0] = FAST_CMD_SMOOTH_MOVE;
    buf[1] = (uint8_t)(x & 0xFF);
    buf[2] = (uint8_t)((x >> 8) & 0xFF);
    buf[3] = (uint8_t)(y & 0xFF);
    buf[4] = (uint8_t)((y >> 8) & 0xFF);
    buf[5] = mode;
    buf[6] = 0;
    buf[7] = 0;
    return FAST_CMD_PACKET_SIZE;
}

/**
 * Build FAST_CMD_MOUSE_MOVE (0x01) packet.
 * KMBox routes this through kmbox_add_mouse_movement() (direct accumulator).
 * No smooth queue / humanization subdivision.
 */
static inline size_t fast_build_mouse_move(uint8_t *buf, int16_t x, int16_t y,
                                           uint8_t buttons, int8_t wheel) {
    buf[0] = FAST_CMD_MOUSE_MOVE;
    buf[1] = (uint8_t)(x & 0xFF);
    buf[2] = (uint8_t)((x >> 8) & 0xFF);
    buf[3] = (uint8_t)(y & 0xFF);
    buf[4] = (uint8_t)((y >> 8) & 0xFF);
    buf[5] = buttons;
    buf[6] = (uint8_t)wheel;
    buf[7] = 0;
    return FAST_CMD_PACKET_SIZE;
}

/**
 * Build FAST_CMD_MOUSE_CLICK (0x02) packet.
 * KMBox generates press + release pairs internally.
 */
static inline size_t fast_build_mouse_click(uint8_t *buf, uint8_t button, uint8_t count) {
    buf[0] = FAST_CMD_MOUSE_CLICK;
    buf[1] = button;
    buf[2] = count;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
    return FAST_CMD_PACKET_SIZE;
}

// Shared builders for fixed-size control packets.
static inline size_t fast_build_ping(uint8_t *buf) {
    memset(buf, 0, FAST_CMD_PACKET_SIZE);
    buf[0] = FAST_CMD_PING;
    return FAST_CMD_PACKET_SIZE;
}

static inline size_t fast_build_set_human(uint8_t *buf, uint8_t mode) {
    memset(buf, 0, FAST_CMD_PACKET_SIZE);
    buf[0] = FAST_CMD_SET_HUMAN;
    buf[1] = mode;
    return FAST_CMD_PACKET_SIZE;
}

static inline size_t fast_build_smooth_config(uint8_t *buf, uint8_t max_per_frame, uint8_t vel_match) {
    memset(buf, 0, FAST_CMD_PACKET_SIZE);
    buf[0] = FAST_CMD_SMOOTH_CONFIG;
    buf[1] = max_per_frame;
    buf[2] = vel_match;
    return FAST_CMD_PACKET_SIZE;
}

#endif // FAST_PROTOCOL_H
