/**
 * Ferrum KM API → KMBox Binary Fast Command Translator
 *
 * Parses Ferrum text commands (km.move, km.left, etc.) and emits
 * 8-byte binary fast command packets.  The KMBox processes these
 * directly without any string parsing.
 *
 * All commands → FAST_CMD_MOUSE_MOVE (0x01) — the translator still
 *   outputs 0x01 packets.  The movement coalescer in main.c intercepts
 *   move packets (0x01 with non-zero x/y) and re-routes them through
 *   FAST_CMD_SMOOTH_MOVE (0x07) at a controlled rate (≤250Hz) so the
 *   KMBox smooth injection queue can apply full humanization without
 *   overflowing.
 */

#include "ferrum_translator.h"
#include "ferrum_protocol.h"
#include "fast_commands.h"
#include <string.h>
#include <stddef.h>
#include <ctype.h>
#include <limits.h>

// Persistent button state — buttons are sticky (press stays until release)
static uint8_t g_button_state = 0;

void ferrum_translator_init(void) {
    g_button_state = 0;
}

// ============================================================================
// Argument parsing helpers
// ============================================================================

static bool parse_int(const char **p, int32_t *out) {
    while (**p == ' ' || **p == '\t') (*p)++;
    if (**p == '\0' || **p == ')') return false;

    bool neg = false;
    if (**p == '-') { neg = true; (*p)++; }
    else if (**p == '+') { (*p)++; }

    if (!isdigit((unsigned char)**p)) return false;

    uint32_t val = 0;
    uint32_t limit = (uint32_t)INT32_MAX + (neg ? 1u : 0u);
    while (isdigit((unsigned char)**p)) {
        uint32_t digit = (uint32_t)(**p - '0');
        if (val > (limit - digit) / 10u) return false;
        val = val * 10u + digit;
        (*p)++;
    }
    *out = neg ? (val == (uint32_t)INT32_MAX + 1u ? INT32_MIN : -(int32_t)val) : (int32_t)val;
    return true;
}

static void skip_separator(const char **p) {
    while (**p == ' ' || **p == '\t' || **p == ',') (*p)++;
}

// ============================================================================
// Button helpers — update persistent mask, emit FAST_CMD_MOUSE_MOVE
// ============================================================================

static void emit_button_packet(ferrum_translated_t *out, uint8_t mask, bool pressed) {
    if (pressed)
        g_button_state |= mask;
    else
        g_button_state &= ~mask;

    // FAST_CMD_MOUSE_MOVE with zero movement, current buttons, zero wheel
    out->length = (uint16_t)fast_build_mouse_move(out->buffer, 0, 0, g_button_state, 0);
    out->needs_response = true;
}

// ============================================================================
// Main translator
// ============================================================================

bool ferrum_translate_line(const char *line, size_t len, ferrum_translated_t *out) {
    if (!line || len == 0 || len >= FERRUM_MAX_LINE || !out) return false;

    // Respect the supplied span; callers need not provide a trailing NUL.
    char bounded_line[FERRUM_MAX_LINE];
    memcpy(bounded_line, line, len);
    bounded_line[len] = '\0';
    line = bounded_line;

    memset(out, 0, sizeof(ferrum_translated_t));

    // Must start with "km."
    if (len < FERRUM_PREFIX_LEN || strncmp(line, FERRUM_PREFIX, FERRUM_PREFIX_LEN) != 0)
        return false;

    const char *p = line + FERRUM_PREFIX_LEN;

    // ----- km.move(x, y) → FAST_CMD_MOUSE_MOVE (direct accumulator) --------
    if (strncmp(p, FERRUM_CMD_MOVE, strlen(FERRUM_CMD_MOVE)) == 0) {
        p += strlen(FERRUM_CMD_MOVE);
        if (*p != '(') return false;
        p++;

        int32_t x, y;
        if (!parse_int(&p, &x)) return false;
        skip_separator(&p);
        if (!parse_int(&p, &y)) return false;
        if (*p != ')' || x < INT16_MIN || x > INT16_MAX ||
            y < INT16_MIN || y > INT16_MAX) return false;

        // Direct accumulator (0x01) — the coalescer in main.c intercepts
        // move packets and re-routes through smooth queue (0x07) at a
        // controlled rate for full humanization.
        out->length = (uint16_t)fast_build_mouse_move(
            out->buffer, (int16_t)x, (int16_t)y, g_button_state, 0);
        out->needs_response = true;
        return true;
    }

    // ----- km.wheel(amount) → FAST_CMD_MOUSE_MOVE with wheel ---------------
    if (strncmp(p, FERRUM_CMD_WHEEL, strlen(FERRUM_CMD_WHEEL)) == 0) {
        p += strlen(FERRUM_CMD_WHEEL);
        if (*p != '(') return false;
        p++;

        int32_t wheel;
        if (!parse_int(&p, &wheel)) return false;
        if (*p != ')' || wheel < INT8_MIN || wheel > INT8_MAX) return false;

        out->length = (uint16_t)fast_build_mouse_move(
            out->buffer, 0, 0, g_button_state, (int8_t)wheel);
        out->needs_response = true;
        return true;
    }

    // ----- Button commands → FAST_CMD_MOUSE_MOVE with updated mask ----------

    // km.left(state)
    if (strncmp(p, FERRUM_CMD_LEFT, strlen(FERRUM_CMD_LEFT)) == 0) {
        p += strlen(FERRUM_CMD_LEFT);
        if (*p != '(') return false; p++;
        int32_t state; if (!parse_int(&p, &state)) return false;
        if (*p != ')') return false;
        emit_button_packet(out, FAST_BTN_LEFT, state != 0);
        return true;
    }

    // km.right(state)
    if (strncmp(p, FERRUM_CMD_RIGHT, strlen(FERRUM_CMD_RIGHT)) == 0) {
        p += strlen(FERRUM_CMD_RIGHT);
        if (*p != '(') return false; p++;
        int32_t state; if (!parse_int(&p, &state)) return false;
        if (*p != ')') return false;
        emit_button_packet(out, FAST_BTN_RIGHT, state != 0);
        return true;
    }

    // km.middle(state)
    if (strncmp(p, FERRUM_CMD_MIDDLE, strlen(FERRUM_CMD_MIDDLE)) == 0) {
        p += strlen(FERRUM_CMD_MIDDLE);
        if (*p != '(') return false; p++;
        int32_t state; if (!parse_int(&p, &state)) return false;
        if (*p != ')') return false;
        emit_button_packet(out, FAST_BTN_MIDDLE, state != 0);
        return true;
    }

    // km.side1(state)
    if (strncmp(p, FERRUM_CMD_SIDE1, strlen(FERRUM_CMD_SIDE1)) == 0) {
        p += strlen(FERRUM_CMD_SIDE1);
        if (*p != '(') return false; p++;
        int32_t state; if (!parse_int(&p, &state)) return false;
        if (*p != ')') return false;
        emit_button_packet(out, FAST_BTN_BACK, state != 0);
        return true;
    }

    // km.side2(state)
    if (strncmp(p, FERRUM_CMD_SIDE2, strlen(FERRUM_CMD_SIDE2)) == 0) {
        p += strlen(FERRUM_CMD_SIDE2);
        if (*p != '(') return false; p++;
        int32_t state; if (!parse_int(&p, &state)) return false;
        if (*p != ')') return false;
        emit_button_packet(out, FAST_BTN_FORWARD, state != 0);
        return true;
    }

    // Keyboard commands — unsupported
    if (strncmp(p, FERRUM_CMD_DOWN, strlen(FERRUM_CMD_DOWN)) == 0 ||
        strncmp(p, FERRUM_CMD_UP, strlen(FERRUM_CMD_UP)) == 0 ||
        strncmp(p, FERRUM_CMD_PRESS, strlen(FERRUM_CMD_PRESS)) == 0 ||
        strncmp(p, FERRUM_CMD_MULTIDOWN, strlen(FERRUM_CMD_MULTIDOWN)) == 0 ||
        strncmp(p, FERRUM_CMD_MULTIUP, strlen(FERRUM_CMD_MULTIUP)) == 0 ||
        strncmp(p, FERRUM_CMD_MULTIPRESS, strlen(FERRUM_CMD_MULTIPRESS)) == 0) {
        return false;
    }

    return false;
}
