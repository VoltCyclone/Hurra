/*
 * KMBox Commands Library
 * Handles serial command parsing and custom HID report generation
 */

#ifndef KMBOX_COMMANDS_H
#define KMBOX_COMMANDS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//--------------------------------------------------------------------+
// Button Definitions
//--------------------------------------------------------------------+

#include "hid_defs.h"

// Legacy aliases — prefer HID_BTN_* from hid_defs.h in new code
#define KMBOX_HID_BTN_LEFT      HID_BTN_LEFT
#define KMBOX_HID_BTN_RIGHT     HID_BTN_RIGHT
#define KMBOX_HID_BTN_MIDDLE    HID_BTN_MIDDLE
#define KMBOX_HID_BTN_BACK      HID_BTN_BACK
#define KMBOX_HID_BTN_FORWARD   HID_BTN_FORWARD

typedef enum {
    KMBOX_BUTTON_LEFT = 0,
    KMBOX_BUTTON_RIGHT,
    KMBOX_BUTTON_MIDDLE,
    KMBOX_BUTTON_SIDE1,
    KMBOX_BUTTON_SIDE2,
    KMBOX_BUTTON_COUNT
} kmbox_button_t;

//--------------------------------------------------------------------+
// Button State Management
//--------------------------------------------------------------------+

typedef struct {
    bool is_pressed;
    bool is_forced;  // True if button state is forced by command
    uint32_t release_time;  // Time when forced release should end (0 if not active)
    bool is_clicking;  // True if button is in a click sequence
    uint32_t click_release_start;  // Time when click press ends and release starts
    uint32_t click_end_time;  // Time when entire click sequence ends
    bool is_locked;  // True if button is locked (physical input masked from output)
} button_state_t;

typedef struct {
    button_state_t buttons[KMBOX_BUTTON_COUNT];
    uint8_t physical_buttons;  // Actual physical button states
    uint32_t last_update_time;
    bool button_callback_enabled;  // True if button state change callback is enabled
    uint8_t last_button_state;     // Last reported button state for callback
    
    // Mouse movement state
    int32_t mouse_x_accumulator;  // Accumulated X movement (int32 to prevent overflow)
    int32_t mouse_y_accumulator;  // Accumulated Y movement (int32 to prevent overflow)
    int16_t wheel_accumulator;     // Accumulated wheel movement
    int16_t pan_accumulator;       // Accumulated pan movement
    
    // Axis lock states
    bool lock_mx;  // Lock X axis (left/right movement)
    bool lock_my;  // Lock Y axis (up/down movement)
    
    // Mouse transform (applied to physical mouse before sending)
    // Scale is fixed-point: 256 = 1.0x, 0 = disabled, -256 = -1.0x (inverted)
    int16_t transform_scale_x;  // X axis scale (256 = 1.0, 0 = block, -256 = invert)
    int16_t transform_scale_y;  // Y axis scale (256 = 1.0, 0 = block, -256 = invert)
    bool transform_enabled;     // Master enable for transforms
    
    // Monitor mode (polling-based button state queries)
    bool monitor_enabled;  // True if monitoring is enabled
} kmbox_state_t;

//--------------------------------------------------------------------+
// Command Parser State
//--------------------------------------------------------------------+

#define KMBOX_CMD_BUFFER_SIZE 64

typedef struct {
    char buffer[KMBOX_CMD_BUFFER_SIZE];
    uint8_t buffer_pos;
    bool in_command;
    bool skip_next_terminator;  // Skip next terminator if it's part of \r\n
    char last_terminator;       // Track last terminator seen ('\r' or '\n')
    char command_terminator[3]; // Store the line terminator(s) used for current command
    uint8_t terminator_len;     // Length of the terminator (1 for \n or \r, 2 for \r\n)
} kmbox_parser_t;

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

// Initialize the kmbox commands module
void kmbox_commands_init(void);
// NULL restores stdout output for hosted tools. Firmware supplies its UART queue.
typedef bool (*kmbox_output_callback_t)(const char *data, size_t len);
void kmbox_set_output_callback(kmbox_output_callback_t callback);

// Process incoming serial data (call this with each received character)
void kmbox_process_serial_char(char c, uint32_t current_time_ms);

// Process a complete command line (without trailing terminator). The caller
// should pass the line contents (len bytes), the terminator bytes (pointer)
// and terminator length (1 or 2). This allows callers to hand over full
// lines from DMA/ring-buffer with a single call instead of per-byte calls.
void kmbox_process_serial_line(const char *line, size_t len, const char *terminator, uint8_t term_len, uint32_t current_time_ms);

// Update button states and handle timing (call this periodically)
void kmbox_update_states(uint32_t current_time_ms);

// Get the current mouse report based on button states
void kmbox_get_mouse_report(uint8_t* buttons, int8_t* x, int8_t* y, int8_t* wheel, int8_t* pan);

// Get the current mouse report with 16-bit precision for high-resolution mice
void kmbox_get_mouse_report_16(uint8_t* buttons, int16_t* x, int16_t* y, int8_t* wheel, int8_t* pan);

// Add mouse movement
void kmbox_add_mouse_movement(int16_t x, int16_t y);
void kmbox_add_wheel_movement(int8_t wheel);
void kmbox_add_pan_movement(int8_t pan);

// Apply mouse transform to physical movement (called internally)
// Returns transformed X,Y values based on current scale settings
void kmbox_transform_movement(int16_t in_x, int16_t in_y, int16_t *out_x, int16_t *out_y);

// Set mouse transform scale (256 = 1.0x, 0 = block axis, -256 = invert)
void kmbox_set_transform(int16_t scale_x, int16_t scale_y, bool enabled);

// Get current transform settings
void kmbox_get_transform(int16_t *scale_x, int16_t *scale_y, bool *enabled);

// Check if there are pending mouse/wheel movements in accumulator
bool kmbox_has_pending_movement(void);

// Add wheel movement
void kmbox_add_wheel_movement(int8_t wheel);

// Set axis lock state
void kmbox_set_axis_lock(bool lock_x, bool lock_y);

// Get axis lock states
bool kmbox_get_lock_mx(void);
bool kmbox_get_lock_my(void);

// Check if any button is currently forced
bool kmbox_has_forced_buttons(void);

// Start a timed button click sequence (press → hold → release with randomized timing)
// button: KMBOX_BUTTON_LEFT through KMBOX_BUTTON_SIDE2
// current_time_ms: current timestamp for timing calculations
void kmbox_start_button_click(kmbox_button_t button, uint32_t current_time_ms);

// Get the current combined button byte (physical | forced) without draining accumulators.
// Used to detect button-only state changes that need an immediate report.
uint8_t kmbox_get_current_buttons(void);

// Atomic check-and-drain: single spinlock acquire to test pending + drain accumulators.
// Returns true if any movement/wheel/pan/button-change was pending and writes drained values.
// Replaces separate kmbox_has_pending_movement() + kmbox_get_mouse_report_16() calls
// to eliminate double spinlock acquisition in the hot path.
bool kmbox_try_drain_mouse_16(uint8_t last_sent_buttons,
                               uint8_t *buttons, int16_t *x, int16_t *y,
                               int8_t *wheel, int8_t *pan);

// Batched accumulation: single spinlock for all axes.
// Symmetric counterpart to kmbox_try_drain_mouse_16().
// Called from Core1 physical mouse handler to reduce spinlock roundtrips.
void kmbox_accumulate_mouse(int16_t x, int16_t y, int8_t wheel, int8_t pan);

// Get button name string for debugging
const char* kmbox_get_button_name(kmbox_button_t button);

// Update physical button states (call this with actual hardware button states)
void kmbox_update_physical_buttons(uint8_t physical_buttons);
// Absolute synthetic button mask; releasing it preserves physical holds.
void kmbox_set_injected_buttons(uint8_t buttons);

// Monitor mode control (polling-based button state queries)
void kmbox_set_monitor_enabled(bool enabled);
bool kmbox_get_monitor_enabled(void);

// Query button states (for monitor mode - returns physical button state)
bool kmbox_isdown_left(void);
bool kmbox_isdown_right(void);
bool kmbox_isdown_middle(void);
bool kmbox_isdown_side1(void);
bool kmbox_isdown_side2(void);

//--------------------------------------------------------------------+
// Utility Functions for Duplicate Code Consolidation
//--------------------------------------------------------------------+

// Map button number (0-5) to HID button bit mask (compatible with FAST_BTN_*)
// Returns 0x01 for button 0 or 1 (left), 0x02 for button 2 (right), etc.
uint8_t kmbox_map_button_to_hid_mask(uint8_t button_num);

// Clamp movement values to int8_t range (-127 to 127)
int8_t kmbox_clamp_movement_i8(int16_t value);

// Clamp wheel values to int8_t range (-128 to 127)  
int8_t kmbox_clamp_wheel_i8(int16_t value);

//--------------------------------------------------------------------+
// Command Parsing Utilities
//--------------------------------------------------------------------+

// Parse km.move(x,y) command, returns true if successful
// Sets x and y to parsed values
bool kmbox_parse_move_command(const char* cmd, int* x, int* y);

// Parse km.click(btn) command, returns true if successful  
// Sets button to parsed button number
bool kmbox_parse_click_command(const char* cmd, int* button);

// Check if command is km.version() - returns true if it matches
bool kmbox_is_version_command(const char* cmd);

#endif // KMBOX_COMMANDS_H