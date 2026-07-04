/*
 * Hurricane PIOKMBox Firmware
 */

#include "usb_hid.h"
#include "defines.h"
#include "led_control.h"
#include "lib/kmbox-commands/kmbox_commands.h"
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "hardware/timer.h"     // For time_us_32() in hid_device_task
#include "kmbox_serial_handler.h" // Include the header for serial handling
#include "state_management.h"   // Include the header for state management
#include "watchdog.h"           // Include the header for watchdog management
#include "smooth_injection.h"   // Include the header for smooth mouse injection
#include "humanization_fpu.h"   // Include for tremor generation
#include "xbox_gip.h"           // Xbox mode flag
#include "xbox_device.h"        // Xbox device descriptors
#include <string.h>             // For strcpy, strlen, memset
#include <math.h>                // For sqrtf, roundf
#include "pico/rand.h"           // For get_rand_32() hardware TRNG

uint16_t attached_vid = 0;
uint16_t attached_pid = 0;
bool attached_has_serial = false;
static volatile bool pending_device_reenumeration = false;
static bool checked_runtime_mouse_override = false;

// Dynamic string descriptor storage
static char attached_manufacturer[64] = "";
static char attached_product[64] = "";
static char attached_serial[32] = "";
static bool string_descriptors_fetched = false;

#define LANGUAGE_ID 0x0409  // English (US)

// UTF-16LE string descriptor -> UTF-8 (ASCII-only) conversion.
// Uses the descriptor's bLength to avoid reading past valid data.
static void utf16_to_utf8(uint16_t *utf16_buf, size_t utf16_buf_bytes, char *utf8_buf, size_t utf8_len)
{
    if (!utf16_buf || !utf8_buf || utf8_len == 0)
        return;

    // String descriptor format: [bLength (1B)][bDescriptorType (1B)][UTF-16LE code units...]
    // Determine actual descriptor length from first byte
    const uint8_t *raw = (const uint8_t *)utf16_buf;

    // Cap length by provided buffer size to be safe
    uint8_t bLength = raw[0];
    if (bLength > utf16_buf_bytes)
    {
        bLength = (uint8_t)utf16_buf_bytes;
    }

    // Compute number of 16-bit code units
    size_t code_units = 0;
    if (bLength >= 2)
    {
        code_units = (size_t)(bLength - 2) / 2;
    }

    size_t utf8_pos = 0;
    
    // Unroll loop by 4 for better performance (most strings are short)
    size_t i = 0;
    size_t utf16_buf_len = utf16_buf_bytes / 2;  // Total UTF-16 code units available
    while (i + 3 < code_units && utf8_pos + 3 < utf8_len - 1 && (1 + i + 3) < utf16_buf_len) {
        uint16_t u0 = utf16_buf[1 + i];
        uint16_t u1 = utf16_buf[1 + i + 1];
        uint16_t u2 = utf16_buf[1 + i + 2];
        uint16_t u3 = utf16_buf[1 + i + 3];
        
        // Early exit on NUL
        if (u0 == 0) break;
        utf8_buf[utf8_pos++] = (u0 <= 0x7F) ? (char)u0 : '?';
        
        if (u1 == 0) break;
        utf8_buf[utf8_pos++] = (u1 <= 0x7F) ? (char)u1 : '?';
        
        if (u2 == 0) break;
        utf8_buf[utf8_pos++] = (u2 <= 0x7F) ? (char)u2 : '?';
        
        if (u3 == 0) break;
        utf8_buf[utf8_pos++] = (u3 <= 0x7F) ? (char)u3 : '?';
        
        i += 4;
    }
    
    // Handle remaining code units
    for (; i < code_units && utf8_pos < utf8_len - 1; i++)
    {
        uint16_t u = utf16_buf[1 + i];
        if (u == 0) break;
        utf8_buf[utf8_pos++] = (u <= 0x7F) ? (char)u : '?';
    }

    utf8_buf[utf8_pos] = '\0';
}

// Function to set the VID and PID of the attached device
void set_attached_device_vid_pid(uint16_t vid, uint16_t pid) {
    // Only update and re-enumerate if VID/PID has actually changed
    if (attached_vid != vid || attached_pid != pid) {
        attached_vid = vid;
        attached_pid = pid;
        attached_has_serial = false; // Default to no serial number unless device has one
        
        // Force USB re-enumeration to update descriptor
        force_usb_reenumeration();
    }
}

void force_usb_reenumeration() {
    // Disconnect from USB host
    tud_disconnect();
    
    // Wait for host to recognize disconnection (500ms for Windows/macOS)
    // Feed watchdog during long wait to prevent reset
    for (int i = 0; i < 50; i++) {
        sleep_ms(10);
        watchdog_core0_heartbeat();
    }
    
    // Reconnect with new descriptor
    tud_connect();
    
    // Wait for reconnection (250ms for stability)
    // Feed watchdog during wait
    for (int i = 0; i < 25; i++) {
        sleep_ms(10);
        watchdog_core0_heartbeat();
    }
}

static inline void request_device_reenumeration(void) {
    __dmb();
    pending_device_reenumeration = true;
}

//--------------------------------------------------------------------+
// USB Descriptor Cloning Infrastructure
//--------------------------------------------------------------------+

// Forward declarations
static void build_runtime_hid_report_with_mouse(const uint8_t *mouse_desc, size_t mouse_len);
static void rebuild_configuration_descriptor(void);
static void parse_host_config_descriptor(const uint8_t *cfg_desc, uint16_t cfg_len);
static void reset_device_string_descriptors(void);  // Defined after all global state

// Captured host device descriptor fields for cloning
static struct {
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    bool     valid;
} host_device_info = { .bcdUSB = 0x0200, .bMaxPacketSize0 = 64, .bcdDevice = 0x0100, .valid = false };

// Captured host config descriptor fields for cloning
static struct {
    uint8_t  bmAttributes;     // Self-powered, remote wakeup
    uint8_t  bMaxPower;        // In 2mA units
    uint8_t  bInterfaceProtocol; // Boot mouse protocol etc
    uint8_t  bInterfaceSubClass;
    uint16_t wMaxPacketSize;   // Endpoint max packet size
    uint8_t  bInterval;        // Polling interval
    bool     valid;
} host_config_info = { .bmAttributes = TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, .bMaxPower = USB_CONFIG_POWER_MA / 2, .bInterfaceProtocol = HID_ITF_PROTOCOL_NONE, .bInterfaceSubClass = 0, .wMaxPacketSize = CFG_TUD_HID_EP_BUFSIZE, .bInterval = HID_POLLING_INTERVAL_MS, .valid = false };

// Runtime configuration descriptor buffer (large enough for multi-interface)
static uint8_t desc_configuration_runtime[DESC_CONFIG_RUNTIME_MAX];
static bool desc_config_runtime_valid = false;

//--------------------------------------------------------------------+
// Multi-interface mirroring infrastructure
//--------------------------------------------------------------------+

// Per-interface state captured from host device for faithful mirroring.
// Gaming mice expose 2-4 HID interfaces (mouse, keyboard-macros, vendor).
// We capture all of them and present matching interfaces on the device side.
typedef struct {
    // Interface properties (from host config descriptor)
    uint8_t  itf_subclass;
    uint8_t  itf_protocol;
    uint16_t ep_in_max_packet;
    uint8_t  ep_in_interval;
    bool     has_ep_out;
    uint16_t ep_out_max_packet;
    uint8_t  ep_out_interval;

    // Runtime state (populated during tuh_hid_mount_cb)
    uint8_t  host_dev_addr;
    uint8_t  host_instance;
    bool     is_mouse;         // Interface we inject mouse/keyboard/consumer into

    // HID report descriptor (non-mouse only; mouse uses desc_hid_report_runtime)
    uint8_t  report_desc[MIRROR_ITF_DESC_MAX];
    uint16_t report_desc_len;

    bool     active;
} mirrored_interface_t;

static mirrored_interface_t mirrored_itfs[MAX_DEVICE_HID_INTERFACES];
static uint8_t mirrored_itf_count = 0;      // Active mirrored interfaces
static uint8_t expected_hid_itf_count = 0;   // Expected from config descriptor
static uint8_t mounted_hid_itf_count = 0;    // Mounted so far

// Which device-side HID instance carries the composite descriptor (keyboard +
// mouse + consumer).  All mouse/keyboard/consumer reports must be sent on this
// instance.  Defaults to 0 for single-interface mode.
static uint8_t mouse_device_instance = 0;

// Vendor report passthrough queue (Core1 producer → Core0 consumer).
// When the host mouse sends vendor reports (e.g. Logitech HID++, Razer),
// Core1 queues them here and Core0 drains them via tud_hid_report().
#define VENDOR_QUEUE_SIZE 16
#define VENDOR_QUEUE_MASK (VENDOR_QUEUE_SIZE - 1)
#define VENDOR_REPORT_MAX_LEN 64
#define PASSTHROUGH_GET_TIMEOUT_US 80000u

typedef struct {
    uint8_t device_instance;  // Which device-side HID instance to send on
    uint8_t report_id;
    uint8_t data[VENDOR_REPORT_MAX_LEN];
    uint8_t len;
} vendor_report_entry_t;

static struct {
    vendor_report_entry_t entries[VENDOR_QUEUE_SIZE];
    volatile uint8_t head;   // Written by Core1 (producer)
    volatile uint8_t tail;   // Read by Core0 (consumer)
} vendor_fwd_queue;

// SET_REPORT passthrough queue (Core0 producer → Core1 consumer).
// When the downstream PC sends SET_REPORT to vendor interfaces, Core0
// queues them here and Core1 forwards to the real mouse.
typedef struct {
    uint8_t host_dev_addr;
    uint8_t host_instance;
    uint8_t report_id;
    uint8_t report_type;
    uint8_t data[VENDOR_REPORT_MAX_LEN];
    uint8_t len;
} set_report_entry_t;

static struct {
    set_report_entry_t entries[VENDOR_QUEUE_SIZE];
    volatile uint8_t head;   // Written by Core0 (producer)
    volatile uint8_t tail;   // Read by Core1 (consumer)
} set_report_queue;

// Extended string descriptor cache.
// Gaming mice may use string indices beyond the standard 1-3 (manufacturer,
// product, serial). Interface strings, HID class strings, etc.
#define MAX_CACHED_STRINGS 8
#define CACHED_STRING_MAX_LEN 64

typedef struct {
    uint8_t index;
    char    str[CACHED_STRING_MAX_LEN];
    bool    valid;
} cached_string_t;

static cached_string_t extra_strings[MAX_CACHED_STRINGS];
static uint8_t extra_string_count = 0;
static uint8_t max_string_index_seen = 3;  // Track highest string index from host

// GET_REPORT cache: stores the last received report per (instance, report_id)
// so that tud_hid_get_report_cb can respond to the host (macOS IOKit sends
// GET_REPORT during device open to verify responsiveness).
// Written by Core1 (in queue_vendor_report), read by Core0 (in get_report_cb).
#define REPORT_CACHE_SLOTS_PER_ITF 8

typedef struct {
    uint8_t report_id;
    uint8_t report_type;
    uint8_t data[VENDOR_REPORT_MAX_LEN];
    uint8_t len;
    bool    valid;
} cached_report_t;

static cached_report_t report_cache[MAX_DEVICE_HID_INTERFACES][REPORT_CACHE_SLOTS_PER_ITF];

typedef struct {
    volatile bool pending;
    volatile bool busy;
    volatile bool done;
    uint8_t host_dev_addr;
    uint8_t host_instance;
    uint8_t report_id;
    uint8_t report_type;
    uint16_t request_len;
    volatile uint16_t actual_len;
    uint8_t data[VENDOR_REPORT_MAX_LEN];
} get_report_bridge_t;

static get_report_bridge_t get_report_bridge;


// Function to fetch string descriptors from attached device
static void fetch_device_string_descriptors(uint8_t dev_addr) {
    // Reset string descriptors
    memset(attached_manufacturer, 0, sizeof(attached_manufacturer));
    memset(attached_product, 0, sizeof(attached_product));
    memset(attached_serial, 0, sizeof(attached_serial));
    string_descriptors_fetched = false;
    
    // Temporary buffers for UTF-16 strings
    uint16_t temp_manufacturer[32];
    uint16_t temp_product[48];
    uint16_t temp_serial[16];

    // Ensure buffers are zeroed to avoid stray data being interpreted
    memset(temp_manufacturer, 0, sizeof(temp_manufacturer));
    memset(temp_product, 0, sizeof(temp_product));
    memset(temp_serial, 0, sizeof(temp_serial));
    
    // Get manufacturer string
    if (tuh_descriptor_get_manufacturer_string_sync(dev_addr, LANGUAGE_ID, temp_manufacturer, sizeof(temp_manufacturer)) == XFER_RESULT_SUCCESS) {
        utf16_to_utf8(temp_manufacturer, sizeof(temp_manufacturer), attached_manufacturer, sizeof(attached_manufacturer));
        kmbox_send_status(attached_manufacturer);
    } else {
        strcpy(attached_manufacturer, MANUFACTURER_STRING);  // Fallback
    }
    
    // Get product string
    if (tuh_descriptor_get_product_string_sync(dev_addr, LANGUAGE_ID, temp_product, sizeof(temp_product)) == XFER_RESULT_SUCCESS) {
        utf16_to_utf8(temp_product, sizeof(temp_product), attached_product, sizeof(attached_product));
        kmbox_send_status(attached_product);
    } else {
        strcpy(attached_product, PRODUCT_STRING);  // Fallback
    }
    
    // Get serial string (optional)
    if (tuh_descriptor_get_serial_string_sync(dev_addr, LANGUAGE_ID, temp_serial, sizeof(temp_serial)) == XFER_RESULT_SUCCESS) {
        utf16_to_utf8(temp_serial, sizeof(temp_serial), attached_serial, sizeof(attached_serial));
        char serial_msg[64];
        snprintf(serial_msg, sizeof(serial_msg), "Serial: %s", attached_serial);
        kmbox_send_status(serial_msg);
        attached_has_serial = (strlen(attached_serial) > 0);
    } else {
        attached_has_serial = false;
    }
    
    string_descriptors_fetched = true;
}

// reset_device_string_descriptors() — defined after all global state declarations
// (see forward declaration above)

// Function to get the VID of the attached device
uint16_t get_attached_vid(void) {
    return attached_vid;
}

// Function to get the PID of the attached device
uint16_t get_attached_pid(void) {
    return attached_pid;
}

// Function to get attached device manufacturer string
const char* get_attached_manufacturer(void) {
    return attached_manufacturer;
}

// Function to get attached device product string
const char* get_attached_product(void) {
    return attached_product;
}

// Function to get dynamic serial string
const char* get_dynamic_serial_string() {
    static char dynamic_serial[64];
    if (attached_vid && attached_pid) {
        snprintf(dynamic_serial, sizeof(dynamic_serial), "PIOKMbox_%04X_%04X", attached_vid, attached_pid);
        return dynamic_serial;
    }
    return "PIOKMbox_v1.0";
}

// Error tracking structure with better organization
typedef struct
{
    uint32_t device_errors;
    uint32_t host_errors;
    uint32_t consecutive_device_errors;
    uint32_t consecutive_host_errors;
    uint32_t last_error_check_time;
    bool device_error_state;
    bool host_error_state;
} usb_error_tracker_t;

// Device connection state with better encapsulation
typedef struct
{
    bool mouse_connected;
    bool keyboard_connected;
    uint8_t mouse_dev_addr;
    uint8_t keyboard_dev_addr;
} device_connection_state_t;

// Device mode state
static bool caps_lock_state = false;

static device_connection_state_t connection_state = {0};

// Error tracking - now properly typed
static usb_error_tracker_t usb_error_tracker = {0};

// USB stack initialization tracking
static bool usb_device_initialized = false;
static bool usb_host_initialized = false;

// Track the last button byte sent to the host across ALL report paths
// (physical mouse forwarding, kmbox injection, smooth injection).
// Used by hid_device_task() to detect button-only changes that need
// an immediate report — a real mouse sends button changes on the very
// next poll.
static volatile uint8_t last_sent_buttons = 0;
// Track whether the previous cycle had activity, so we can send one
// final zero-delta "stop" report on the active→idle edge.
static volatile bool was_active = false;
// Set by Core1 after accumulating physical mouse data.
// Checked by Core0's hid_device_task() to bypass the 1ms timer
// and send immediately when the USB endpoint is ready.
static volatile bool fresh_mouse_data = false;

//--------------------------------------------------------------------+
// Output-stage PRNG (xorshift32, independent from smooth_injection.c)
//--------------------------------------------------------------------+
static uint32_t hid_rng_state = 0;

static void hid_rng_seed(uint32_t seed) {
    hid_rng_state = seed ? seed : 0xDEADBEEF;
}

static inline uint32_t hid_rng_next(void) {
    uint32_t x = hid_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    hid_rng_state = x;
    return x;
}

// Gaussian approximation via CLT: sum of 4 uniform [0,1) values → ~N(2,1/√3)
// Normalized to ~N(0,1) by subtracting 2 and scaling.
static inline float hid_rng_gaussian(void) {
    const float scale = 1.0f / 4294967296.0f;  // 1/(2^32)
    float sum = (float)hid_rng_next() * scale
              + (float)hid_rng_next() * scale
              + (float)hid_rng_next() * scale
              + (float)hid_rng_next() * scale;
    // sum ∈ [0,4), mean=2, stddev≈0.577; normalize to ~N(0,1)
    return (sum - 2.0f) * 1.7320508f;  // 1/0.577 ≈ √3
}

// Device management helpers
static void handle_device_disconnection(uint8_t dev_addr);
static void handle_hid_device_connection(uint8_t dev_addr, uint8_t itf_protocol);

// Report processing helpers
static bool process_keyboard_report_internal(const hid_keyboard_report_t *report);
static bool process_mouse_report_internal(const hid_mouse_report_t *report);

// Humanization helpers
static void apply_output_humanization(int16_t *x, int16_t *y, int16_t injected_x, int16_t injected_y);
static inline int8_t clamp_i8(int32_t val);

// Stochastic rounding: a value of 0.3 rounds to 1 with 30% probability, 0 with 70%.
// This preserves the statistical mean while making sub-pixel tremor actually visible
// in the output — deterministic roundf() kills any tremor < 0.5px.
static inline int16_t stochastic_round(float v) {
    if (v >= 0.0f) {
        int16_t floor_v = (int16_t)v;
        float frac = v - (float)floor_v;
        float r = (float)(hid_rng_next() & 0xFFFF) * (1.0f / 65536.0f);
        return floor_v + (r < frac ? 1 : 0);
    } else {
        float abs_v = -v;
        int16_t floor_v = (int16_t)abs_v;
        float frac = abs_v - (float)floor_v;
        float r = (float)(hid_rng_next() & 0xFFFF) * (1.0f / 65536.0f);
        return -(floor_v + (r < frac ? 1 : 0));
    }
}

// --- Runtime HID descriptor mirroring storage & helpers ---
// Gaming mice (Razer, Logitech, SteelSeries) can have very large HID
// report descriptors — 500-1000+ bytes with multiple collections for
// mouse, keyboard macros, and vendor-specific features.
#define HID_DESC_BUF_SIZE 1024

static const uint8_t desc_hid_keyboard[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD))};

// 16-bit mouse descriptor for gaming mice (G703, G Pro Wireless, etc.)
// Matches the G703 Lightspeed structure: 16-bit buttons, 16-bit X/Y, 8-bit wheel/pan
static const uint8_t desc_hid_mouse_16bit[] = {
    HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP      )                   ,
    HID_USAGE      ( HID_USAGE_DESKTOP_MOUSE     )                   ,
    HID_COLLECTION ( HID_COLLECTION_APPLICATION  )                   ,
      HID_REPORT_ID( REPORT_ID_MOUSE )
      HID_USAGE      ( HID_USAGE_DESKTOP_POINTER )                   ,
      HID_COLLECTION ( HID_COLLECTION_PHYSICAL   )                   ,
        // Buttons: 16 bits (to match high-end gaming mice)
        HID_USAGE_PAGE  ( HID_USAGE_PAGE_BUTTON  )                   ,
        HID_USAGE_MIN   ( 1                      )                   ,
        HID_USAGE_MAX   ( 16                     )                   ,
        HID_LOGICAL_MIN ( 0                      )                   ,
        HID_LOGICAL_MAX ( 1                      )                   ,
        HID_REPORT_COUNT( 16                     )                   ,
        HID_REPORT_SIZE ( 1                      )                   ,
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE )   ,
        // X, Y: 16-bit relative (-32767 to +32767)
        HID_USAGE_PAGE  ( HID_USAGE_PAGE_DESKTOP )                   ,
        HID_USAGE       ( HID_USAGE_DESKTOP_X    )                   ,
        HID_USAGE       ( HID_USAGE_DESKTOP_Y    )                   ,
        HID_LOGICAL_MIN_N ( -32767, 2            )                   ,
        HID_LOGICAL_MAX_N ( 32767, 2             )                   ,
        HID_REPORT_COUNT( 2                      )                   ,
        HID_REPORT_SIZE ( 16                     )                   ,
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE )   ,
        // Wheel: 8-bit relative
        HID_USAGE       ( HID_USAGE_DESKTOP_WHEEL )                  ,
        HID_LOGICAL_MIN ( 0x81                   )                   ,
        HID_LOGICAL_MAX ( 0x7F                   )                   ,
        HID_REPORT_COUNT( 1                      )                   ,
        HID_REPORT_SIZE ( 8                      )                   ,
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE )   ,
        // AC Pan: 8-bit relative (horizontal scroll)
        HID_USAGE_PAGE  ( HID_USAGE_PAGE_CONSUMER )                  ,
        HID_USAGE_N     ( HID_USAGE_CONSUMER_AC_PAN, 2 )             ,
        HID_LOGICAL_MIN ( 0x81                   )                   ,
        HID_LOGICAL_MAX ( 0x7F                   )                   ,
        HID_REPORT_COUNT( 1                      )                   ,
        HID_REPORT_SIZE ( 8                      )                   ,
        HID_INPUT       ( HID_DATA | HID_VARIABLE | HID_RELATIVE )   ,
      HID_COLLECTION_END                                             ,
    HID_COLLECTION_END
};

static const uint8_t desc_hid_consumer[] = {
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(REPORT_ID_CONSUMER_CONTROL))};

// Static fallback concatenated descriptor (used by config descriptor sizeof)
const uint8_t desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(REPORT_ID_MOUSE)),
    TUD_HID_REPORT_DESC_CONSUMER(HID_REPORT_ID(REPORT_ID_CONSUMER_CONTROL))};

static uint8_t desc_hid_report_runtime[HID_DESC_BUF_SIZE];
static size_t desc_hid_runtime_len = 0;
static bool desc_hid_runtime_valid = false;
static bool using_16bit_output_override = false;  // True when we override to 16-bit descriptor

static uint8_t host_mouse_desc[HID_DESC_BUF_SIZE];
static size_t host_mouse_desc_len = 0;
static bool host_mouse_has_report_id = false;
static uint8_t host_mouse_report_id = 0;

// --- Raw report forwarding for gaming mice ---
// When we clone the host mouse's HID report descriptor, we must send reports
// in the exact same binary format.  We parse the host descriptor to discover
// the byte layout (offsets & widths of buttons, X, Y, wheel) and then forward
// incoming reports with kmbox/smooth deltas injected in-place.
//
// Layout populated by parse_mouse_report_layout() during tuh_hid_mount_cb.

// Fast-path classification for forward_raw_mouse_report():
// Most gaming mice use byte-aligned 8-bit or 16-bit XY.  Classifying the
// layout at parse time lets the hot path skip complex bitwise extraction.
typedef enum {
    LAYOUT_GENERIC,     // Arbitrary bit-width / non-aligned — full extraction needed
    LAYOUT_FAST_8BIT,   // buttons[1] + X[i8] + Y[i8] + optional wheel/pan — all byte-aligned
    LAYOUT_FAST_16BIT,  // buttons[1-2] + X[i16 LE] + Y[i16 LE] — all byte-aligned
} layout_class_t;

typedef struct {
    // Total expected report size (excluding report-ID prefix byte)
    uint8_t  report_size;

    // Button byte
    uint8_t  buttons_offset;
    uint8_t  buttons_bits;     // typically 5 or 8

    // X axis
    uint8_t  x_offset;
    bool     x_is_16bit;
    uint16_t x_bit_offset;
    uint8_t  x_bits;
    uint8_t  x_start_byte;
    uint8_t  x_bit_in_byte;

    // Y axis
    uint8_t  y_offset;
    bool     y_is_16bit;
    uint16_t y_bit_offset;
    uint8_t  y_bits;
    uint8_t  y_start_byte;
    uint8_t  y_bit_in_byte;

    // Wheel (vertical scroll)
    uint8_t  wheel_offset;
    bool     has_wheel;

    // Horizontal scroll / pan
    uint8_t  pan_offset;
    bool     has_pan;

    // Report ID for the mouse collection (0 = no report IDs in descriptor)
    uint8_t  mouse_report_id;
    bool     has_report_id;

    bool     valid;            // true once successfully parsed
    layout_class_t layout_class; // fast-path classification (set by classify_layout)
} mouse_report_layout_t;

static mouse_report_layout_t host_mouse_layout = { .valid = false };

static mouse_report_layout_t output_mouse_layout_16bit = {
    .report_size = 8,
    .buttons_offset = 0,
    .buttons_bits = 16,
    .x_offset = 2,
    .x_is_16bit = true,
    .x_bit_offset = 16,
    .x_bits = 16,
    .x_start_byte = 2,
    .x_bit_in_byte = 0,
    .y_offset = 4,
    .y_is_16bit = true,
    .y_bit_offset = 32,
    .y_bits = 16,
    .y_start_byte = 4,
    .y_bit_in_byte = 0,
    .wheel_offset = 6,
    .has_wheel = true,
    .pan_offset = 7,
    .has_pan = true,
    .mouse_report_id = REPORT_ID_MOUSE,
    .has_report_id = true,
    .valid = true
};

// Track which dev_addr we've already cloned device/config descriptors for,
// so we only do it once for multi-interface composite devices (e.g. Razer
// Basilisk V3 has 4 HID interfaces, each triggers tuh_hid_mount_cb).
static uint8_t cloned_dev_addr = 0;

// Runtime report IDs — may be remapped to avoid conflicts with host mouse descriptor
static uint8_t runtime_kbd_report_id = REPORT_ID_KEYBOARD;
static uint8_t runtime_consumer_report_id = REPORT_ID_CONSUMER_CONTROL;

// Function to reset string descriptors and cloned state when device is disconnected
static void reset_device_string_descriptors(void) {
    memset(attached_manufacturer, 0, sizeof(attached_manufacturer));
    memset(attached_product, 0, sizeof(attached_product));
    memset(attached_serial, 0, sizeof(attached_serial));
    string_descriptors_fetched = false;
    attached_has_serial = false;

    // Reset cloned descriptor state
    host_device_info.valid = false;
    host_config_info.valid = false;
    host_mouse_layout.valid = false;
    host_mouse_desc_len = 0;
    host_mouse_has_report_id = false;
    host_mouse_report_id = 0;
    cloned_dev_addr = 0;

    // Reset multi-interface mirroring state
    mirrored_itf_count = 0;
    expected_hid_itf_count = 0;
    mounted_hid_itf_count = 0;
    mouse_device_instance = 0;
    memset(mirrored_itfs, 0, sizeof(mirrored_itfs));

    // Reset vendor report queues and GET_REPORT cache
    vendor_fwd_queue.head = vendor_fwd_queue.tail = 0;
    set_report_queue.head = set_report_queue.tail = 0;
    memset(report_cache, 0, sizeof(report_cache));
    memset(&get_report_bridge, 0, sizeof(get_report_bridge));

    // Reset extra string descriptor cache
    extra_string_count = 0;
    max_string_index_seen = 3;
    memset(extra_strings, 0, sizeof(extra_strings));

    // Reset runtime report IDs to defaults
    runtime_kbd_report_id = REPORT_ID_KEYBOARD;
    runtime_consumer_report_id = REPORT_ID_CONSUMER_CONTROL;

    // Rebuild config descriptor with defaults
    build_runtime_hid_report_with_mouse(NULL, 0);
    rebuild_configuration_descriptor();
}

//--------------------------------------------------------------------+
// Inline Helper Functions
//--------------------------------------------------------------------+

static __force_inline int8_t clamp_i8(int32_t val) {
    if (val > 127) return 127;
    if (val < -128) return -128;
    return (int8_t)val;
}

//--------------------------------------------------------------------+
// Final Stage Humanization (Applied proportionally to injected movement)
//--------------------------------------------------------------------+

/**
 * Apply humanization tremor to final HID output movement.
 * 
 * KEY DESIGN: Human mouse movement is already human — it doesn't need
 * additional tremor/jitter. Tremor is only applied proportionally to the
 * synthetic (injected) fraction of the total movement. This keeps the
 * mouse feeling natural and responsive during normal use while still
 * humanizing injected/bot movement.
 * 
 * Blend logic:
 *   - Pure physical movement (injected == 0): no tremor applied
 *   - Mixed (physical + injected): tremor scaled by injected fraction
 *   - Pure injected (no physical): full tremor applied
 * 
 * @param x Pointer to total X movement (modified in place)
 * @param y Pointer to total Y movement (modified in place)
 * @param injected_x The injected/synthetic X component (smooth queue + kmbox serial)
 * @param injected_y The injected/synthetic Y component (smooth queue + kmbox serial)
 */
static void apply_output_humanization(int16_t *x, int16_t *y, int16_t injected_x, int16_t injected_y) {
    // Skip if humanization is completely disabled
    humanization_mode_t mode = smooth_get_humanization_mode();
    if (mode == HUMANIZATION_OFF) {
        return;
    }
    
    // Get humanization parameters from smooth injection state
    int32_t jitter_amount_fp;
    bool jitter_enabled;
    smooth_get_humanization_params(&jitter_amount_fp, &jitter_enabled);
    
    if (!jitter_enabled) {
        return;
    }
    
    // --- Calculate blend ratio: how much of this movement is synthetic? ---
    // If there's no injected component, the user is just moving their mouse.
    // Human movement is already human — don't add tremor to it.
    float inject_mag = sqrtf((float)injected_x * injected_x + (float)injected_y * injected_y);
    
    // No injected movement — nothing to humanize. Do NOT apply tremor.
    // This prevents phantom tremor after injection queue drains (the "shaking" bug).
    if (inject_mag < 0.5f) {
        return;
    }
    
    float total_mag  = sqrtf((float)(*x) * (*x) + (float)(*y) * (*y));
    
    // No movement at all — skip (don't generate idle tremor on pure physical idle)
    if (total_mag < 0.5f && inject_mag < 0.5f) {
        return;
    }
    
    // Blend ratio: 0.0 = pure physical, 1.0 = pure injected
    float blend;
    if (total_mag < 0.5f) {
        // Total is ~zero but inject is non-zero (rare edge: physical cancelled inject)
        blend = 1.0f;
    } else {
        blend = inject_mag / total_mag;
        // Clamp to [0, 1] — inject_mag can exceed total_mag if they oppose
        if (blend > 1.0f) blend = 1.0f;
    }
    
    // If movement is almost entirely physical, skip tremor entirely
    // This threshold avoids wasting FPU cycles for negligible tremor
    if (blend < 0.05f) {
        return;
    }
    
    // --- Calculate tremor magnitude ---
    float magnitude = total_mag;
    
    // Mode-dependent intensity scaling
    // MICRO was 0.5x which, combined with 0.5px base jitter, gave only 0.25px
    // effective tremor — too small to survive even stochastic rounding consistently.
    float mode_scale = 1.0f;
    switch (mode) {
        case HUMANIZATION_MICRO:
            mode_scale = 0.75f;
            break;
        case HUMANIZATION_FULL:
            mode_scale = 1.0f;
            break;
        default:
            break;
    }
    
    // Calculate tremor scale with blend factor
    float movement_scale = humanization_jitter_scale(magnitude);
    float base_jitter = (float)jitter_amount_fp / 65536.0f;  // Convert from 16.16 fixed-point
    float tremor_scale = base_jitter * movement_scale * mode_scale * blend;

    // Scale noise DOWN at low velocities to prevent overwhelming the signal.
    // At 1-2 count deltas, ±1 of tremor is 50-100% perturbation, creating
    // chaotic scribble instead of smooth slow transitions.
    // Ramp: 0 at 0px → full at 4px
    float low_speed_scale = fminf(1.0f, magnitude / 4.0f);
    tremor_scale *= low_speed_scale;

    // Get runtime tremor (layered oscillators + noise)
    float tremor_x, tremor_y;
    humanization_get_tremor(tremor_scale, &tremor_x, &tremor_y);
    
    if (magnitude > 2.0f) {
        // Moving cursor: perpendicular + parallel decomposition
        float fx = (float)(*x);
        float fy = (float)(*y);
        float norm_x = fx / magnitude;
        float norm_y = fy / magnitude;
        
        // Perpendicular component (tremor_y) - primary humanization signal
        float perp_dx = -norm_y * tremor_y;
        float perp_dy = norm_x * tremor_y;
        
        // Parallel component (tremor_x) - speed variation (smaller)
        float para_dx = norm_x * tremor_x * 0.3f;
        float para_dy = norm_y * tremor_x * 0.3f;
        
        // Apply tremor to output (stochastic rounding so sub-pixel tremor
        // probabilistically produces visible ±1 counts instead of always 0)
        *x = (int16_t)(*x + stochastic_round(perp_dx + para_dx));
        *y = (int16_t)(*y + stochastic_round(perp_dy + para_dy));
    } else {
        // Small/idle movement with injection active: apply tremor as raw X/Y
        *x += stochastic_round(tremor_x);
        *y += stochastic_round(tremor_y);
    }
}

// Sensor noise: gaussian-based quantization noise with stochastic rounding.
// Real optical sensors have continuous noise that maps to discrete count
// perturbations. Using a continuous gaussian source ensures consecutive
// identical underlying deltas get DIFFERENT noise samples, reducing repeats.
// Always-±1 binary noise has P(match)=50% — worse than {-1,0,+1} was.
static inline int16_t apply_sensor_noise(int16_t value) {
    if (value == 0) return 0;  // stationary sensor produces no noise
    // Gaussian with stddev ~0.7 → mostly ±1, sometimes ±2 or 0.
    // Continuous source means consecutive values are decorrelated.
    float noise = hid_rng_gaussian() * 0.7f;
    return value + stochastic_round(noise);
}

// Minimal HID descriptor parser — extracts report field layout for the mouse
// collection so we know where to inject deltas in raw reports.
// This is intentionally simple and handles the common gaming mouse patterns:
//   buttons (1-3 bytes), X (8/12/16 bit), Y (8/12/16 bit), wheel, pan.
static void parse_mouse_report_layout(const uint8_t *desc, size_t len,
                                       mouse_report_layout_t *layout)
{
    memset(layout, 0, sizeof(*layout));
    layout->valid = false;

    if (!desc || len < 4) return;

    // HID descriptor state machine — minimal implementation
    bool in_mouse_collection = false;
    uint16_t usage_page = 0;  // 16-bit: vendor pages (0xFF00) truncate in uint8_t
    uint8_t usage = 0;
    uint32_t bit_offset = 0;        // current bit position in the report
    uint32_t mouse_bit_max = 0;     // track highest bit offset within mouse collection
    uint8_t report_size_bits = 0;   // current REPORT_SIZE
    uint8_t report_count = 0;       // current REPORT_COUNT
    int collection_depth = 0;
    int mouse_collection_depth = -1;
    uint8_t current_report_id = 0;  // current Report ID context
    uint8_t mouse_report_id = 0;    // Report ID that contains the mouse collection
    bool found_mouse_report_id = false;

    // Track what usages we've seen before each INPUT item
    #define MAX_USAGES 16
    uint8_t usage_stack[MAX_USAGES];
    uint8_t usage_stack_count = 0;
    bool has_usage_range = false;
    uint8_t usage_min = 0;

    size_t i = 0;
    while (i < len) {
        uint8_t item = desc[i];
        uint8_t item_size = item & 0x03;
        if (item_size == 3) item_size = 4; // size=3 means 4 bytes

        if (i + 1 + item_size > len) break;

        uint32_t value = 0;
        for (uint8_t b = 0; b < item_size; b++) {
            value |= (uint32_t)desc[i + 1 + b] << (b * 8);
        }

        uint8_t item_tag = item & 0xFC; // tag + type

        switch (item_tag) {
            case 0x04: // Usage Page (Global)
                usage_page = (uint16_t)value;
                break;
            case 0x08: // Usage (Local)
                if (usage_stack_count < MAX_USAGES) {
                    usage_stack[usage_stack_count++] = (uint8_t)value;
                }
                usage = (uint8_t)value;
                break;
            case 0x18: // Usage Minimum (Local)
                has_usage_range = true;
                usage_min = (uint8_t)value;
                break;
            case 0x28: // Usage Maximum (Local)
                break;
            case 0xA0: // Collection
                collection_depth++;
                if (usage_page == HID_USAGE_PAGE_DESKTOP && usage == HID_USAGE_DESKTOP_MOUSE) {
                    in_mouse_collection = true;
                    mouse_collection_depth = collection_depth;
                    // Record which Report ID contains the mouse collection
                    mouse_report_id = current_report_id;
                    found_mouse_report_id = (current_report_id != 0);
                }
                break;
            case 0xC0: // End Collection
                if (in_mouse_collection && collection_depth == mouse_collection_depth) {
                    // Exiting the mouse collection — record max bit offset for size calc
                    if (bit_offset > mouse_bit_max) {
                        mouse_bit_max = bit_offset;
                    }
                    in_mouse_collection = false;
                    mouse_collection_depth = -1;
                }
                collection_depth--;
                break;
            case 0x74: // Report Size (Global)
                report_size_bits = (uint8_t)value;
                break;
            case 0x94: // Report Count (Global)
                report_count = (uint8_t)value;
                break;
            case 0x80: // Input (Main)
            {
                if (in_mouse_collection) {
                    bool is_constant = (value & 0x01); // bit 0: constant vs data
                    uint32_t total_bits = (uint32_t)report_size_bits * report_count;

                    if (!is_constant) {
                        // Determine what this input field is based on usage context
                        bool is_desktop_range = (has_usage_range && usage_page == HID_USAGE_PAGE_DESKTOP);
                        
                        if (usage_page == HID_USAGE_PAGE_BUTTON || (has_usage_range && !is_desktop_range)) {
                            // Button field
                            layout->buttons_offset = bit_offset / 8;
                            layout->buttons_bits = report_count;
                        } else if (usage_page == HID_USAGE_PAGE_DESKTOP) {
                            // Process usages (explicit stack or range-based)
                            for (uint8_t u = 0; u < report_count; u++) {
                                uint8_t cur_usage = 0;
                                
                                if (has_usage_range) {
                                    cur_usage = usage_min + u;
                                } else if (u < usage_stack_count) {
                                    cur_usage = usage_stack[u];
                                } else {
                                    // No more usages in stack vs report count
                                    break;
                                }

                                uint32_t field_bit_offset = bit_offset + (u * report_size_bits);
                                uint8_t byte_off = field_bit_offset / 8;

                                if (cur_usage == HID_USAGE_DESKTOP_X) {
                                    layout->x_offset = byte_off;
                                    layout->x_is_16bit = (report_size_bits >= 16);
                                    layout->x_bit_offset = (uint16_t)field_bit_offset;
                                    layout->x_bits = report_size_bits;
                                    layout->x_start_byte = byte_off;
                                    layout->x_bit_in_byte = field_bit_offset % 8;
                                } else if (cur_usage == HID_USAGE_DESKTOP_Y) {
                                    layout->y_offset = byte_off;
                                    layout->y_is_16bit = (report_size_bits >= 16);
                                    layout->y_bit_offset = (uint16_t)field_bit_offset;
                                    layout->y_bits = report_size_bits;
                                    layout->y_start_byte = byte_off;
                                    layout->y_bit_in_byte = field_bit_offset % 8;
                                } else if (cur_usage == HID_USAGE_DESKTOP_WHEEL) {
                                    layout->wheel_offset = byte_off;
                                    layout->has_wheel = true;
                                }
                            }
                            
                            if (!has_usage_range && usage_stack_count <= 1 && usage == HID_USAGE_DESKTOP_X && report_count >= 2) {
                                uint8_t x_byte_off = bit_offset / 8;
                                uint8_t y_byte_off = (bit_offset + report_size_bits) / 8;
                                layout->x_offset = x_byte_off;
                                layout->x_is_16bit = (report_size_bits >= 16);
                                layout->x_bit_offset = (uint16_t)bit_offset;
                                layout->x_bits = report_size_bits;
                                layout->x_start_byte = x_byte_off;
                                layout->x_bit_in_byte = bit_offset % 8;
                                layout->y_offset = y_byte_off;
                                layout->y_is_16bit = (report_size_bits >= 16);
                                layout->y_bit_offset = (uint16_t)(bit_offset + report_size_bits);
                                layout->y_bits = report_size_bits;
                                layout->y_start_byte = y_byte_off;
                                layout->y_bit_in_byte = (bit_offset + report_size_bits) % 8;
                            }
                        } else if (usage_page == HID_USAGE_PAGE_CONSUMER) {
                            // AC Pan (horizontal scroll)
                            if (usage_stack_count > 0 && usage_stack[0] == 0x38) { // AC Pan = 0x238
                                layout->pan_offset = bit_offset / 8;
                                layout->has_pan = true;
                            }
                        }
                    }

                    bit_offset += total_bits;
                }

                // Clear local state after Main item
                usage_stack_count = 0;
                has_usage_range = false;
                break;
            }
            case 0x84: // Report ID (Global)
                // Record the current Report ID context before processing
                // If we were in the mouse collection, save its final bit offset
                if (in_mouse_collection && bit_offset > mouse_bit_max) {
                    mouse_bit_max = bit_offset;
                }
                current_report_id = (uint8_t)value;
                // If we're already inside the mouse collection, update the
                // mouse report ID.  Some receivers (e.g. Logitech LIGHTSPEED)
                // place the Report ID *after* the Collection(Application) tag
                // for the mouse, so the initial value captured at collection
                // open time may be stale (e.g. keyboard's Report ID 1).
                if (in_mouse_collection) {
                    mouse_report_id = current_report_id;
                    found_mouse_report_id = (current_report_id != 0);
                }
                // Reset bit offset for new report ID section
                // (report ID byte is NOT counted in the bit offset of field data)
                bit_offset = 0;
                break;
        }

        i += 1 + item_size;
    }

    // Use mouse-specific bit count for report size, not the final global bit_offset
    // which may include data from non-mouse report IDs after the mouse collection
    if (in_mouse_collection && bit_offset > mouse_bit_max) {
        mouse_bit_max = bit_offset; // Still in mouse collection at end of descriptor
    }
    layout->report_size = (mouse_bit_max + 7) / 8;
    
    // Store the discovered report ID
    layout->mouse_report_id = mouse_report_id;
    layout->has_report_id = found_mouse_report_id;
    
    // A valid mouse report MUST have at least X and Y axes defined.
    // Otherwise we risk using a broken layout (offsets=0) and reading garbage.
    if (layout->report_size < 3 || layout->x_bits == 0 || layout->y_bits == 0) {
        // Invalid or incomplete layout - better to fallback to legacy mode
        layout->valid = false;
        return;
    }
    layout->valid = true;
}

// Per-instance HID usage tracking for non-boot-protocol devices (e.g. Logitech receivers)
// These devices report HID_ITF_PROTOCOL_NONE but we can detect mouse/keyboard
// usage from parsing the HID report descriptor.
#define MAX_HID_INSTANCES 16  // CFG_TUH_HID * CFG_TUH_DEVICE_MAX
typedef struct {
    uint8_t dev_addr;
    uint8_t instance;
    uint8_t effective_protocol;  // HID_ITF_PROTOCOL_MOUSE/KEYBOARD/NONE
    bool    has_report_id;
    uint8_t mouse_report_id;     // Report ID for mouse reports (0 if no report IDs)
    uint8_t keyboard_report_id;  // Report ID for keyboard reports
    bool    active;
} hid_instance_info_t;

static hid_instance_info_t hid_instances[MAX_HID_INSTANCES];

static hid_instance_info_t* find_hid_instance(uint8_t dev_addr, uint8_t instance) {
    for (int i = 0; i < MAX_HID_INSTANCES; i++) {
        if (hid_instances[i].active && 
            hid_instances[i].dev_addr == dev_addr && 
            hid_instances[i].instance == instance) {
            return &hid_instances[i];
        }
    }
    return NULL;
}

static hid_instance_info_t* alloc_hid_instance(uint8_t dev_addr, uint8_t instance) {
    // First check if already exists
    hid_instance_info_t* existing = find_hid_instance(dev_addr, instance);
    if (existing) return existing;
    
    // Find empty slot
    for (int i = 0; i < MAX_HID_INSTANCES; i++) {
        if (!hid_instances[i].active) {
            memset(&hid_instances[i], 0, sizeof(hid_instance_info_t));
            hid_instances[i].dev_addr = dev_addr;
            hid_instances[i].instance = instance;
            hid_instances[i].active = true;
            return &hid_instances[i];
        }
    }
    return NULL;
}

static void free_hid_instances_for_device(uint8_t dev_addr) {
    for (int i = 0; i < MAX_HID_INSTANCES; i++) {
        if (hid_instances[i].active && hid_instances[i].dev_addr == dev_addr) {
            hid_instances[i].active = false;
        }
    }
}

// Detect mouse/keyboard usage from HID report descriptor
// Returns effective protocol: HID_ITF_PROTOCOL_MOUSE, HID_ITF_PROTOCOL_KEYBOARD, or HID_ITF_PROTOCOL_NONE
static uint8_t detect_usage_from_report_descriptor(const uint8_t *desc_report, uint16_t desc_len,
                                                    hid_instance_info_t *info) {
    if (!desc_report || desc_len == 0) return HID_ITF_PROTOCOL_NONE;
    
    // Use TinyUSB's built-in parser
    tuh_hid_report_info_t report_info[8];
    uint8_t report_count = tuh_hid_parse_report_descriptor(report_info, 8, desc_report, desc_len);
    
    uint8_t detected_protocol = HID_ITF_PROTOCOL_NONE;
    
    for (uint8_t i = 0; i < report_count; i++) {
        if (report_info[i].usage_page == HID_USAGE_PAGE_DESKTOP) {
            if (report_info[i].usage == HID_USAGE_DESKTOP_MOUSE) {
                detected_protocol = HID_ITF_PROTOCOL_MOUSE;
                if (info) {
                    info->has_report_id = (report_info[i].report_id != 0);
                    info->mouse_report_id = report_info[i].report_id;
                }
                // Mouse usage detected
                break;  // Mouse takes priority
            } else if (report_info[i].usage == HID_USAGE_DESKTOP_KEYBOARD) {
                detected_protocol = HID_ITF_PROTOCOL_KEYBOARD;
                if (info) {
                    info->has_report_id = (report_info[i].report_id != 0);
                    info->keyboard_report_id = report_info[i].report_id;
                }
                // Keyboard usage detected
                // Don't break - keep looking for mouse
            }
        }
    }
    
    return detected_protocol;
}

// Strip vendor-specific HID collections (Logitech HID++ Report IDs 0x10/0x11,
// Usage Page >= 0xFF00) from a cloned HID report descriptor.  Including these
// in the proxy output descriptor causes host drivers to install vendor filters
// (e.g. Logitech HID++ driver) that fight the proxy for the device.
//
// Walks the descriptor item-by-item.  When entering a top-level Collection
// under a vendor Usage Page, skips all items until the matching End Collection.
static size_t strip_vendor_collections(const uint8_t *src, size_t src_len,
                                        uint8_t *dst, size_t dst_max)
{
    if (!src || src_len == 0 || !dst || dst_max == 0) return 0;

    size_t out = 0;
    size_t i = 0;
    uint16_t cur_usage_page = 0;
    int skip_depth = 0;

    while (i < src_len) {
        uint8_t item = src[i];
        uint8_t item_size = item & 0x03;
        if (item_size == 3) item_size = 4;
        uint8_t total_len = 1 + item_size;
        if (i + total_len > src_len) break;

        uint32_t value = 0;
        for (uint8_t b = 0; b < item_size; b++)
            value |= (uint32_t)src[i + 1 + b] << (b * 8);

        uint8_t item_tag = item & 0xFC;

        if (skip_depth > 0) {
            if (item_tag == 0xA0) skip_depth++;
            else if (item_tag == 0xC0) skip_depth--;
            i += total_len;
            continue;
        }

        if (item_tag == 0x04) // Usage Page (Global)
            cur_usage_page = (uint16_t)value;

        if (item_tag == 0xA0 && cur_usage_page >= 0xFF00) {
            // Entering vendor collection — skip it entirely
            skip_depth = 1;
            i += total_len;
            continue;
        }

        if (out + total_len <= dst_max) {
            memcpy(&dst[out], &src[i], total_len);
            out += total_len;
        } else {
            break;
        }
        i += total_len;
    }
    return out;
}

static void build_runtime_hid_report_with_mouse(const uint8_t *mouse_desc, size_t mouse_len)
{
    // Build the composite HID report descriptor:
    //   Keyboard (report ID N) + Mouse (report ID M) + Consumer Control (report ID P)
    //
    // CRITICAL CONSTRAINT (USB HID §5.6): When ANY top-level collection uses a
    // Report ID, ALL collections in the descriptor MUST have a Report ID.
    // Our keyboard and consumer descriptors always have Report IDs, so we MUST
    // ensure the mouse portion also has one.
    //
    // For gaming mice whose descriptors already contain Report IDs, we use the
    // descriptor as-is and remap keyboard/consumer to avoid conflicts.
    //
    // For boot-protocol mice (or mice whose descriptors lack Report IDs), we
    // inject REPORT_ID_MOUSE (2) into the mouse descriptor so it's valid in the
    // composite.
    
    // --- Scan mouse descriptor for existing Report IDs ---
    bool used_ids[256];
    memset(used_ids, 0, sizeof(used_ids));
    bool mouse_has_report_ids = false;
    
    if (mouse_desc != NULL && mouse_len > 0) {
        for (size_t i = 0; i + 1 < mouse_len; i++) {
            if (mouse_desc[i] == 0x85) { // Report ID tag (1-byte value)
                used_ids[mouse_desc[i + 1]] = true;
                mouse_has_report_ids = true;
            }
        }
    }
    
    // --- If mouse has no Report IDs, we'll inject REPORT_ID_MOUSE ---
    // Build a modified copy with Report ID inserted after the first
    // Collection(Application) tag [0xA1, 0x01].
    uint8_t mouse_desc_patched[HID_DESC_BUF_SIZE];
    size_t mouse_len_patched = 0;
    const uint8_t *mouse_desc_final = mouse_desc;
    size_t mouse_len_final = mouse_len;
    
    if (mouse_desc != NULL && mouse_len > 0 && !mouse_has_report_ids) {
        // Inject Report ID after first Collection(Application)
        bool injected = false;
        size_t dst = 0;
        for (size_t src = 0; src < mouse_len && dst < sizeof(mouse_desc_patched) - 2; src++) {
            mouse_desc_patched[dst++] = mouse_desc[src];
            
            // Look for Collection(Application): 0xA1 0x01
            if (!injected && src + 1 < mouse_len &&
                mouse_desc[src] == 0xA1 && mouse_desc[src + 1] == 0x01) {
                // Copy the 0x01 value byte
                src++;
                mouse_desc_patched[dst++] = mouse_desc[src];
                // Inject Report ID(REPORT_ID_MOUSE)
                mouse_desc_patched[dst++] = 0x85; // Report ID tag
                mouse_desc_patched[dst++] = REPORT_ID_MOUSE;
                injected = true;
            }
        }
        
        if (injected) {
            mouse_len_patched = dst;
            mouse_desc_final = mouse_desc_patched;
            mouse_len_final = mouse_len_patched;
            used_ids[REPORT_ID_MOUSE] = true;
        }
    }
    
    // --- Determine safe Report IDs for keyboard and consumer ---
    uint8_t kbd_report_id = REPORT_ID_KEYBOARD;      // default 1
    uint8_t consumer_report_id = REPORT_ID_CONSUMER_CONTROL; // default 3
    
    // If default keyboard ID conflicts, find an unused one starting from 0x10
    if (used_ids[kbd_report_id]) {
        for (uint8_t id = 0x10; id < 0xFE; id++) {
            if (!used_ids[id]) { kbd_report_id = id; break; }
        }
    }
    used_ids[kbd_report_id] = true; // Mark it used
    
    // Same for consumer
    if (used_ids[consumer_report_id]) {
        for (uint8_t id = 0x11; id < 0xFE; id++) {
            if (!used_ids[id]) { consumer_report_id = id; break; }
        }
    }
    
    // Build remapped keyboard and consumer descriptors
    uint8_t kbd_desc[sizeof(desc_hid_keyboard)];
    memcpy(kbd_desc, desc_hid_keyboard, sizeof(desc_hid_keyboard));
    for (size_t i = 0; i + 1 < sizeof(kbd_desc); i++) {
        if (kbd_desc[i] == 0x85 && kbd_desc[i+1] == REPORT_ID_KEYBOARD) {
            kbd_desc[i+1] = kbd_report_id;
            break;
        }
    }
    
    uint8_t consumer_desc[sizeof(desc_hid_consumer)];
    memcpy(consumer_desc, desc_hid_consumer, sizeof(desc_hid_consumer));
    for (size_t i = 0; i + 1 < sizeof(consumer_desc); i++) {
        if (consumer_desc[i] == 0x85 && consumer_desc[i+1] == REPORT_ID_CONSUMER_CONTROL) {
            consumer_desc[i+1] = consumer_report_id;
            break;
        }
    }
    
    size_t pos = 0;

    // Copy keyboard descriptor
    if (pos + sizeof(kbd_desc) >= HID_DESC_BUF_SIZE)
        return;
    memcpy(&desc_hid_report_runtime[pos], kbd_desc, sizeof(kbd_desc));
    pos += sizeof(kbd_desc);

    // Copy mouse descriptor (possibly patched with injected Report ID)
    if (mouse_desc_final != NULL && mouse_len_final > 0)
    {
        // CRITICAL FIX: For 16-bit gaming mice (G703, G Pro Wireless, etc.),
        // macOS may expose an 8-bit HID descriptor even though the mouse reports
        // 16-bit X/Y values. If we clone the 8-bit descriptor, the 16-bit reports
        // get truncated. Solution: If the parsed layout shows 16-bit X/Y fields,
        // override the cloned descriptor with our own 16-bit descriptor.
        bool use_16bit_override = (host_mouse_layout.valid && 
                                   host_mouse_layout.x_bits == 16 && 
                                   host_mouse_layout.y_bits == 16);
        
        if (use_16bit_override) {
            // Use our 16-bit mouse descriptor instead of the cloned 8-bit one
            size_t dlen = sizeof(desc_hid_mouse_16bit);
            if (pos + dlen >= HID_DESC_BUF_SIZE)
                return;
            memcpy(&desc_hid_report_runtime[pos], desc_hid_mouse_16bit, dlen);
            pos += dlen;
            using_16bit_output_override = true;
        } else {
            // Use the cloned descriptor from the host mouse
            if (pos + mouse_len_final >= HID_DESC_BUF_SIZE)
                return;
            memcpy(&desc_hid_report_runtime[pos], mouse_desc_final, mouse_len_final);
            pos += mouse_len_final;
            using_16bit_output_override = false;
        }
    }
    else
    {
        // Fallback: Use 16-bit mouse descriptor for Lightspeed and other high-res mice
        // This path is triggered when host_mouse_desc_len == 0 (e.g., Logitech Lightspeed)
        size_t dlen = sizeof(desc_hid_mouse_16bit);
        if (pos + dlen >= HID_DESC_BUF_SIZE)
            return;
        memcpy(&desc_hid_report_runtime[pos], desc_hid_mouse_16bit, dlen);
        pos += dlen;
        using_16bit_output_override = true;
    }

    // Copy consumer control descriptor
    if (pos + sizeof(consumer_desc) >= HID_DESC_BUF_SIZE)
        return;
    memcpy(&desc_hid_report_runtime[pos], consumer_desc, sizeof(consumer_desc));
    pos += sizeof(consumer_desc);

    // Zero-fill remainder
    if (pos < HID_DESC_BUF_SIZE) {
        memset(&desc_hid_report_runtime[pos], 0, HID_DESC_BUF_SIZE - pos);
    }

    // Set actual content length so host reads only valid descriptor data
    desc_hid_runtime_len = pos;
    desc_hid_runtime_valid = true;
    
    // Store the runtime IDs for use by all report-sending paths
    runtime_kbd_report_id = kbd_report_id;
    runtime_consumer_report_id = consumer_report_id;
}

bool usb_hid_init(void)
{
    gpio_init(PIN_BUTTON);

    gpio_set_dir(PIN_BUTTON, GPIO_IN);
    gpio_pull_up(PIN_BUTTON);

    // Initialize connection state
    memset(&connection_state, 0, sizeof(connection_state));

    // Initialize per-instance HID tracking
    memset(hid_instances, 0, sizeof(hid_instances));

    // Initialize multi-interface mirroring state
    memset(mirrored_itfs, 0, sizeof(mirrored_itfs));
    mirrored_itf_count = 0;
    expected_hid_itf_count = 0;
    mounted_hid_itf_count = 0;
    mouse_device_instance = 0;
    vendor_fwd_queue.head = vendor_fwd_queue.tail = 0;
    set_report_queue.head = set_report_queue.tail = 0;
    memset(report_cache, 0, sizeof(report_cache));
    memset(&get_report_bridge, 0, sizeof(get_report_bridge));
    extra_string_count = 0;

    // Seed output-stage PRNG from hardware TRNG
    hid_rng_seed(get_rand_32());

    // Build default runtime HID report descriptor (keyboard + default mouse + consumer)
    build_runtime_hid_report_with_mouse(NULL, 0);

    // Build initial runtime config descriptor with defaults
    rebuild_configuration_descriptor();

    (void)0; // suppressed init log
    return true;
}



bool usb_host_enable_power(void)
{
    #ifdef PIN_USB_5V
    gpio_put(PIN_USB_5V, 1); // Enable USB power
    #endif
    sleep_ms(100);           // Allow power to stabilize
    return true;
}

void usb_device_mark_initialized(void)
{
    usb_device_initialized = true;
}

void usb_host_mark_initialized(void)
{
    usb_host_initialized = true;
}

//--------------------------------------------------------------------+
// IMPROVED STATE MANAGEMENT
//--------------------------------------------------------------------+

bool get_caps_lock_state(void)
{
    return caps_lock_state;
}

bool is_mouse_connected(void)
{
    return connection_state.mouse_connected;
}

bool is_keyboard_connected(void)
{
    return connection_state.keyboard_connected;
}

static void handle_device_disconnection(uint8_t dev_addr)
{
    // Only reset connection flags for the specific device that was disconnected
    if (dev_addr == connection_state.mouse_dev_addr)
    {
        connection_state.mouse_connected = false;
        connection_state.mouse_dev_addr = 0;
    }

    if (dev_addr == connection_state.keyboard_dev_addr)
    {
        connection_state.keyboard_connected = false;
        connection_state.keyboard_dev_addr = 0;
    }
}

static void handle_hid_device_connection(uint8_t dev_addr, uint8_t itf_protocol)
{
    // Validate input parameters
    if (dev_addr == 0)
    {
        // Invalid device address - avoid heavy logging in hot path
        return;
    }

    // Track connected device types and store device addresses.
    // Use LED activity to indicate connection instead of console logging.
    switch (itf_protocol)
    {
    case HID_ITF_PROTOCOL_MOUSE:
        connection_state.mouse_connected = true;
        connection_state.mouse_dev_addr = dev_addr;
        neopixel_trigger_activity_flash_color(COLOR_MOUSE_ACTIVITY); // Flash magenta for mouse connection
        kmbox_send_status("Mouse connected");
        break;

    case HID_ITF_PROTOCOL_KEYBOARD:
        connection_state.keyboard_connected = true;
        connection_state.keyboard_dev_addr = dev_addr;
        neopixel_trigger_activity_flash_color(COLOR_KEYBOARD_ACTIVITY); // Flash yellow for keyboard connection
        kmbox_send_status("Keyboard connected");
        break;

    default:
        // Unknown HID protocol: silently ignore logging in the hot path
        kmbox_send_status("Unknown HID device connected");
        break;
    }

    // Update LED status instead of verbose console output
    neopixel_update_status();
}

static bool __not_in_flash_func(process_keyboard_report_internal)(const hid_keyboard_report_t *report)
{
    if (report == NULL)
    {
        return false;
    }

    // CRITICAL FIX: Check readiness before attempting to send
    // If endpoint is busy, return true anyway to avoid blocking the HID report pipeline
    // The keyboard state will be sent with the next available opportunity
    if (!tud_hid_n_ready(mouse_device_instance))
    {
        return true;  // Endpoint busy, continue processing without blocking
    }

    // Fast path: send report immediately if endpoint is ready
    bool success = tud_hid_n_report(mouse_device_instance, runtime_kbd_report_id, report, sizeof(hid_keyboard_report_t));
    if (success)
    {
        // Skip error counter reset for performance
        return true;
    }
    else
    {
        return false;
    }
}

// Helper: write a signed axis value into a raw report buffer at an arbitrary
// bit offset and bit width.  Handles 8, 12, 16, and any other width correctly,
// including packed layouts where X and Y share a byte (e.g. 12-bit Logitech).
static inline void __not_in_flash_func(write_axis_bits)(uint8_t *buf, uint8_t buf_len,
                                                         uint8_t start_byte, uint8_t bit_in_byte,
                                                         uint8_t nbits, bool is_16bit,
                                                         int16_t value)
{
    if (nbits > 0 && nbits != 8 && nbits != 16) {
        int32_t max_val = (1 << (nbits - 1)) - 1;
        int32_t min_val = -(1 << (nbits - 1));
        int32_t cv = value;
        if (cv > max_val) cv = max_val;
        if (cv < min_val) cv = min_val;
        uint32_t uval = (uint32_t)cv & ((1u << nbits) - 1);
        if (start_byte + 2 < buf_len) {
            uint32_t mask = ((1u << nbits) - 1) << bit_in_byte;
            uint32_t raw32 = (uint32_t)buf[start_byte]
                           | ((uint32_t)buf[start_byte + 1] << 8)
                           | ((uint32_t)buf[start_byte + 2] << 16);
            raw32 = (raw32 & ~mask) | ((uval << bit_in_byte) & mask);
            buf[start_byte]     = (uint8_t)(raw32 & 0xFF);
            buf[start_byte + 1] = (uint8_t)((raw32 >> 8) & 0xFF);
            buf[start_byte + 2] = (uint8_t)((raw32 >> 16) & 0xFF);
        }
    } else if (is_16bit) {
        if (start_byte + 1 < buf_len) {
            buf[start_byte]     = (uint8_t)(value & 0xFF);
            buf[start_byte + 1] = (uint8_t)((value >> 8) & 0xFF);
        }
    } else {
        int8_t cv = (value > 127) ? 127 : ((value < -128) ? -128 : (int8_t)value);
        if (start_byte < buf_len)
            buf[start_byte] = (uint8_t)cv;
    }
}

// Build a complete raw mouse report from individual values using parsed layout.
static inline void build_raw_mouse_report(uint8_t *buf, uint8_t sz,
                                           const mouse_report_layout_t *L,
                                           uint8_t buttons, int16_t x, int16_t y,
                                           int8_t wheel, int8_t pan)
{
    memset(buf, 0, sz);
    
    // Buttons (handle both 8-bit and 16-bit button fields)
    if (L->buttons_offset < sz) {
        if (L->buttons_bits > 8 && L->buttons_offset + 1 < sz) {
            // 16-bit button field: write two bytes
            buf[L->buttons_offset] = buttons;
            buf[L->buttons_offset + 1] = 0;  // High byte (buttons 9-16, typically unused)
        } else {
            // 8-bit or less
            buf[L->buttons_offset] = buttons;
        }
    }
    
    write_axis_bits(buf, sz, L->x_start_byte, L->x_bit_in_byte, L->x_bits, L->x_is_16bit, x);
    write_axis_bits(buf, sz, L->y_start_byte, L->y_bit_in_byte, L->y_bits, L->y_is_16bit, y);
    if (L->has_wheel && L->wheel_offset < sz)
        buf[L->wheel_offset] = (uint8_t)wheel;
    if (L->has_pan && L->pan_offset < sz)
        buf[L->pan_offset] = (uint8_t)pan;
}

// Classify a parsed layout for fast-path dispatch in forward_raw_mouse_report().
// Called once after parse (+ any fixups), not on the hot path.
static void classify_mouse_layout(mouse_report_layout_t *L) {
    if (!L->valid) { L->layout_class = LAYOUT_GENERIC; return; }

    // Check byte-alignment: all axes must start on byte boundary
    bool x_aligned = (L->x_bit_in_byte == 0);
    bool y_aligned = (L->y_bit_in_byte == 0);

    if (x_aligned && y_aligned && L->x_bits == 16 && L->y_bits == 16 &&
        L->x_is_16bit && L->y_is_16bit && L->buttons_bits <= 8) {
        L->layout_class = LAYOUT_FAST_16BIT;
    } else if (x_aligned && y_aligned && L->x_bits == 8 && L->y_bits == 8 &&
               !L->x_is_16bit && !L->y_is_16bit && L->buttons_bits <= 8) {
        L->layout_class = LAYOUT_FAST_8BIT;
    } else {
        L->layout_class = LAYOUT_GENERIC;
    }
}

/**
 * Core1 accumulate-only mouse report handler.
 *
 * ARCHITECTURE NOTE: TinyUSB device API (tud_hid_report, etc.) is NOT
 * thread-safe.  Core0 runs tud_task() in the main loop, so ALL device
 * API calls must happen on Core0.  This function — called from Core1's
 * tuh_hid_report_received_cb — ONLY extracts physical mouse data and
 * writes it into the shared accumulators (spinlock-protected).  Core0's
 * hid_device_task() drains the accumulators and sends the USB report.
 */
static void __not_in_flash_func(forward_raw_mouse_report)(const uint8_t *raw, uint16_t raw_len)
{
    const mouse_report_layout_t *L = &host_mouse_layout;

    int16_t phys_x = 0, phys_y = 0;
    int8_t  phys_wheel = 0, phys_pan = 0;
    uint8_t phys_buttons = 0;

    // --- Fast paths for common byte-aligned layouts (95%+ of gaming mice) ---
    // Avoids all the bitwise extraction, bounds checks, and branches below.
    if (__builtin_expect(L->layout_class == LAYOUT_FAST_16BIT, 1)) {
        // 16-bit XY, byte-aligned, <=8-bit buttons
        if (__builtin_expect(raw_len >= L->y_offset + 2, 1)) {
            phys_buttons = raw[L->buttons_offset] & ((L->buttons_bits >= 8) ? 0xFF : ((1u << L->buttons_bits) - 1));
            phys_x = (int16_t)(raw[L->x_offset] | (raw[L->x_offset + 1] << 8));
            phys_y = (int16_t)(raw[L->y_offset] | (raw[L->y_offset + 1] << 8));
            if (L->has_wheel && L->wheel_offset < raw_len) phys_wheel = (int8_t)raw[L->wheel_offset];
            if (L->has_pan && L->pan_offset < raw_len)     phys_pan = (int8_t)raw[L->pan_offset];
            goto accumulate;
        }
    } else if (L->layout_class == LAYOUT_FAST_8BIT) {
        // 8-bit XY, byte-aligned, <=8-bit buttons
        if (__builtin_expect(raw_len >= L->y_offset + 1, 1)) {
            phys_buttons = raw[L->buttons_offset] & ((L->buttons_bits >= 8) ? 0xFF : ((1u << L->buttons_bits) - 1));
            phys_x = (int8_t)raw[L->x_offset];
            phys_y = (int8_t)raw[L->y_offset];
            if (L->has_wheel && L->wheel_offset < raw_len) phys_wheel = (int8_t)raw[L->wheel_offset];
            if (L->has_pan && L->pan_offset < raw_len)     phys_pan = (int8_t)raw[L->pan_offset];
            goto accumulate;
        }
    }

    // --- Generic path: arbitrary bit-width and non-aligned fields ---

    // Extract buttons (handle both 8-bit and 16-bit button fields)
    if (L->buttons_offset < raw_len) {
        if (L->buttons_bits > 8 && L->buttons_offset + 1 < raw_len) {
            uint16_t buttons16 = raw[L->buttons_offset] | (raw[L->buttons_offset + 1] << 8);
            uint16_t mask = (L->buttons_bits >= 16) ? 0xFFFF : ((1u << L->buttons_bits) - 1);
            phys_buttons = (uint8_t)(buttons16 & mask);
        } else {
            uint8_t mask = (L->buttons_bits >= 8) ? 0xFF : ((1u << L->buttons_bits) - 1);
            phys_buttons = raw[L->buttons_offset] & mask;
        }
    }

    // X axis extraction (all bit-width variants)
    if ((L->x_bits > 0 && L->x_bits != 8 && L->x_bits != 16) || (L->x_is_16bit && (L->x_bit_offset % 8 != 0))) {
        uint8_t nbits = L->x_bits;
        uint8_t start_byte = L->x_start_byte;
        uint8_t bit_in_byte = L->x_bit_in_byte;
        uint8_t end_byte = (L->x_bit_offset + nbits - 1) / 8;
        if (end_byte < raw_len) {
            uint32_t raw32 = (uint32_t)raw[start_byte];
            if (start_byte + 1 < raw_len) raw32 |= ((uint32_t)raw[start_byte + 1] << 8);
            if (start_byte + 2 < raw_len) raw32 |= ((uint32_t)raw[start_byte + 2] << 16);
            raw32 >>= bit_in_byte;
            raw32 &= (1u << nbits) - 1;
            if (raw32 & (1u << (nbits - 1)))
                raw32 |= ~((1u << nbits) - 1);
            phys_x = (int16_t)(int32_t)raw32;
        }
    } else if (L->x_is_16bit) {
        if (L->x_offset + 1 < raw_len)
            phys_x = (int16_t)(raw[L->x_offset] | (raw[L->x_offset + 1] << 8));
    } else if (L->x_bits == 8) {
        if (L->x_offset < raw_len)
            phys_x = (int8_t)raw[L->x_offset];
    }

    // Y axis extraction
    if ((L->y_bits > 0 && L->y_bits != 8 && L->y_bits != 16) || (L->y_is_16bit && (L->y_bit_offset % 8 != 0))) {
        uint8_t nbits = L->y_bits;
        uint8_t start_byte = L->y_start_byte;
        uint8_t bit_in_byte = L->y_bit_in_byte;
        uint8_t end_byte = (L->y_bit_offset + nbits - 1) / 8;
        if (end_byte < raw_len) {
            uint32_t raw32 = (uint32_t)raw[start_byte];
            if (start_byte + 1 < raw_len) raw32 |= ((uint32_t)raw[start_byte + 1] << 8);
            if (start_byte + 2 < raw_len) raw32 |= ((uint32_t)raw[start_byte + 2] << 16);
            raw32 >>= bit_in_byte;
            raw32 &= (1u << nbits) - 1;
            if (raw32 & (1u << (nbits - 1)))
                raw32 |= ~((1u << nbits) - 1);
            phys_y = (int16_t)(int32_t)raw32;
        }
    } else if (L->y_is_16bit) {
        if (L->y_offset + 1 < raw_len)
            phys_y = (int16_t)(raw[L->y_offset] | (raw[L->y_offset + 1] << 8));
    } else if (L->y_bits == 8) {
        if (L->y_offset < raw_len)
            phys_y = (int8_t)raw[L->y_offset];
    }

    if (L->has_wheel && L->wheel_offset < raw_len) {
        phys_wheel = (int8_t)raw[L->wheel_offset];
    }
    if (L->has_pan && L->pan_offset < raw_len) {
        phys_pan = (int8_t)raw[L->pan_offset];
    }

accumulate:
    // --- Accumulate into shared state (single spinlock for all axes) ---
    kmbox_update_physical_buttons(phys_buttons & 0x1F);

    {
        int16_t tx = 0, ty = 0;
        if (phys_x != 0 || phys_y != 0) {
            kmbox_transform_movement(phys_x, phys_y, &tx, &ty);
            smooth_record_physical_movement(tx, ty);
        }
        kmbox_accumulate_mouse(tx, ty, phys_wheel, phys_pan);
    }

    // Signal Core0 that fresh physical data is available.
    // hid_device_task() on Core0 will drain accumulators and send the report.
    fresh_mouse_data = true;
    was_active = true;
}

/**
 * Accumulate-only mouse report handler.
 *
 * ARCHITECTURE: This function is called from both Core0 (fast command click
 * handler) and Core1 (legacy boot-protocol mouse path).  It MUST NOT call
 * any tud_* device API.  It only accumulates into the shared kmbox state;
 * Core0's hid_device_task() will drain and send.
 */
static bool __not_in_flash_func(process_mouse_report_internal)(const hid_mouse_report_t *report)
{
    if (report == NULL)
    {
        return false;
    }

    // Fast button validation using bitwise AND
    uint8_t valid_buttons = report->buttons & 0x1F;

    // Update physical button states in kmbox (for lock functionality)
    kmbox_update_physical_buttons(valid_buttons);

    // Accumulate movement, wheel into shared state (single spinlock)
    {
        int16_t transformed_x = 0, transformed_y = 0;
        if (report->x != 0 || report->y != 0)
        {
            kmbox_transform_movement(report->x, report->y, &transformed_x, &transformed_y);
            smooth_record_physical_movement(transformed_x, transformed_y);
        }
        kmbox_accumulate_mouse(transformed_x, transformed_y, report->wheel, 0);
    }

    // Signal Core0 that data is available
    fresh_mouse_data = true;
    was_active = true;
    return true;
}

void __not_in_flash_func(process_kbd_report)(const hid_keyboard_report_t *report)
{
    if (report == NULL)
    {
        return; // Fast fail without printf for performance
    }

    static uint32_t activity_counter = 0;
    if ((++activity_counter & KEYBOARD_ACTIVITY_MASK) == 0)
    {
        neopixel_trigger_activity_flash_color(COLOR_KEYBOARD_ACTIVITY);
    }

    // Fast forward the report
    if (process_keyboard_report_internal(report))
    {
        // Report processed successfully
    }
}

void __not_in_flash_func(process_mouse_report)(const hid_mouse_report_t *report)
{
    if (report == NULL)
    {
        return; // Fast fail without printf for performance
    }

    static uint32_t activity_counter = 0;
    if ((++activity_counter & MOUSE_ACTIVITY_MASK) == 0)
    {
        neopixel_trigger_activity_flash_color(0x000000FF); // Blue for mouse activity
    }

    // Fast forward the report
    if (process_mouse_report_internal(report))
    {
        // Report processed successfully  
    }

    // TEMPORARILY DISABLED: Rainbow effect for debugging UART
    // If the report contains movement, advance the rainbow hue based on movement
    // if (report->x != 0 || report->y != 0)
    // {
    //     neopixel_rainbow_on_movement(report->x, report->y);
    // }
}

bool find_key_in_report(const hid_keyboard_report_t *report, uint8_t keycode)
{
    if (report == NULL)
    {
        return false;
    }

    // Unrolled loop - HID_KEYBOARD_KEYCODE_COUNT is always 6
    // Avoids branch overhead on RP2350 Cortex-M33 pipeline
    return (report->keycode[0] == keycode) |
           (report->keycode[1] == keycode) |
           (report->keycode[2] == keycode) |
           (report->keycode[3] == keycode) |
           (report->keycode[4] == keycode) |
           (report->keycode[5] == keycode);
}

void hid_device_task(void)
{
    // Xbox mode: skip HID device task entirely, xbox_device_task handles it
    if (g_xbox_mode) return;

    if (pending_device_reenumeration) {
        pending_device_reenumeration = false;
        __dmb();
        force_usb_reenumeration();
        return;
    }

    // Use cheap microsecond timer for polling with optional jitter.
    // Real mice have crystal/scheduling jitter; perfectly regular 1ms is detectable.
    static uint32_t start_us = 0;
    static int32_t jitter_us = 0;  // per-frame jitter offset (set after each send)
    uint32_t current_us = time_us_32();
    uint32_t elapsed_us = current_us - start_us;

    // Event-driven fast path: when fresh physical mouse data is available,
    // bypass the 1ms timer and send ASAP. The USB endpoint readiness
    // (tud_hid_n_ready check below) naturally throttles us to the PC's
    // polling rate. This reduces average passthrough latency from ~500μs to ~5μs.
    bool force_immediate = fresh_mouse_data;

    if (!force_immediate) {
        // CRITICAL: Signed arithmetic to avoid uint32 wrap when jitter is negative.
        int32_t interval_signed = (int32_t)(HID_DEVICE_TASK_INTERVAL_MS * 1000) + jitter_us;
        if (interval_signed < 500) interval_signed = 500;
        if (interval_signed > 2500) interval_signed = 2500;

        if (elapsed_us < (uint32_t)interval_signed)
        {
            return; // Not enough time elapsed
        }
    }
    start_us = current_us;

    // Remote wakeup handling — wake on button press OR pending injection data.
    // Without this, USB suspend blocks all smooth injection output until the
    // physical mouse moves (which prevents suspend in the first place).
    if (tud_suspended())
    {
        bool has_data = !gpio_get(PIN_BUTTON) ||
                        smooth_has_pending() ||
                        kmbox_has_pending_movement();
        if (has_data) {
            tud_remote_wakeup();
        }
        return;
    }

    // Only send reports when USB device is properly mounted and ready
    if (!tud_mounted() || !tud_ready())
    {
        return;
    }

    // --- Track last-sent button state for change detection ---
    // Real mice send a report immediately on button state changes.
    // A bridge-injected click with no movement must trigger a report
    // on the very next poll — otherwise the delay is detectably wrong.
    // (last_sent_buttons / was_active are file-scope for cross-path sync)

    // ARCHITECTURE: Core0 is the ONLY core that calls tud_hid_report().
    // Core1 (physical mouse callbacks) accumulates into kmbox accumulators
    // and sets was_active=true.  We drain everything here.
    if (tud_hid_n_ready(mouse_device_instance))
    {
        // Clear fresh data signal — if Core1 sets it between here and the
        // drain, the data is still in the accumulators and will be caught
        // on the next iteration (tud_hid_n_ready gates re-entry anyway).
        fresh_mouse_data = false;

        // Cache humanization mode for this frame — avoid 3 function calls per frame
        const humanization_mode_t frame_human_mode = smooth_get_humanization_mode();

        // Atomic check-and-drain: single spinlock instead of separate
        // kmbox_has_pending_movement() + kmbox_get_mouse_report_16() (was 2 spinlock roundtrips)
        uint8_t buttons;
        int16_t x, y;
        int8_t wheel, pan;
        bool has_kmbox = kmbox_try_drain_mouse_16(last_sent_buttons,
                                                   &buttons, &x, &y, &wheel, &pan);
        bool has_smooth = smooth_has_pending();
        bool buttons_changed = (buttons != last_sent_buttons);

        bool has_pending = has_kmbox || has_smooth || buttons_changed;

        if (!has_pending) goto check_idle;

        // Process smooth injection (int16_t for high-DPI support)
        int16_t smooth_x = 0, smooth_y = 0;
        if (has_smooth) {
            smooth_process_frame(&smooth_x, &smooth_y);
            x += smooth_x;
            y += smooth_y;
        }

        // Apply output-stage humanization (tremor).
        // Physical mouse movement is already human; only humanize the
        // synthetic (smooth injection + bridge) portion.
        if (!connection_state.mouse_connected) {
            // No physical mouse — everything is synthetic, full humanization
            apply_output_humanization(&x, &y, x, y);
        } else {
            // Physical mouse connected — only humanize the smooth injection part
            if (smooth_x != 0 || smooth_y != 0) {
                apply_output_humanization(&x, &y, smooth_x, smooth_y);
            }
        }

        // All output-stage noise is scaled by movement magnitude to prevent
        // overwhelming low-speed signals.  At 1-2 counts, ±1 noise is 50-100%
        // perturbation, creating chaotic scribble.
        if (frame_human_mode != HUMANIZATION_OFF &&
            (x != 0 || y != 0)) {
            float out_mag = sqrtf((float)x * x + (float)y * y);
            float noise_gate = fminf(1.0f, out_mag / 4.0f);  // ramp 0→1 over [0,4]px

            // Sub-pixel quantization noise: real sensors accumulate fractional
            // pixel residuals that occasionally leak as ±1 count perturbations.
            // Accumulator stddev 0.45 → crosses ±1 roughly every 3-5 frames
            // during fast movement, producing 3-8% sub-pixel noise.
            static float subpx_accum_x = 0.0f;
            static float subpx_accum_y = 0.0f;
            float subpx_noise = 0.45f * noise_gate;
            subpx_accum_x += hid_rng_gaussian() * subpx_noise;
            subpx_accum_y += hid_rng_gaussian() * subpx_noise;
            if (subpx_accum_x >= 1.0f) { x += 1; subpx_accum_x -= 1.0f; }
            else if (subpx_accum_x <= -1.0f) { x -= 1; subpx_accum_x += 1.0f; }
            if (subpx_accum_y >= 1.0f) { y += 1; subpx_accum_y -= 1.0f; }
            else if (subpx_accum_y <= -1.0f) { y -= 1; subpx_accum_y += 1.0f; }

            // Sensor noise: gaussian-based, also scaled by magnitude.
            // Only apply when movement is large enough that noise won't dominate.
            if (noise_gate > 0.25f) {
                x = apply_sensor_noise(x);
                y = apply_sensor_noise(y);
            }
        }

        // Send if there's any movement, wheel, OR button state to report.
        if (x != 0 || y != 0 || wheel != 0 || buttons != 0 || buttons_changed) {
            // Build raw report matching host mouse descriptor when available.
            if (using_16bit_output_override) {
                uint8_t raw[16];
                build_raw_mouse_report(raw, sizeof(raw), &output_mouse_layout_16bit,
                                       buttons, x, y, wheel, pan);
                tud_hid_n_report(mouse_device_instance, REPORT_ID_MOUSE, raw, output_mouse_layout_16bit.report_size);
            } else if (host_mouse_layout.valid && host_mouse_desc_len > 0) {
                uint8_t raw[64];
                uint8_t sz = host_mouse_layout.report_size;
                if (sz > sizeof(raw)) sz = sizeof(raw);

                build_raw_mouse_report(raw, sz, &host_mouse_layout,
                                       buttons, x, y, wheel, pan);

                uint8_t rid = host_mouse_layout.has_report_id ? host_mouse_layout.mouse_report_id : REPORT_ID_MOUSE;
                tud_hid_n_report(mouse_device_instance, rid, raw, sz);
            } else {
                // Clamp to int8 for standard HID mouse report
                int8_t cx = (x > 127) ? 127 : ((x < -128) ? -128 : (int8_t)x);
                int8_t cy = (y > 127) ? 127 : ((y < -128) ? -128 : (int8_t)y);
                tud_hid_n_mouse_report(mouse_device_instance, REPORT_ID_MOUSE, buttons, cx, cy, wheel, pan);
            }
            last_sent_buttons = buttons;
            was_active = true;

            // Compute next frame's timing jitter.
            // Always jitter when humanization is active — perfectly regular 1ms
            // intervals are a fingerprint regardless of physical mouse state.
            // Real USB polling has crystal oscillator drift + OS scheduling jitter.
            if (frame_human_mode != HUMANIZATION_OFF) {
                // Gaussian jitter, stddev ~350us → CV ≈ 0.35 on 1000us base.
                // Smaller than before to avoid excessive clamping; range is
                // roughly ±700us (95th percentile), keeping interval in [500, 2000].
                jitter_us = (int32_t)(hid_rng_gaussian() * 350.0f);
            } else {
                jitter_us = 0;
            }
            return;
        }
    }
check_idle:

    // --- Active → idle edge: send one final zero-delta stop report ---
    // Real mice send a last report with zero deltas (confirming the stop)
    // before they begin NAKing idle polls.  Mirror that behavior here.
    if (was_active && tud_hid_n_ready(mouse_device_instance))
    {
        uint8_t current_buttons = kmbox_get_current_buttons();
        if (using_16bit_output_override) {
            uint8_t raw[16];
            build_raw_mouse_report(raw, sizeof(raw), &output_mouse_layout_16bit,
                                   current_buttons, 0, 0, 0, 0);
            tud_hid_n_report(mouse_device_instance, REPORT_ID_MOUSE, raw, output_mouse_layout_16bit.report_size);
        } else if (host_mouse_layout.valid && host_mouse_desc_len > 0) {
            uint8_t raw[64];
            uint8_t sz = host_mouse_layout.report_size;
            if (sz > sizeof(raw)) sz = sizeof(raw);

            build_raw_mouse_report(raw, sz, &host_mouse_layout,
                                   current_buttons, 0, 0, 0, 0);

            uint8_t rid = host_mouse_layout.has_report_id ? host_mouse_layout.mouse_report_id : REPORT_ID_MOUSE;
            tud_hid_n_report(mouse_device_instance, rid, raw, sz);
        } else {
            tud_hid_n_mouse_report(mouse_device_instance, REPORT_ID_MOUSE, current_buttons, 0, 0, 0, 0);
        }
        last_sent_buttons = current_buttons;
        was_active = false;
        return;
    }

    // --- Truly idle: NAK naturally (no report sent) ---
    // When the mouse is idle, real mice NAK interrupt IN polls.
    // Sending continuous zero reports when the real device wouldn't is
    // itself a fingerprint.  So we intentionally do nothing here when
    // a physical mouse is connected.

    // Only send fallback reports when NO devices are connected at all
    // (standalone mode without a physical mouse/keyboard attached).
    if (!connection_state.mouse_connected && !connection_state.keyboard_connected)
    {
        send_hid_report(REPORT_ID_MOUSE);
    }

    // --- Drain vendor report queue (Core1 → Core0 passthrough) ---
    // Forward vendor/non-mouse reports from the host device to the downstream PC.
    // This enables software like Logitech G Hub / Razer Synapse to communicate
    // through the proxy for battery status, DPI changes, lighting, etc.
    while (vendor_fwd_queue.tail != vendor_fwd_queue.head) {
        vendor_report_entry_t *e = &vendor_fwd_queue.entries[vendor_fwd_queue.tail];
        if (e->device_instance < CFG_TUD_HID && tud_hid_n_ready(e->device_instance)) {
            tud_hid_n_report(e->device_instance, e->report_id, e->data, e->len);
        }
        __dmb();
        vendor_fwd_queue.tail = (vendor_fwd_queue.tail + 1) & VENDOR_QUEUE_MASK;
    }
}

void send_hid_report(uint8_t report_id)
{
    // Multiple checks to prevent endpoint allocation conflicts
    if (!tud_mounted() || !tud_ready())
    {
        return; // Prevent endpoint conflicts by not sending reports when device isn't ready
    }

    // Additional check to ensure device stack is stable
    static uint32_t last_mount_check = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_mount_check > 1000)
    { // Check every second
        if (!tud_mounted())
        {
            return; // Device was unmounted, don't send reports
        }
        last_mount_check = current_time;
    }

    switch (report_id)
    {
    case REPORT_ID_KEYBOARD:
        if (!connection_state.keyboard_connected)
        {
            // Check device readiness before each report
            if (tud_hid_n_ready(mouse_device_instance))
            {
                // Use static array to avoid stack allocation overhead
                static const uint8_t empty_keycode[HID_KEYBOARD_KEYCODE_COUNT] = {0};
                tud_hid_n_keyboard_report(mouse_device_instance, runtime_kbd_report_id, 0, empty_keycode);
            }
        }
        break;

    case REPORT_ID_MOUSE:
        // Only send button-based mouse movement if no mouse is connected
        if (!connection_state.mouse_connected)
        {
            // Check device readiness before each report
            if (tud_hid_n_ready(mouse_device_instance))
            {
                static bool prev_button_state = true; // true = not pressed (active low)
                bool current_button_state = gpio_get(PIN_BUTTON);

                if (!current_button_state)
                { // button pressed (active low)
                    // Mouse move up (negative Y direction)
                    tud_hid_n_mouse_report(mouse_device_instance, REPORT_ID_MOUSE, MOUSE_BUTTON_NONE,
                                         MOUSE_NO_MOVEMENT, MOUSE_BUTTON_MOVEMENT_DELTA,
                                         MOUSE_NO_MOVEMENT, MOUSE_NO_MOVEMENT);
                }
                else if (prev_button_state != current_button_state)
                {
                    // Send stop movement when button is released
                    tud_hid_n_mouse_report(mouse_device_instance, REPORT_ID_MOUSE, MOUSE_BUTTON_NONE,
                                         MOUSE_NO_MOVEMENT, MOUSE_NO_MOVEMENT,
                                         MOUSE_NO_MOVEMENT, MOUSE_NO_MOVEMENT);
                }

                prev_button_state = current_button_state;
            }
        }
        break;

    case REPORT_ID_CONSUMER_CONTROL:
    {
        // CRITICAL: Check device readiness before each report
        if (tud_hid_n_ready(mouse_device_instance))
        {
            static const uint16_t empty_key = 0;
            tud_hid_n_report(mouse_device_instance, runtime_consumer_report_id, &empty_key, HID_CONSUMER_CONTROL_SIZE);
        }
        break;
    }

    default:
        break;
    }
}

void hid_host_task(void)
{
    // Drain SET_REPORT passthrough queue (Core0 → Core1).
    // Forward vendor SET_REPORT requests from the downstream PC to the real mouse.
    // Process at most 1 per call — tuh_hid_set_report() may block on USB
    // control transfer, and draining the full queue could stall Core1's PIO USB.
    if (set_report_queue.tail != set_report_queue.head) {
        set_report_entry_t *e = &set_report_queue.entries[set_report_queue.tail];
        tuh_hid_set_report(e->host_dev_addr, e->host_instance,
                           e->report_id, e->report_type,
                           (void*)e->data, e->len);
        __dmb();
        set_report_queue.tail = (set_report_queue.tail + 1) & VENDOR_QUEUE_MASK;
    }

    if (get_report_bridge.pending && !get_report_bridge.busy) {
        get_report_bridge.pending = false;
        get_report_bridge.done = false;
        get_report_bridge.actual_len = 0;
        get_report_bridge.busy = true;
        __dmb();

        bool started = tuh_hid_get_report(get_report_bridge.host_dev_addr,
                                          get_report_bridge.host_instance,
                                          get_report_bridge.report_id,
                                          get_report_bridge.report_type,
                                          get_report_bridge.data,
                                          get_report_bridge.request_len);
        if (!started) {
            get_report_bridge.busy = false;
            get_report_bridge.done = true;
        }
    }
}

// Device callbacks with improved error handling
void tud_mount_cb(void)
{
    led_set_blink_interval(LED_BLINK_MOUNTED_MS);
    neopixel_update_status();
}

void tud_umount_cb(void)
{
    led_set_blink_interval(LED_BLINK_UNMOUNTED_MS);

    // Track device unmount as potential error condition
    usb_error_tracker.consecutive_device_errors++;

    neopixel_update_status();
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    led_set_blink_interval(LED_BLINK_SUSPENDED_MS);
    neopixel_update_status();
}

void tud_resume_cb(void)
{
    led_set_blink_interval(LED_BLINK_RESUMED_MS);
    neopixel_update_status();
}

// Host callbacks with improved error handling
void tuh_mount_cb(uint8_t dev_addr)
{
    (void)dev_addr;
    neopixel_trigger_activity_flash_color(COLOR_USB_CONNECTION);
    neopixel_update_status();
}

void tuh_umount_cb(uint8_t dev_addr)
{

    // Handle device disconnection
    handle_device_disconnection(dev_addr);

    static uint32_t last_unmount_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // If unmounts happen in quick succession, count as consecutive errors
    if (current_time - last_unmount_time < 5000)
    {
        usb_error_tracker.consecutive_host_errors++;
    }
    else
    {
        // Reset error count for normal disconnections
        usb_error_tracker.consecutive_host_errors = 0;
    }
    last_unmount_time = current_time;

    // Trigger visual feedback
    neopixel_trigger_activity_flash_color(COLOR_USB_DISCONNECTION);
    neopixel_update_status();
}

// HID host callbacks — multi-interface mirroring with vendor report passthrough
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, const uint8_t *desc_report, uint16_t desc_len)
{
    uint16_t vid, pid;
    tuh_vid_pid_get(dev_addr, &vid, &pid);

    // === DEVICE-LEVEL CLONING (once per physical device) ===
    bool first_interface_for_device = (cloned_dev_addr != dev_addr);

    if (first_interface_for_device) {
        cloned_dev_addr = dev_addr;
        mounted_hid_itf_count = 0;
        mirrored_itf_count = 0;
        mouse_device_instance = 0;
        checked_runtime_mouse_override = false;
        pending_device_reenumeration = false;
        memset(&get_report_bridge, 0, sizeof(get_report_bridge));
        memset(mirrored_itfs, 0, sizeof(mirrored_itfs));

        // Fetch string descriptors from the attached device
        fetch_device_string_descriptors(dev_addr);

        // Capture full device descriptor for identity cloning
        tusb_desc_device_t host_dev_desc;
        if (tuh_descriptor_get_device_sync(dev_addr, &host_dev_desc, sizeof(host_dev_desc)) == XFER_RESULT_SUCCESS) {
            host_device_info.bcdUSB          = host_dev_desc.bcdUSB;
            host_device_info.bDeviceClass    = host_dev_desc.bDeviceClass;
            host_device_info.bDeviceSubClass = host_dev_desc.bDeviceSubClass;
            host_device_info.bDeviceProtocol = host_dev_desc.bDeviceProtocol;
            host_device_info.bMaxPacketSize0 = host_dev_desc.bMaxPacketSize0;
            host_device_info.bcdDevice       = host_dev_desc.bcdDevice;
            host_device_info.valid           = true;

            // Track string indices from device descriptor
            if (host_dev_desc.iManufacturer > max_string_index_seen) max_string_index_seen = host_dev_desc.iManufacturer;
            if (host_dev_desc.iProduct > max_string_index_seen) max_string_index_seen = host_dev_desc.iProduct;
            if (host_dev_desc.iSerialNumber > max_string_index_seen) max_string_index_seen = host_dev_desc.iSerialNumber;
        }

        // Capture and parse full configuration descriptor (all interfaces + endpoints)
        uint8_t cfg_buf[512];
        if (tuh_descriptor_get_configuration_sync(dev_addr, 0, cfg_buf, sizeof(cfg_buf)) == XFER_RESULT_SUCCESS) {
            uint16_t cfg_total = cfg_buf[2] | (cfg_buf[3] << 8);
            if (cfg_total > sizeof(cfg_buf)) cfg_total = sizeof(cfg_buf);
            parse_host_config_descriptor(cfg_buf, cfg_total);
        }

        // Fetch any extra string descriptors beyond the standard 3 (manufacturer, product, serial)
        for (uint8_t si = 4; si <= max_string_index_seen && extra_string_count < MAX_CACHED_STRINGS; si++) {
            uint16_t tmp_buf[48];
            memset(tmp_buf, 0, sizeof(tmp_buf));
            if (tuh_descriptor_get_string_sync(dev_addr, si, LANGUAGE_ID, tmp_buf, sizeof(tmp_buf)) == XFER_RESULT_SUCCESS) {
                cached_string_t *cs = &extra_strings[extra_string_count];
                cs->index = si;
                utf16_to_utf8(tmp_buf, sizeof(tmp_buf), cs->str, sizeof(cs->str));
                cs->valid = (strlen(cs->str) > 0);
                if (cs->valid) extra_string_count++;
            }
        }
    }

    mounted_hid_itf_count++;

    // === DETERMINE EFFECTIVE PROTOCOL ===
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    hid_instance_info_t *inst_info = alloc_hid_instance(dev_addr, instance);
    uint8_t effective_protocol = itf_protocol;

    if (itf_protocol == HID_ITF_PROTOCOL_NONE && desc_report != NULL && desc_len > 0) {
        effective_protocol = detect_usage_from_report_descriptor(desc_report, desc_len, inst_info);
    }
    if (inst_info) {
        inst_info->effective_protocol = effective_protocol;
    }

    bool is_mouse_interface = (effective_protocol == HID_ITF_PROTOCOL_MOUSE);

    // === ALLOCATE MIRRORED INTERFACE SLOT ===
    // Each host HID interface gets a corresponding device-side interface.
    uint8_t mirror_idx = mirrored_itf_count;
    if (mirror_idx < MAX_DEVICE_HID_INTERFACES) {
        mirrored_interface_t *mitf = &mirrored_itfs[mirror_idx];
        mitf->active = true;
        mitf->host_dev_addr = dev_addr;
        mitf->host_instance = instance;
        mitf->is_mouse = is_mouse_interface;

        // Endpoint config was already parsed from config descriptor;
        // the Nth HID interface in the config maps to mirrored_itfs[N].
        // (parse_host_config_descriptor pre-populated subclass/protocol/ep_*)

        // Track which device-side instance is the mouse (composite descriptor)
        if (is_mouse_interface) {
            mouse_device_instance = mirror_idx;
        }

        if (is_mouse_interface && desc_report != NULL && desc_len > 0) {
            // --- MOUSE INTERFACE ---
            // Store full descriptor (including vendor collections) for the mouse interface.
            size_t copy_len = desc_len;
            if (copy_len > sizeof(host_mouse_desc)) copy_len = sizeof(host_mouse_desc);
            memcpy(host_mouse_desc, desc_report, copy_len);
            host_mouse_desc_len = copy_len;

            // Parse mouse report layout for raw forwarding
            parse_mouse_report_layout(host_mouse_desc, host_mouse_desc_len, &host_mouse_layout);

            // Detect OS descriptor mismatches (8-bit desc for 16-bit mouse)
            if (host_mouse_layout.valid &&
                host_mouse_layout.x_bits == 8 && host_mouse_layout.y_bits == 8 &&
                host_mouse_layout.report_size >= 8 && host_mouse_layout.buttons_bits >= 8) {
                host_mouse_layout.x_bits = 16;
                host_mouse_layout.y_bits = 16;
                host_mouse_layout.x_is_16bit = true;
                host_mouse_layout.y_is_16bit = true;
                host_mouse_layout.report_size = 8;
            }

            // Classify layout for fast-path dispatch (after all fixups)
            classify_mouse_layout(&host_mouse_layout);

            // Extract mouse report ID
            if (host_mouse_layout.valid && host_mouse_layout.has_report_id) {
                host_mouse_has_report_id = true;
                host_mouse_report_id = host_mouse_layout.mouse_report_id;
            } else {
                host_mouse_has_report_id = false;
                host_mouse_report_id = 0;
                for (size_t i = 0; i + 1 < host_mouse_desc_len; ++i) {
                    if (host_mouse_desc[i] == 0x85) {
                        host_mouse_has_report_id = true;
                        host_mouse_report_id = host_mouse_desc[i + 1];
                        break;
                    }
                }
            }

            if (inst_info) {
                inst_info->has_report_id = host_mouse_has_report_id;
                inst_info->mouse_report_id = host_mouse_report_id;
            }

            // Build composite HID report descriptor (keyboard + mouse + consumer)
            build_runtime_hid_report_with_mouse(host_mouse_desc, host_mouse_desc_len);

        } else if (desc_report != NULL && desc_len > 0) {
            // --- NON-MOUSE INTERFACE (keyboard-macros, vendor, etc.) ---
            // Store verbatim descriptor for faithful mirroring
            size_t copy_len = desc_len;
            if (copy_len > MIRROR_ITF_DESC_MAX) copy_len = MIRROR_ITF_DESC_MAX;
            memcpy(mitf->report_desc, desc_report, copy_len);
            mitf->report_desc_len = copy_len;
        }

        mirrored_itf_count++;
    }

    // Handle HID device connection using effective protocol
    handle_hid_device_connection(dev_addr, effective_protocol);

    // === START RECEIVING REPORTS ===
    // Receive from ALL interfaces on the mouse device for vendor report passthrough.
    // Standalone keyboards on a different device are also received.
    // Filtering of composite macro-keyboard garbage happens in report_received_cb.
    bool should_receive = false;

    if (is_mouse_interface) {
        should_receive = true;
    } else if (dev_addr != connection_state.mouse_dev_addr) {
        // Standalone keyboard on different device
        should_receive = true;
    } else {
        // Non-mouse interface on same device — receive for vendor report passthrough
        should_receive = true;
    }

    if (should_receive) {
        if (!tuh_hid_receive_report(dev_addr, instance)) {
            neopixel_trigger_activity_flash_color(COLOR_USB_DISCONNECTION);
        }
    }

    // === DEFERRED RE-ENUMERATION ===
    // Wait until all expected HID interfaces have mounted before triggering
    // re-enumeration.  This ensures the config descriptor includes ALL
    // interfaces, not just the first one.
    if (mounted_hid_itf_count >= expected_hid_itf_count) {
        // All interfaces captured — build final descriptors
        if (host_mouse_desc_len == 0) {
            // No mouse interface found; use defaults
            build_runtime_hid_report_with_mouse(NULL, 0);
        }
        rebuild_configuration_descriptor();

        // Trigger re-enumeration (only fires if VID/PID actually changed)
        set_attached_device_vid_pid(vid, pid);
    }

    neopixel_update_status();
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    (void)instance;

    // Free per-instance tracking for this device
    free_hid_instances_for_device(dev_addr);

    // Reset string descriptors when device is disconnected
    reset_device_string_descriptors();

    // Handle device disconnection
    handle_device_disconnection(dev_addr);

    // Trigger visual feedback
    neopixel_trigger_activity_flash_color(COLOR_USB_DISCONNECTION);
    neopixel_update_status();
}

// Helper: parse a mouse report from raw bytes, handling variable report sizes.
// When we have a parsed host mouse layout (gaming mice), forward the raw bytes
// with kmbox/smooth deltas injected.  Otherwise fall back to the legacy path
// that re-encodes through hid_mouse_report_t.
static void __not_in_flash_func(parse_and_forward_mouse_report)(const uint8_t *data, uint16_t data_len)
{
    if (data_len < 3) return;

    // --- RUNTIME DETECTION: 16-bit mouse with wrong descriptor ---
    // macOS/Windows may expose an 8-bit descriptor for 16-bit mice (G703, etc.)
    // Detect by comparing actual report size vs descriptor-claimed size
    if (!checked_runtime_mouse_override && host_mouse_layout.valid && data_len >= 8) {
        checked_runtime_mouse_override = true;
        if (host_mouse_layout.x_bits == 8 && host_mouse_layout.y_bits == 8) {
            // Descriptor says 8-bit but we're receiving 8+ byte reports
            // Force 16-bit override
            host_mouse_layout.x_bits = 16;
            host_mouse_layout.y_bits = 16;
            host_mouse_layout.x_is_16bit = true;
            host_mouse_layout.y_is_16bit = true;
            host_mouse_layout.report_size = 8;
            
            // Rebuild descriptors with 16-bit override
            build_runtime_hid_report_with_mouse(host_mouse_desc, host_mouse_desc_len);
            rebuild_configuration_descriptor();
            request_device_reenumeration();
        }
    }

    // --- RAW FORWARDING PATH (gaming mice with cloned descriptor) ---
    if (host_mouse_layout.valid && host_mouse_desc_len > 0) {
        forward_raw_mouse_report(data, data_len);
        return;
    }

    // --- LEGACY FALLBACK PATH (boot-protocol / unknown mice) ---
    hid_mouse_report_t mouse_report_local;
    
    if (data_len == 8) {
        // 16-bit coordinate mouse  - check if this is G703-style or legacy
        // G703: bytes 0-1=buttons(16bit), 2-3=X(16bit), 4-5=Y(16bit), 6=wheel, 7=pan
        // Legacy: bytes 0=buttons(8bit), 1=X(8bit), 2=Y(8bit), 3=wheel, 4=pan, 5-7=X(16bit)/Y(16bit)
        // Heuristic: if byte 1 looks like button data (mostly zeros), use G703 layout
        bool is_g703_layout = (data[1] == 0);  // Byte 1 is high button bits (usually 0)
        
        if (is_g703_layout) {
            // G703 Lightspeed layout: 16-bit buttons at 0-1, 16-bit X at 2-3, 16-bit Y at 4-5
            mouse_report_local.buttons = data[0];  // Only low 8 button bits
            
            int16_t x16 = (int16_t)(data[2] | (data[3] << 8));
            int16_t y16 = (int16_t)(data[4] | (data[5] << 8));
            
            mouse_report_local.x = (x16 > 127) ? 127 : (x16 < -128) ? -128 : (int8_t)x16;
            mouse_report_local.y = (y16 > 127) ? 127 : (y16 < -128) ? -128 : (int8_t)y16;
            mouse_report_local.wheel = (int8_t)data[6];
            mouse_report_local.pan = (int8_t)data[7];
        } else {
            // Legacy layout: assume X at bytes 4-5, Y at bytes 6-7
            mouse_report_local.buttons = data[0];
            
            int16_t x16 = (int16_t)(data[4] | (data[5] << 8)) >> 2;
            int16_t y16 = (int16_t)(data[6] | (data[7] << 8)) >> 2;
            
            mouse_report_local.x = (x16 > 127) ? 127 : (x16 < -128) ? -128 : (int8_t)x16;
            mouse_report_local.y = (y16 > 127) ? 127 : (y16 < -128) ? -128 : (int8_t)y16;
            
            // Wheel detection
            mouse_report_local.wheel = (data[1] != 0) ? (int8_t)data[1] : 
                                       (data[2] != 0) ? (int8_t)data[2] : (int8_t)data[3];
            mouse_report_local.pan = 0;
        }
    } else {
        // Standard format - can cast directly if length allows
        if (data_len >= sizeof(hid_mouse_report_t)) {
            process_mouse_report((const hid_mouse_report_t*)data);
            return;
        }
        
        // Partial report - build manually
        mouse_report_local.buttons = data[0];
        mouse_report_local.x = (int8_t)data[1];
        mouse_report_local.y = (int8_t)data[2];
        mouse_report_local.wheel = (data_len >= 4) ? (int8_t)data[3] : 0;
        mouse_report_local.pan = (data_len >= 5) ? (int8_t)data[4] : 0;
    }

    process_mouse_report(&mouse_report_local);
}

static void cache_report(uint8_t device_instance, uint8_t report_id, uint8_t report_type,
                         const uint8_t *data, uint8_t data_len);

// Queue a vendor/non-mouse report for Core0 to send on the device side.
// Called from Core1 — must not call any tud_* functions.
static void __not_in_flash_func(queue_vendor_report)(uint8_t device_instance, uint8_t report_id,
                                                       const uint8_t *data, uint8_t data_len)
{
    uint8_t capped_len = (data_len > VENDOR_REPORT_MAX_LEN) ? VENDOR_REPORT_MAX_LEN : data_len;

    // Update GET_REPORT cache so tud_hid_get_report_cb can respond to macOS IOKit.
    cache_report(device_instance, report_id, HID_REPORT_TYPE_INPUT, data, capped_len);

    // Queue for Core0 to forward via interrupt IN endpoint
    uint8_t next_head = (vendor_fwd_queue.head + 1) & VENDOR_QUEUE_MASK;
    if (next_head == vendor_fwd_queue.tail) return;  // Queue full, drop

    vendor_report_entry_t *e = &vendor_fwd_queue.entries[vendor_fwd_queue.head];
    e->device_instance = device_instance;
    e->report_id = report_id;
    e->len = capped_len;
    memcpy(e->data, data, capped_len);

    __dmb();  // Ensure data written before head advances
    vendor_fwd_queue.head = next_head;
}

// Find which mirrored interface slot corresponds to a host (dev_addr, instance).
static mirrored_interface_t* find_mirrored_interface(uint8_t dev_addr, uint8_t instance) {
    for (uint8_t i = 0; i < mirrored_itf_count; i++) {
        if (mirrored_itfs[i].active &&
            mirrored_itfs[i].host_dev_addr == dev_addr &&
            mirrored_itfs[i].host_instance == instance) {
            return &mirrored_itfs[i];
        }
    }
    return NULL;
}

// Get the device-side instance index for a mirrored interface.
static uint8_t mirrored_device_instance(const mirrored_interface_t *mitf) {
    return (uint8_t)(mitf - mirrored_itfs);
}

static void cache_report(uint8_t device_instance, uint8_t report_id, uint8_t report_type,
                         const uint8_t *data, uint8_t data_len)
{
    if (device_instance >= MAX_DEVICE_HID_INTERFACES || data == NULL) return;

    uint8_t capped_len = (data_len > VENDOR_REPORT_MAX_LEN) ? VENDOR_REPORT_MAX_LEN : data_len;
    cached_report_t *slots = report_cache[device_instance];
    int slot = -1;
    int empty = -1;
    for (int i = 0; i < REPORT_CACHE_SLOTS_PER_ITF; i++) {
        if (slots[i].valid && slots[i].report_id == report_id && slots[i].report_type == report_type) {
            slot = i;
            break;
        }
        if (!slots[i].valid && empty < 0) empty = i;
    }
    if (slot < 0) slot = (empty >= 0) ? empty : 0;
    slots[slot].report_id = report_id;
    slots[slot].report_type = report_type;
    slots[slot].len = capped_len;
    memcpy(slots[slot].data, data, capped_len);
    __dmb();
    slots[slot].valid = true;
}

void __not_in_flash_func(tuh_hid_report_received_cb)(uint8_t dev_addr, uint8_t instance, const uint8_t *report, uint16_t len)
{
    if (report == NULL || len == 0)
    {
        tuh_hid_receive_report(dev_addr, instance);
        return;
    }

    // Look up effective protocol from per-instance tracking
    uint8_t effective_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    hid_instance_info_t *inst_info = find_hid_instance(dev_addr, instance);
    bool has_report_id = false;
    uint8_t mouse_report_id_local = 0;

    if (inst_info) {
        effective_protocol = inst_info->effective_protocol;
        has_report_id = inst_info->has_report_id;
        mouse_report_id_local = inst_info->mouse_report_id;
    }

    switch (effective_protocol)
    {
    case HID_ITF_PROTOCOL_KEYBOARD:
        {
            // Only forward keyboard from standalone devices (different device than mouse)
            if (dev_addr == connection_state.mouse_dev_addr) {
                // Composite gaming mouse macro/media keys — queue for vendor passthrough
                // instead of interpreting as keyboard input
                mirrored_interface_t *mitf = find_mirrored_interface(dev_addr, instance);
                if (mitf) {
                    uint8_t dev_inst = mirrored_device_instance(mitf);
                    uint8_t rid = (has_report_id && len > 0) ? report[0] : 0;
                    const uint8_t *data = (has_report_id && len > 0) ? report + 1 : report;
                    uint8_t dlen = (has_report_id && len > 0) ? (uint8_t)(len - 1) : (uint8_t)len;
                    queue_vendor_report(dev_inst, rid, data, dlen);
                }
                break;
            }

            const uint8_t *kbd_data = report;
            uint16_t kbd_len = len;

            if (has_report_id && kbd_len > 0) {
                kbd_data++;
                kbd_len--;
            }

            if (kbd_len >= (int)sizeof(hid_keyboard_report_t)) {
                process_kbd_report((const hid_keyboard_report_t*)kbd_data);
            }
        }
        break;

    case HID_ITF_PROTOCOL_MOUSE:
        {
            const uint8_t *mouse_data = report;
            uint16_t mouse_len = len;

            if (has_report_id && mouse_len > 0) {
                uint8_t received_id = mouse_data[0];
                if (received_id != mouse_report_id_local) {
                    // Not the mouse report — this is a vendor report on the mouse
                    // interface (e.g. Logitech HID++ on same interface as mouse).
                    // Queue for passthrough to device side.
                    mirrored_interface_t *mitf = find_mirrored_interface(dev_addr, instance);
                    if (mitf) {
                        uint8_t dev_inst = mirrored_device_instance(mitf);
                        queue_vendor_report(dev_inst, received_id, mouse_data + 1, (uint8_t)(mouse_len - 1));
                    }
                    break;
                }
                mouse_data++;
                mouse_len--;
            }

            parse_and_forward_mouse_report(mouse_data, mouse_len);
        }
        break;

    default:
        {
            // Unknown/vendor protocol — forward entire report for passthrough
            mirrored_interface_t *mitf = find_mirrored_interface(dev_addr, instance);
            if (mitf) {
                uint8_t dev_inst = mirrored_device_instance(mitf);
                uint8_t rid = (has_report_id && len > 0) ? report[0] : 0;
                const uint8_t *data = (has_report_id && len > 0) ? report + 1 : report;
                uint8_t dlen = (has_report_id && len > 0) ? (uint8_t)(len - 1) : (uint8_t)len;
                queue_vendor_report(dev_inst, rid, data, dlen);
            }
        }
        break;
    }

    // Continue to request reports
    tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_get_report_complete_cb(uint8_t dev_addr, uint8_t instance, uint8_t report_id, uint8_t report_type, uint16_t len)
{
    if (get_report_bridge.busy &&
        get_report_bridge.host_dev_addr == dev_addr &&
        get_report_bridge.host_instance == instance &&
        get_report_bridge.report_id == report_id &&
        get_report_bridge.report_type == report_type) {
        get_report_bridge.actual_len = len;
        __dmb();
        get_report_bridge.busy = false;
        get_report_bridge.done = true;
        return;
    }

}

void tuh_hid_set_report_complete_cb(uint8_t dev_addr, uint8_t instance, uint8_t report_id, uint8_t report_type, uint16_t len)
{
    (void)dev_addr;
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)len;
}

// HID device callbacks with improved validation
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    if (instance >= MAX_DEVICE_HID_INTERFACES || !buffer || reqlen == 0) return 0;

    // Real passthrough for vendor/feature/input GET_REPORT. Logitech G Hub,
    // Razer Synapse, etc. use this for identity, battery, DPI and profile
    // handshakes; cached zeros make Windows software reject the device.
    if (instance < mirrored_itf_count && mirrored_itfs[instance].active &&
        !get_report_bridge.pending && !get_report_bridge.busy) {
        mirrored_interface_t *mitf = &mirrored_itfs[instance];

        uint16_t request_len = reqlen;
        if (report_id != 0 && request_len < VENDOR_REPORT_MAX_LEN) {
            request_len++;
        }
        if (request_len > VENDOR_REPORT_MAX_LEN) request_len = VENDOR_REPORT_MAX_LEN;

        get_report_bridge.host_dev_addr = mitf->host_dev_addr;
        get_report_bridge.host_instance = mitf->host_instance;
        get_report_bridge.report_id = report_id;
        get_report_bridge.report_type = (uint8_t)report_type;
        get_report_bridge.request_len = request_len;
        get_report_bridge.actual_len = 0;
        get_report_bridge.done = false;
        get_report_bridge.pending = true;
        __dmb();

        uint32_t start_us = time_us_32();
        while (!get_report_bridge.done &&
               (uint32_t)(time_us_32() - start_us) < PASSTHROUGH_GET_TIMEOUT_US) {
            watchdog_core0_heartbeat();
            sleep_us(100);
        }

        if (get_report_bridge.done && get_report_bridge.actual_len > 0) {
            uint16_t actual_len = get_report_bridge.actual_len;
            uint16_t src_offset = 0;
            if (report_id != 0 && actual_len > 0 && get_report_bridge.data[0] == report_id) {
                src_offset = 1;
            }

            if (actual_len > src_offset) {
                uint16_t available = actual_len - src_offset;
                uint16_t copy_len = (available < reqlen) ? available : reqlen;
                memcpy(buffer, &get_report_bridge.data[src_offset], copy_len);
                cache_report(instance, report_id, (uint8_t)report_type, buffer, (uint8_t)copy_len);
                get_report_bridge.done = false;
                return copy_len;
            }
        }

        get_report_bridge.done = false;
    }

    // Look up cached report from the real device
    cached_report_t *slots = report_cache[instance];
    __dmb();  // Ensure we see Core1's latest cache writes
    for (int i = 0; i < REPORT_CACHE_SLOTS_PER_ITF; i++) {
        if (slots[i].valid && slots[i].report_id == report_id && slots[i].report_type == (uint8_t)report_type) {
            uint16_t copy_len = (slots[i].len < reqlen) ? slots[i].len : reqlen;
            memcpy(buffer, slots[i].data, copy_len);
            return copy_len;
        }
    }

    // No cached data yet — return zeros so the host sees the device as responsive.
    // macOS IOKit sends GET_REPORT during device open; returning 0 (no data) causes
    // "open failed". Returning a zeroed buffer keeps the open path happy.
    uint16_t fill_len = (reqlen > VENDOR_REPORT_MAX_LEN) ? VENDOR_REPORT_MAX_LEN : reqlen;
    memset(buffer, 0, fill_len);
    return fill_len;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, const uint8_t *buffer, uint16_t bufsize)
{
    // Handle keyboard LED output reports (caps lock, etc.)
    if (report_type == HID_REPORT_TYPE_OUTPUT && report_id == runtime_kbd_report_id)
    {
        if (buffer == NULL || bufsize < MIN_BUFFER_SIZE) return;

        uint8_t const kbd_leds = buffer[0];
        bool new_caps_state = (kbd_leds & KEYBOARD_LED_CAPSLOCK) != 0;

        if (new_caps_state != caps_lock_state)
        {
            caps_lock_state = new_caps_state;
            neopixel_trigger_caps_lock_flash();
        }
        return;
    }

    // Forward all other SET_REPORT requests to the real mouse for vendor passthrough.
    // This enables Logitech G Hub, Razer Synapse, etc. to configure DPI, lighting,
    // macros, battery queries through the proxy.
    if (buffer != NULL && bufsize > 0 && instance < mirrored_itf_count && mirrored_itfs[instance].active) {
        mirrored_interface_t *mitf = &mirrored_itfs[instance];

        uint8_t next_head = (set_report_queue.head + 1) & VENDOR_QUEUE_MASK;
        if (next_head != set_report_queue.tail) {
            set_report_entry_t *e = &set_report_queue.entries[set_report_queue.head];
            e->host_dev_addr = mitf->host_dev_addr;
            e->host_instance = mitf->host_instance;
            e->report_id = report_id;
            e->report_type = (uint8_t)report_type;
            if (report_id != 0) {
                e->data[0] = report_id;
                uint8_t copy_len = (bufsize >= (VENDOR_REPORT_MAX_LEN - 1)) ? (VENDOR_REPORT_MAX_LEN - 1) : (uint8_t)bufsize;
                memcpy(&e->data[1], buffer, copy_len);
                e->len = (uint8_t)(copy_len + 1);
            } else {
                e->len = (bufsize > VENDOR_REPORT_MAX_LEN) ? VENDOR_REPORT_MAX_LEN : (uint8_t)bufsize;
                memcpy(e->data, buffer, e->len);
            }
            cache_report(instance, report_id, (uint8_t)report_type, buffer,
                         (bufsize > VENDOR_REPORT_MAX_LEN) ? VENDOR_REPORT_MAX_LEN : (uint8_t)bufsize);
            __dmb();
            set_report_queue.head = next_head;
        }
    }
}

void tud_hid_report_complete_cb(uint8_t instance, const uint8_t *report, uint16_t len)
{
    (void)instance;
    (void)len;
    (void)report;
}

bool usb_device_stack_reset(void)
{
    neopixel_trigger_usb_reset_pending();

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());

    // Only reset if device stack was previously initialized
    if (!usb_device_initialized)
    {
        return true;
    }

    // Mark as successful to prevent repeated attempts
    usb_error_tracker.device_errors = 0;
    usb_error_tracker.consecutive_device_errors = 0;
    usb_error_tracker.device_error_state = false;

    const uint32_t elapsed_time = to_ms_since_boot(get_absolute_time()) - start_time;
    (void)elapsed_time; // suppressed timing log

    return true; // Return success to prevent repeated attempts
}

bool usb_host_stack_reset(void)
{
#if PIO_USB_AVAILABLE
    neopixel_trigger_usb_reset_pending();

    const uint32_t start_time = to_ms_since_boot(get_absolute_time());

    // Only reset if host stack was previously initialized
    if (!usb_host_initialized)
    {
        return true;
    }

    // Reset connection tracking without reinitializing stacks
    memset(&connection_state, 0, sizeof(connection_state));

    // Mark as successful to prevent repeated attempts
    usb_error_tracker.host_errors = 0;
    usb_error_tracker.consecutive_host_errors = 0;
    usb_error_tracker.host_error_state = false;

    const uint32_t elapsed_time = to_ms_since_boot(get_absolute_time()) - start_time;
    (void)elapsed_time; // suppressed timing log

    return true; // Return success to prevent repeated attempts
#else
    (void)0; // suppressed log
    return false;
#endif
}

bool usb_stacks_reset(void)
{
    neopixel_trigger_usb_reset_pending();

    const bool device_success = usb_device_stack_reset();
    const bool host_success = usb_host_stack_reset();
    const bool overall_success = device_success && host_success;

    if (overall_success)
    {
        neopixel_trigger_usb_reset_success();
    }
    else
    {
        neopixel_trigger_usb_reset_failed();
    }

    return overall_success;
}

void usb_stack_error_check(void)
{
    const uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Only check errors at specified intervals
    if (current_time - usb_error_tracker.last_error_check_time < USB_ERROR_CHECK_INTERVAL_MS)
    {
        return;
    }
    usb_error_tracker.last_error_check_time = current_time;

    const bool device_healthy = tud_ready(); // Remove mount requirement as device can be functional without being mounted
    if (!device_healthy)
    {
        usb_error_tracker.consecutive_device_errors++;
    }
    else
    {
        usb_error_tracker.consecutive_device_errors = 0;
        usb_error_tracker.device_error_state = false;
    }

    // Trigger reset if error threshold is exceeded
    if (usb_error_tracker.consecutive_device_errors >= USB_STACK_ERROR_THRESHOLD)
    {
        if (!usb_error_tracker.device_error_state)
        {
            usb_error_tracker.device_error_state = true;
        }
    }

#if PIO_USB_AVAILABLE
    if (usb_error_tracker.consecutive_host_errors >= USB_STACK_ERROR_THRESHOLD)
    {
        if (!usb_error_tracker.host_error_state)
        {
            usb_error_tracker.host_error_state = true;
        }
    }
#endif
}

// Device Descriptors
// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor — clones host mouse fields when available
uint8_t const * tud_descriptor_device_cb(void)
{
    // Xbox mode: return Xbox controller device descriptor
    if (g_xbox_mode) {
        return xbox_get_device_descriptor();
    }

    static tusb_desc_device_t desc_device;

    desc_device = (tusb_desc_device_t) {
        .bLength            = sizeof(tusb_desc_device_t),
        .bDescriptorType    = TUSB_DESC_DEVICE,
        // Clone ALL device descriptor fields from the host mouse to look identical
        .bcdUSB             = host_device_info.valid ? host_device_info.bcdUSB : 0x0200,
        .bDeviceClass       = host_device_info.valid ? host_device_info.bDeviceClass : 0x00,
        .bDeviceSubClass    = host_device_info.valid ? host_device_info.bDeviceSubClass : 0x00,
        .bDeviceProtocol    = host_device_info.valid ? host_device_info.bDeviceProtocol : 0x00,
        .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,  // Must match our controller's actual EP0 size

        // Clone VID/PID from host device
        .idVendor           = (get_attached_vid() != 0) ? get_attached_vid() : USB_VENDOR_ID,
        .idProduct          = (get_attached_pid() != 0) ? get_attached_pid() : USB_PRODUCT_ID,
        // Clone device revision from host
        .bcdDevice          = host_device_info.valid ? host_device_info.bcdDevice : 0x0100,

        .iManufacturer      = 0x01,
        .iProduct           = 0x02,
        .iSerialNumber      = attached_has_serial ? 0x03 : 0x00,

        .bNumConfigurations = 0x01
    };

    return (uint8_t const *)&desc_device;
}

// HID Report Descriptor — per-instance for multi-interface mirroring.
// Instance 0 is typically the mouse interface (composite: keyboard + mouse + consumer).
// Other instances return verbatim cloned descriptors from the host device.
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // Multi-interface mode: return the correct descriptor for each instance
    if (mirrored_itf_count > 0 && instance < mirrored_itf_count && mirrored_itfs[instance].active) {
        if (mirrored_itfs[instance].is_mouse) {
            // Mouse interface returns composite descriptor (keyboard + mouse + consumer)
            return desc_hid_runtime_valid ? desc_hid_report_runtime : desc_hid_report;
        }
        // Non-mouse interface returns verbatim host descriptor
        if (mirrored_itfs[instance].report_desc_len > 0) {
            return mirrored_itfs[instance].report_desc;
        }
    }

    // Single-interface fallback
    if (instance == 0 && desc_hid_runtime_valid) {
        return desc_hid_report_runtime;
    }
    return desc_hid_report;
}

// Configuration Descriptor - now dynamic for cloning
enum
{
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

// Helper: write one HID interface block (interface desc + HID desc + endpoint(s))
// into a buffer.  Returns number of bytes written.
static uint16_t write_hid_interface_desc(uint8_t *buf, uint16_t buf_max,
                                          uint8_t itf_num, uint8_t subclass,
                                          uint8_t protocol, uint16_t report_desc_len,
                                          uint8_t ep_in_addr, uint16_t ep_in_size,
                                          uint8_t ep_in_interval,
                                          bool has_ep_out, uint8_t ep_out_interval)
{
    uint16_t pos = 0;
    uint8_t num_eps = has_ep_out ? 2 : 1;

    // Interface descriptor (9 bytes)
    if (pos + 9 > buf_max) return 0;
    buf[pos++] = 9;
    buf[pos++] = TUSB_DESC_INTERFACE;
    buf[pos++] = itf_num;
    buf[pos++] = 0;              // bAlternateSetting
    buf[pos++] = num_eps;
    buf[pos++] = TUSB_CLASS_HID;
    buf[pos++] = subclass;
    buf[pos++] = protocol;
    buf[pos++] = 0;              // iInterface

    // HID descriptor (9 bytes)
    if (pos + 9 > buf_max) return 0;
    buf[pos++] = 9;
    buf[pos++] = HID_DESC_TYPE_HID;
    buf[pos++] = 0x11;           // bcdHID low (1.11)
    buf[pos++] = 0x01;           // bcdHID high
    buf[pos++] = 0;              // bCountryCode
    buf[pos++] = 1;              // bNumDescriptors
    buf[pos++] = HID_DESC_TYPE_REPORT;
    buf[pos++] = (uint8_t)(report_desc_len & 0xFF);
    buf[pos++] = (uint8_t)((report_desc_len >> 8) & 0xFF);

    // IN endpoint descriptor (7 bytes)
    if (pos + 7 > buf_max) return 0;
    buf[pos++] = 7;
    buf[pos++] = TUSB_DESC_ENDPOINT;
    buf[pos++] = ep_in_addr;
    buf[pos++] = TUSB_XFER_INTERRUPT;
    buf[pos++] = (uint8_t)(ep_in_size & 0xFF);
    buf[pos++] = (uint8_t)((ep_in_size >> 8) & 0xFF);
    buf[pos++] = ep_in_interval;

    // OUT endpoint descriptor (7 bytes, optional)
    if (has_ep_out) {
        if (pos + 7 > buf_max) return 0;
        buf[pos++] = 7;
        buf[pos++] = TUSB_DESC_ENDPOINT;
        buf[pos++] = ep_in_addr & 0x0F;  // Same EP number, OUT direction
        buf[pos++] = TUSB_XFER_INTERRUPT;
        buf[pos++] = (uint8_t)(ep_in_size & 0xFF);
        buf[pos++] = (uint8_t)((ep_in_size >> 8) & 0xFF);
        buf[pos++] = ep_out_interval;
    }

    return pos;
}

// Build the configuration descriptor from current runtime state.
// Supports 1-4 HID interfaces mirroring the host device's layout.
static void rebuild_configuration_descriptor(void) {
    uint8_t num_itfs = (mirrored_itf_count > 0) ? mirrored_itf_count : 1;
    if (num_itfs > MAX_DEVICE_HID_INTERFACES) num_itfs = MAX_DEVICE_HID_INTERFACES;

    uint8_t cfg_attributes = TU_BIT(7) | (host_config_info.valid ? host_config_info.bmAttributes : TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP);
    uint8_t cfg_max_power = host_config_info.valid ? host_config_info.bMaxPower : (USB_CONFIG_POWER_MA / 2);

    // Leave 9 bytes for config header, fill interfaces after
    uint16_t pos = 9;

    for (uint8_t i = 0; i < num_itfs; i++) {
        uint16_t report_len;
        uint8_t subclass, protocol, ep_interval;
        bool has_out = false;
        uint8_t ep_out_interval_val = 1;

        if (mirrored_itf_count > 0 && i < mirrored_itf_count && mirrored_itfs[i].active) {
            mirrored_interface_t *itf = &mirrored_itfs[i];
            if (itf->is_mouse) {
                report_len = desc_hid_runtime_valid ? desc_hid_runtime_len : sizeof(desc_hid_report);
            } else {
                report_len = itf->report_desc_len;
            }
            subclass = itf->itf_subclass;
            protocol = itf->itf_protocol;
            ep_interval = itf->ep_in_interval ? itf->ep_in_interval : HID_POLLING_INTERVAL_MS;
            has_out = itf->has_ep_out;
            ep_out_interval_val = itf->ep_out_interval ? itf->ep_out_interval : ep_interval;
        } else {
            // Default single-interface fallback
            report_len = desc_hid_runtime_valid ? desc_hid_runtime_len : sizeof(desc_hid_report);
            subclass = host_config_info.valid ? host_config_info.bInterfaceSubClass : 0;
            protocol = host_config_info.valid ? host_config_info.bInterfaceProtocol : HID_ITF_PROTOCOL_NONE;
            ep_interval = host_config_info.valid ? host_config_info.bInterval : HID_POLLING_INTERVAL_MS;
        }

        // CRITICAL: wMaxPacketSize must be our buffer size, not the host's.
        // The downstream PC uses this to allocate receive buffers.
        uint16_t written = write_hid_interface_desc(
            &desc_configuration_runtime[pos], DESC_CONFIG_RUNTIME_MAX - pos,
            i,                         // bInterfaceNumber
            subclass, protocol,
            report_len,
            0x81 + i,                  // EP IN address: 0x81, 0x82, 0x83, 0x84
            CFG_TUD_HID_EP_BUFSIZE,
            ep_interval,
            has_out, ep_out_interval_val
        );
        if (written == 0) break;  // Buffer full
        pos += written;
    }

    // Fill config descriptor header (first 9 bytes)
    desc_configuration_runtime[0] = 9;                          // bLength
    desc_configuration_runtime[1] = TUSB_DESC_CONFIGURATION;    // bDescriptorType
    desc_configuration_runtime[2] = (uint8_t)(pos & 0xFF);      // wTotalLength low
    desc_configuration_runtime[3] = (uint8_t)((pos >> 8) & 0xFF); // wTotalLength high
    desc_configuration_runtime[4] = num_itfs;                   // bNumInterfaces
    desc_configuration_runtime[5] = 1;                          // bConfigurationValue
    desc_configuration_runtime[6] = 0;                          // iConfiguration
    desc_configuration_runtime[7] = cfg_attributes;             // bmAttributes
    desc_configuration_runtime[8] = cfg_max_power;              // bMaxPower

    desc_config_runtime_valid = true;
}

// Parse host configuration descriptor to extract ALL HID interfaces and their endpoints.
// Populates mirrored_itfs[] with per-interface endpoint configs, and tracks string indices.
static void parse_host_config_descriptor(const uint8_t *cfg_desc, uint16_t cfg_len) {
    if (!cfg_desc || cfg_len < TUD_CONFIG_DESC_LEN) return;

    // Extract config-level fields
    host_config_info.bmAttributes = cfg_desc[7] & 0x7F; // Mask off reserved bit 7 (we add it back)
    host_config_info.bMaxPower = cfg_desc[8];

    // Reset per-interface state
    expected_hid_itf_count = 0;

    // Walk the descriptor chain to find ALL HID interfaces + their endpoints
    uint16_t offset = 0;
    int current_hid_idx = -1;  // Index into mirrored_itfs for current HID interface

    while (offset + 1 < cfg_len) {
        uint8_t desc_length = cfg_desc[offset];
        uint8_t desc_type = cfg_desc[offset + 1];

        if (desc_length == 0) break; // Prevent infinite loop on malformed descriptors

        // Interface descriptor
        if (desc_type == TUSB_DESC_INTERFACE && desc_length >= 9 && offset + 8 < cfg_len) {
            uint8_t itf_class = cfg_desc[offset + 5];
            if (itf_class == TUSB_CLASS_HID && expected_hid_itf_count < MAX_DEVICE_HID_INTERFACES) {
                current_hid_idx = expected_hid_itf_count;
                mirrored_itfs[current_hid_idx].itf_subclass = cfg_desc[offset + 6];
                mirrored_itfs[current_hid_idx].itf_protocol = cfg_desc[offset + 7];
                mirrored_itfs[current_hid_idx].has_ep_out = false;

                // Track interface string index for faithful string mirroring
                uint8_t iInterface = cfg_desc[offset + 8];
                if (iInterface > 0 && iInterface > max_string_index_seen)
                    max_string_index_seen = iInterface;

                // Back-compat: populate legacy host_config_info from first HID interface
                if (expected_hid_itf_count == 0) {
                    host_config_info.bInterfaceSubClass = cfg_desc[offset + 6];
                    host_config_info.bInterfaceProtocol = cfg_desc[offset + 7];
                }

                expected_hid_itf_count++;
            } else {
                current_hid_idx = -1; // Non-HID interface, ignore endpoints
            }
        }

        // Endpoint descriptor (after a HID interface)
        if (current_hid_idx >= 0 && desc_type == TUSB_DESC_ENDPOINT && desc_length >= 7 && offset + 6 < cfg_len) {
            uint8_t ep_addr = cfg_desc[offset + 2];
            uint8_t ep_attr = cfg_desc[offset + 3];
            uint16_t ep_size = cfg_desc[offset + 4] | (cfg_desc[offset + 5] << 8);
            uint8_t ep_interval = cfg_desc[offset + 6];

            if ((ep_attr & 0x03) == TUSB_XFER_INTERRUPT) {
                if (ep_addr & 0x80) {
                    // IN endpoint
                    mirrored_itfs[current_hid_idx].ep_in_max_packet = ep_size;
                    mirrored_itfs[current_hid_idx].ep_in_interval = ep_interval;

                    // Back-compat: populate legacy host_config_info from first IN endpoint
                    if (!host_config_info.valid) {
                        host_config_info.wMaxPacketSize = ep_size;
                        host_config_info.bInterval = ep_interval;
                        host_config_info.valid = true;
                    }
                } else {
                    // OUT endpoint
                    mirrored_itfs[current_hid_idx].has_ep_out = true;
                    mirrored_itfs[current_hid_idx].ep_out_max_packet = ep_size;
                    mirrored_itfs[current_hid_idx].ep_out_interval = ep_interval;
                }
            }
        }

        offset += desc_length;
    }

    if (expected_hid_itf_count == 0) {
        expected_hid_itf_count = 1; // At least 1 interface expected
    }
}

// Static fallback (used only for initial sizeof reference)
uint8_t const desc_configuration[] =
    {
        // Config number, interface count, string index, total length, attribute, power in mA
        TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, USB_CONFIG_POWER_MA),

        // Interface number, string index, protocol, report descriptor len, EP In address, size & polling interval
        TUD_HID_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, HID_POLLING_INTERVAL_MS)};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Returns the runtime (cloned) config descriptor if available
uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index; // for multiple configurations

    // Xbox mode: return Xbox controller configuration descriptor
    if (g_xbox_mode) {
        return xbox_get_config_descriptor();
    }

    if (desc_config_runtime_valid) {
        return desc_configuration_runtime;
    }
    return desc_configuration;
}

// String Descriptors
char const *string_desc_arr[] =
    {
        (const char[]){USB_LANGUAGE_ENGLISH_US_BYTE1, USB_LANGUAGE_ENGLISH_US_BYTE2}, // 0: is supported language is English (0x0409)
        MANUFACTURER_STRING,                                                          // 1: Manufacturer
        PRODUCT_STRING,                                                               // 2: Product
        "PIOKMbox_fallback"                                                           // 3: Fallback serial (unused - dynamic serial used instead)
};

static uint16_t _desc_str[MAX_STRING_DESCRIPTOR_CHARS + 1];

// Convert ASCII string into UTF-16
static uint8_t convert_string_to_utf16(const char *str, uint16_t *desc_str)
{
    if (str == NULL || desc_str == NULL)
    {
        return 0;
    }

    // Cap at max char
    uint8_t chr_count = strlen(str);
    if (chr_count > MAX_STRING_DESCRIPTOR_CHARS)
    {
        chr_count = MAX_STRING_DESCRIPTOR_CHARS;
    }

    // Convert ASCII string into UTF-16
    for (uint8_t i = 0; i < chr_count; i++)
    {
        desc_str[STRING_DESC_FIRST_CHAR_OFFSET + i] = str[i];
    }

    return chr_count;
}

// Invoked when received GET STRING DESCRIPTOR request
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    // Xbox mode: return Xbox controller string descriptors
    if (g_xbox_mode) {
        return xbox_get_string_descriptor(index, langid);
    }

    (void)langid;

    uint8_t chr_count;

    if (index == BUFFER_FIRST_ELEMENT_INDEX)
    {
        memcpy(&_desc_str[STRING_DESC_FIRST_CHAR_OFFSET],
               string_desc_arr[BUFFER_FIRST_ELEMENT_INDEX],
               STRING_DESC_HEADER_SIZE);
        chr_count = STRING_DESC_CHAR_COUNT_INIT;
    }
    else
    {
        const char *str = NULL;

        // Standard indices 1-3: manufacturer, product, serial
        if (string_descriptors_fetched) {
            switch (index) {
                case STRING_DESC_MANUFACTURER_IDX:
                    str = attached_manufacturer;
                    break;
                case STRING_DESC_PRODUCT_IDX:
                    str = attached_product;
                    break;
                case STRING_DESC_SERIAL_IDX:
                    if (attached_has_serial && strlen(attached_serial) > 0) {
                        str = attached_serial;
                    } else {
                        str = get_dynamic_serial_string();
                    }
                    break;
                default:
                    break;
            }
        }

        // Check extra string cache for higher indices (interface strings, etc.)
        if (str == NULL && index > STRING_DESC_SERIAL_IDX) {
            for (uint8_t i = 0; i < extra_string_count; i++) {
                if (extra_strings[i].valid && extra_strings[i].index == index) {
                    str = extra_strings[i].str;
                    break;
                }
            }
        }

        // Fallback to static array or defaults
        if (str == NULL) {
            if (index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
                str = string_desc_arr[index];
            }
        }

        // Final fallback for serial
        if (str == NULL && index == STRING_DESC_SERIAL_IDX) {
            str = get_dynamic_serial_string();
        }

        if (str == NULL) return NULL;

        chr_count = convert_string_to_utf16(str, _desc_str);
    }

    // first byte is length (including header), second byte is string type
    _desc_str[BUFFER_FIRST_ELEMENT_INDEX] = (TUSB_DESC_STRING << STRING_DESC_TYPE_SHIFT) |
                                            (STRING_DESC_LENGTH_MULTIPLIER * chr_count + STRING_DESC_HEADER_SIZE);

    return _desc_str;
}
