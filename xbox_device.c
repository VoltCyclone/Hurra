/*
 * Xbox Controller Device Class
 *
 * TinyUSB custom device class presenting as an Xbox Wireless Controller.
 * Runs on Core 0. Handles GIP announce, drains gamepad accumulator,
 * merges physical + injection inputs, and proxies auth from console
 * to the real controller via cross-core queues.
 */

#include "xbox_device.h"
#include "xbox_host.h"
#include "xbox_gip.h"
#include "tusb.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"
#include "led_control.h"
#include "defines.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include <string.h>

//--------------------------------------------------------------------+
// Xbox Device Descriptors
//--------------------------------------------------------------------+

static const tusb_desc_device_t xbox_device_desc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0xFF,    // Vendor specific
    .bDeviceSubClass    = 0x47,    // GIP
    .bDeviceProtocol    = 0xD0,    // GIP protocol
    .bMaxPacketSize0    = 64,
    .idVendor           = XBOX_VID,
    .idProduct          = XBOX_PID,
    .bcdDevice          = 0x0508,  // Firmware version 5.8
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

// Configuration descriptor with vendor-specific interface and interrupt endpoints
#define XBOX_CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 9 + 7 + 7) // config + interface + EP_IN + EP_OUT

static const uint8_t xbox_config_desc[] = {
    // Configuration descriptor
    9, TUSB_DESC_CONFIGURATION,
    U16_TO_U8S_LE(XBOX_CONFIG_TOTAL_LEN), // wTotalLength
    1,                                      // bNumInterfaces
    1,                                      // bConfigurationValue
    0,                                      // iConfiguration
    0x80 | TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, // bmAttributes
    250,                                    // bMaxPower (500mA)

    // Interface descriptor - vendor specific GIP
    9, TUSB_DESC_INTERFACE,
    0,                    // bInterfaceNumber
    0,                    // bAlternateSetting
    2,                    // bNumEndpoints
    XBOX_ITF_CLASS,       // bInterfaceClass (0xFF)
    XBOX_ITF_SUBCLASS,    // bInterfaceSubClass (0x47)
    XBOX_ITF_PROTOCOL,    // bInterfaceProtocol (0xD0)
    0,                    // iInterface

    // Endpoint IN (interrupt, 64 bytes, 4ms)
    7, TUSB_DESC_ENDPOINT,
    0x81,                 // bEndpointAddress (IN, EP1)
    TUSB_XFER_INTERRUPT,  // bmAttributes
    U16_TO_U8S_LE(64),   // wMaxPacketSize
    4,                    // bInterval (4ms)

    // Endpoint OUT (interrupt, 64 bytes, 4ms)
    7, TUSB_DESC_ENDPOINT,
    0x01,                 // bEndpointAddress (OUT, EP1)
    TUSB_XFER_INTERRUPT,  // bmAttributes
    U16_TO_U8S_LE(64),   // wMaxPacketSize
    8,                    // bInterval (8ms)
};

_Static_assert(sizeof(xbox_config_desc) == XBOX_CONFIG_TOTAL_LEN,
               "Xbox config descriptor size mismatch");

// String descriptors
static const char *xbox_string_desc[] = {
    "\x09\x04",                          // 0: Language (English US)
    "Microsoft",                          // 1: Manufacturer
    "Xbox Wireless Controller",           // 2: Product
    "3039373231303836383232363731",        // 3: Serial number
};

//--------------------------------------------------------------------+
// Private State
//--------------------------------------------------------------------+

static uint8_t  xbox_dev_ep_in  = 0;
static uint8_t  xbox_dev_ep_out = 0;
static bool     xbox_dev_mounted = false;
static bool     announce_sent = false;

// OUT endpoint receive buffer
static uint8_t  ep_out_buf[GIP_MAX_PACKET_SIZE];
static uint8_t proxy_in_buf[GIP_MAX_PACKET_SIZE];
static uint8_t pending_in_len;

// Cached merged report for sending
static uint8_t  report_buf[GIP_HEADER_SIZE + sizeof(xbox_input_report_t)];

// String descriptor buffer
static uint16_t xbox_desc_str[32 + 1];

//--------------------------------------------------------------------+
// TinyUSB Custom Device Driver Callbacks
//--------------------------------------------------------------------+

static void xbox_device_driver_init(void) {
    xbox_dev_mounted = false;
    announce_sent = false;
    pending_in_len = 0;
}

static bool xbox_device_driver_deinit(void) {
    xbox_dev_mounted = false;
    announce_sent = false;
    pending_in_len = 0;
    xbox_dev_ep_in = 0;
    xbox_dev_ep_out = 0;
    return true;
}

static void xbox_device_driver_reset(uint8_t rhport) {
    (void)rhport;
    xbox_dev_mounted = false;
    announce_sent = false;
    pending_in_len = 0;
    xbox_dev_ep_in = 0;
    xbox_dev_ep_out = 0;
}

static uint16_t xbox_device_driver_open(uint8_t rhport,
                                         tusb_desc_interface_t const *desc_itf,
                                         uint16_t max_len) {
    (void)rhport;

    // Only claim our vendor-specific interface
    if (desc_itf->bInterfaceClass    != XBOX_ITF_CLASS ||
        desc_itf->bInterfaceSubClass != XBOX_ITF_SUBCLASS ||
        desc_itf->bInterfaceProtocol != XBOX_ITF_PROTOCOL) {
        return 0;
    }

    uint16_t drv_len = sizeof(tusb_desc_interface_t);
    uint8_t const *p_desc = (uint8_t const *)desc_itf + drv_len;

    // Open endpoints
    for (uint8_t i = 0; i < desc_itf->bNumEndpoints; i++) {
        if (drv_len >= max_len) break;

        tusb_desc_endpoint_t const *ep_desc = (tusb_desc_endpoint_t const *)p_desc;
        if (ep_desc->bDescriptorType != TUSB_DESC_ENDPOINT) break;

        usbd_edpt_open(rhport, ep_desc);

        if (tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_IN) {
            xbox_dev_ep_in = ep_desc->bEndpointAddress;
        } else {
            xbox_dev_ep_out = ep_desc->bEndpointAddress;
        }

        drv_len += tu_desc_len(p_desc);
        p_desc = tu_desc_next(p_desc);
    }

    xbox_dev_mounted = true;

    // Prepare to receive data on OUT endpoint
    if (xbox_dev_ep_out) {
        usbd_edpt_xfer(rhport, xbox_dev_ep_out, ep_out_buf, sizeof(ep_out_buf));
    }

    return drv_len;
}

static bool __not_in_flash_func(xbox_device_driver_xfer_cb)(uint8_t rhport, uint8_t ep_addr,
                                        xfer_result_t result, uint32_t xferred_bytes) {
    if (ep_addr == xbox_dev_ep_out) {
        if (result == XFER_RESULT_SUCCESS && xferred_bytes >= GIP_HEADER_SIZE) {
            // All console->controller packets get queued unconditionally
            gip_queue_push(xbox_host_get_d2h_queue(), ep_out_buf, (uint8_t)xferred_bytes);
        }
        // Always re-submit OUT endpoint for next packet
        usbd_edpt_xfer(rhport, xbox_dev_ep_out, ep_out_buf, sizeof(ep_out_buf));
    }
    // IN endpoint completion: no action needed, xbox_device_task sends when ready
    return true;
}

static bool xbox_device_driver_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                                tusb_control_request_t const *request) {
    (void)rhport;
    (void)stage;
    (void)request;
    // Xbox controllers don't use standard control transfers for data
    // Return false to let TinyUSB handle standard requests
    return false;
}

//--------------------------------------------------------------------+
// Register Custom Device Driver
//--------------------------------------------------------------------+

static const usbd_class_driver_t xbox_device_driver = {
    .name            = "XBOX",
    .init            = xbox_device_driver_init,
    .deinit          = xbox_device_driver_deinit,
    .reset           = xbox_device_driver_reset,
    .open            = xbox_device_driver_open,
    .control_xfer_cb = xbox_device_driver_control_xfer_cb,
    .xfer_cb         = xbox_device_driver_xfer_cb,
    .sof             = NULL,
};

// TinyUSB calls this weak function to get application-level device drivers
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &xbox_device_driver;
}

//--------------------------------------------------------------------+
// GIP Announce Sequence
//--------------------------------------------------------------------+

static void xbox_send_announce(void) {
    if (!xbox_dev_mounted || !xbox_dev_ep_in) return;

    // GIP announce: command=0x02, flags=0x20, seq=0, length=varies
    // This tells the console "I am an Xbox controller"
    static uint8_t announce_pkt[] = {
        GIP_CMD_ANNOUNCE, GIP_FLAG_INTERNAL, 0x00, 0x09,
        // Payload: device info (simplified announce)
        0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    if (usbd_edpt_busy(0, xbox_dev_ep_in)) return;
    announce_sent = usbd_edpt_xfer(0, xbox_dev_ep_in, announce_pkt, sizeof(announce_pkt));
}

//--------------------------------------------------------------------+
// Input Report Merge & Send
//--------------------------------------------------------------------+

static void __not_in_flash_func(xbox_build_and_send_report)(void) {
    if (!xbox_dev_mounted || !xbox_dev_ep_in) return;
    if (usbd_edpt_busy(0, xbox_dev_ep_in)) return;

    xbox_gamepad_state_t *gs = xbox_host_get_gamepad();
    xbox_input_report_t merged;

    // Snapshot everything under spinlock — minimize hold time
    uint32_t save = spin_lock_blocking(gs->lock);
    merged = gs->physical;
    bool updated = gs->physical_updated;
    gs->physical_updated = false;
    uint8_t mask = gs->inject_mask;
    uint16_t inj_buttons = gs->inject_buttons;
    uint16_t inj_tl = gs->inject_trigger_left;
    uint16_t inj_tr = gs->inject_trigger_right;
    int16_t inj_lx = gs->inject_stick_lx;
    int16_t inj_ly = gs->inject_stick_ly;
    int16_t inj_rx = gs->inject_stick_rx;
    int16_t inj_ry = gs->inject_stick_ry;
    uint8_t seq = gs->output_sequence++;
    spin_unlock(gs->lock, save);

    // Only send if there's new data or active injection
    if (!updated && mask == 0) return;

    // Apply injection overrides (outside spinlock — uses local copies)
    if (mask & XBOX_INJECT_BUTTONS) {
        merged.buttons |= inj_buttons;  // OR (additive)
    }
    if (mask & XBOX_INJECT_TRIGGERS) {
        merged.trigger_left = inj_tl;
        merged.trigger_right = inj_tr;
    }
    if (mask & XBOX_INJECT_STICK_L) {
        merged.stick_lx = inj_lx;
        merged.stick_ly = inj_ly;
    }
    if (mask & XBOX_INJECT_STICK_R) {
        merged.stick_rx = inj_rx;
        merged.stick_ry = inj_ry;
    }

    // Build GIP input report packet
    gip_header_t *hdr = (gip_header_t *)report_buf;
    hdr->command  = GIP_CMD_INPUT;
    hdr->client   = 0x20;  // Internal flag + client 0
    hdr->sequence = seq;
    hdr->length   = sizeof(xbox_input_report_t);

    memcpy(report_buf + GIP_HEADER_SIZE, &merged, sizeof(xbox_input_report_t));

    usbd_edpt_xfer(0, xbox_dev_ep_in, report_buf, GIP_INPUT_REPORT_SIZE);
}

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

void xbox_device_init(void) {
    // Initialization handled by driver callback
}

void __not_in_flash_func(xbox_device_task)(void) {
    if (!g_xbox_mode || !xbox_dev_mounted) return;

    // Send announce on first enumeration
    if (!announce_sent) {
        xbox_send_announce();
        return; // Wait for announce to complete before sending reports
    }

    // Forward one packet per call from host_to_device_queue to console.
    // Only pop when the endpoint is free to avoid dropping packets.
    gip_packet_queue_t *h2d = xbox_host_get_h2d_queue();
    if (xbox_dev_ep_in && !usbd_edpt_busy(0, xbox_dev_ep_in)) {
        if (!pending_in_len && !gip_queue_empty(h2d)) {
            gip_queue_pop(h2d, proxy_in_buf, &pending_in_len);
        }
        if (pending_in_len) {
            if (usbd_edpt_xfer(0, xbox_dev_ep_in, proxy_in_buf, pending_in_len)) {
                pending_in_len = 0;
            }
            return;
        }
    }

    // Build and send input reports
    xbox_build_and_send_report();
}

uint8_t const *xbox_get_device_descriptor(void) {
    return (uint8_t const *)&xbox_device_desc;
}

uint8_t const *xbox_get_config_descriptor(void) {
    return xbox_config_desc;
}

uint16_t const *xbox_get_string_descriptor(uint8_t index, uint16_t langid) {
    (void)langid;

    if (index == 0) {
        xbox_desc_str[1] = 0x0409; // English US
        xbox_desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | 4);
        return xbox_desc_str;
    }

    if (index >= sizeof(xbox_string_desc) / sizeof(xbox_string_desc[0])) {
        return NULL;
    }

    const char *str = xbox_string_desc[index];
    uint8_t chr_count = strlen(str);
    if (chr_count > 31) chr_count = 31;

    for (uint8_t i = 0; i < chr_count; i++) {
        xbox_desc_str[1 + i] = str[i];
    }

    xbox_desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (chr_count * 2 + 2));
    return xbox_desc_str;
}

//--------------------------------------------------------------------+
// Injection API (called from serial handler on Core 0)
//--------------------------------------------------------------------+

void xbox_inject_buttons(uint16_t buttons, uint16_t trigger_left, uint16_t trigger_right) {
    xbox_gamepad_state_t *gs = xbox_host_get_gamepad();
    uint32_t save = spin_lock_blocking(gs->lock);
    gs->inject_buttons = buttons;
    gs->inject_trigger_left = trigger_left;
    gs->inject_trigger_right = trigger_right;
    gs->inject_mask |= XBOX_INJECT_BUTTONS | XBOX_INJECT_TRIGGERS;
    spin_unlock(gs->lock, save);
}

void xbox_inject_stick_left(int16_t x, int16_t y) {
    xbox_gamepad_state_t *gs = xbox_host_get_gamepad();
    uint32_t save = spin_lock_blocking(gs->lock);
    gs->inject_stick_lx = x;
    gs->inject_stick_ly = y;
    gs->inject_mask |= XBOX_INJECT_STICK_L;
    spin_unlock(gs->lock, save);
}

void xbox_inject_stick_right(int16_t x, int16_t y) {
    xbox_gamepad_state_t *gs = xbox_host_get_gamepad();
    uint32_t save = spin_lock_blocking(gs->lock);
    gs->inject_stick_rx = x;
    gs->inject_stick_ry = y;
    gs->inject_mask |= XBOX_INJECT_STICK_R;
    spin_unlock(gs->lock, save);
}

void xbox_inject_clear(void) {
    xbox_gamepad_state_t *gs = xbox_host_get_gamepad();
    uint32_t save = spin_lock_blocking(gs->lock);
    gs->inject_mask = 0;
    gs->inject_buttons = 0;
    gs->inject_trigger_left = 0;
    gs->inject_trigger_right = 0;
    gs->inject_stick_lx = 0;
    gs->inject_stick_ly = 0;
    gs->inject_stick_rx = 0;
    gs->inject_stick_ry = 0;
    spin_unlock(gs->lock, save);
}
