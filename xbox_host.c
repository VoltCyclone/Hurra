/*
 * Xbox Controller Host Driver
 *
 * TinyUSB custom host class driver for Xbox GIP controllers.
 * Runs on Core 1. Detects Xbox controllers by vendor-specific interface
 * class (0xFF/0x47/0xD0), sets g_xbox_mode, proxies GIP traffic between
 * controller and console, and populates shared gamepad state.
 */

#include "xbox_host.h"
#include "xbox_gip.h"
#include "tusb.h"
#include "host/usbh.h"
#include "host/usbh_pvt.h"
#include "led_control.h"
#include "defines.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include <string.h>

//--------------------------------------------------------------------+
// Global Xbox Mode Flag
//--------------------------------------------------------------------+

volatile bool g_xbox_mode = false;

//--------------------------------------------------------------------+
// Private State
//--------------------------------------------------------------------+

// Endpoint addresses for the connected Xbox controller
static uint8_t  xbox_ep_in  = 0;
static uint8_t  xbox_ep_out = 0;
static uint8_t  xbox_dev_addr = 0;
static uint8_t  xbox_itf_num = 0;
static bool     xbox_mounted = false;

// GIP sequence counters
static uint8_t  gip_seq_host = 0;   // Sequence for host->controller packets

// Auth proxy state
static xbox_auth_state_t auth_state = XBOX_AUTH_IDLE;

// Flag for Core 0 to know it needs to re-enumerate after Xbox disconnect
static volatile bool xbox_reenum_pending = false;

// Keepalive timer
static uint32_t last_keepalive_ms = 0;

// Shared gamepad state accumulator
static xbox_gamepad_state_t gamepad_state;

// Cross-core packet queues
// host_to_device: Core 1 writes (controller responses), Core 0 reads (sends to console)
// device_to_host: Core 0 writes (console commands), Core 1 reads (sends to controller)
static gip_packet_queue_t host_to_device_queue;
static gip_packet_queue_t device_to_host_queue;

// IN transfer buffer
static uint8_t  xfer_buf_in[GIP_MAX_PACKET_SIZE];
// USB transfers retain the buffer until completion. Only modify when OUT is idle.
static uint8_t xfer_buf_out[GIP_MAX_PACKET_SIZE];
static uint8_t pending_out_len;
static bool power_on_pending;

// Spinlock numbers for shared state (claim unique IDs from Pico SDK pool)
#define XBOX_SPINLOCK_GAMEPAD   24
#define XBOX_SPINLOCK_H2D_QUEUE 25
#define XBOX_SPINLOCK_D2H_QUEUE 26

//--------------------------------------------------------------------+
// Forward Declarations
//--------------------------------------------------------------------+

static bool xbox_host_send_power_on(void);
static void xbox_host_process_gip_packet(const uint8_t *data, uint16_t len);
static void xbox_host_submit_in_xfer(void);

//--------------------------------------------------------------------+
// TinyUSB Custom Host Driver Callbacks
//--------------------------------------------------------------------+

static bool xbox_host_driver_init(void) {
    pending_out_len = 0;
    power_on_pending = false;
    memset(&gamepad_state, 0, sizeof(gamepad_state));
    gamepad_state.lock = spin_lock_instance(XBOX_SPINLOCK_GAMEPAD);
    gip_queue_init(&host_to_device_queue, XBOX_SPINLOCK_H2D_QUEUE);
    gip_queue_init(&device_to_host_queue, XBOX_SPINLOCK_D2H_QUEUE);
    return true;
}

static bool xbox_host_driver_deinit(void) {
    pending_out_len = 0;
    power_on_pending = false;
    xbox_mounted = false;
    xbox_ep_in = 0;
    xbox_ep_out = 0;
    xbox_dev_addr = 0;
    g_xbox_mode = false;
    auth_state = XBOX_AUTH_IDLE;
    return true;
}

static bool xbox_host_driver_open(uint8_t rhport, uint8_t dev_addr,
                                   tusb_desc_interface_t const *desc_itf,
                                   uint16_t max_len) {
    (void)rhport;

    // Match Xbox GIP interface: class=0xFF, subclass=0x47, protocol=0xD0
    if (desc_itf->bInterfaceClass    != XBOX_ITF_CLASS ||
        desc_itf->bInterfaceSubClass != XBOX_ITF_SUBCLASS ||
        desc_itf->bInterfaceProtocol != XBOX_ITF_PROTOCOL) {
        return false;
    }

    // Parse endpoint descriptors
    uint16_t drv_len = sizeof(tusb_desc_interface_t);
    uint8_t const *p_desc = (uint8_t const *)desc_itf + drv_len;

    for (uint8_t i = 0; i < desc_itf->bNumEndpoints; i++) {
        if (drv_len >= max_len) break;

        tusb_desc_endpoint_t const *ep_desc = (tusb_desc_endpoint_t const *)p_desc;
        if (ep_desc->bDescriptorType != TUSB_DESC_ENDPOINT) break;

        if (tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_IN) {
            xbox_ep_in = ep_desc->bEndpointAddress;
            tuh_edpt_open(dev_addr, ep_desc);
        } else {
            xbox_ep_out = ep_desc->bEndpointAddress;
            tuh_edpt_open(dev_addr, ep_desc);
        }

        drv_len += tu_desc_len(p_desc);
        p_desc = tu_desc_next(p_desc);
    }

    if (xbox_ep_in == 0 || xbox_ep_out == 0) {
        return false; // Need both IN and OUT endpoints
    }

    xbox_dev_addr = dev_addr;
    xbox_itf_num = desc_itf->bInterfaceNumber;
    xbox_mounted = true;
    g_xbox_mode = true;
    auth_state = XBOX_AUTH_IDLE;
    gip_seq_host = 0;

    return true;
}

static bool xbox_host_driver_set_config(uint8_t dev_addr, uint8_t itf_num) {
    (void)itf_num;

    // Send power-on command to controller
    power_on_pending = !xbox_host_send_power_on();

    // Start listening for IN transfers
    xbox_host_submit_in_xfer();

    // Signal to TinyUSB that configuration is complete
    usbh_driver_set_config_complete(dev_addr, itf_num);

    last_keepalive_ms = to_ms_since_boot(get_absolute_time());

    neopixel_set_status_override(STATUS_CONSOLE_MODE);

    return true;
}

static bool __not_in_flash_func(xbox_host_driver_xfer_cb)(uint8_t dev_addr, uint8_t ep_addr,
                                      xfer_result_t result, uint32_t xferred_bytes) {
    (void)dev_addr;

    if (result != XFER_RESULT_SUCCESS) {
        // Re-submit IN transfer on failure
        if (ep_addr == xbox_ep_in) {
            xbox_host_submit_in_xfer();
        }
        return true;
    }

    if (ep_addr == xbox_ep_in) {
        // Process received GIP packet from controller
        if (xferred_bytes >= GIP_HEADER_SIZE && xferred_bytes <= sizeof(xfer_buf_in)) {
            xbox_host_process_gip_packet(xfer_buf_in, (uint16_t)xferred_bytes);
        }

        // Re-submit IN transfer for next packet
        xbox_host_submit_in_xfer();
    }

    return true;
}

static void xbox_host_driver_close(uint8_t dev_addr) {
    if (dev_addr == xbox_dev_addr) {
        pending_out_len = 0;
        power_on_pending = false;
        xbox_mounted = false;
        xbox_ep_in = 0;
        xbox_ep_out = 0;
        xbox_dev_addr = 0;
        g_xbox_mode = false;
        auth_state = XBOX_AUTH_IDLE;
        xbox_reenum_pending = true;  // Signal Core 0 to re-enumerate
        neopixel_clear_status_override();
    }
}

//--------------------------------------------------------------------+
// Register Custom Host Driver
//--------------------------------------------------------------------+

static const usbh_class_driver_t xbox_host_driver = {
    .name       = "XBOX",
    .init       = xbox_host_driver_init,
    .deinit     = xbox_host_driver_deinit,
    .open       = xbox_host_driver_open,
    .set_config = xbox_host_driver_set_config,
    .xfer_cb    = xbox_host_driver_xfer_cb,
    .close      = xbox_host_driver_close,
};

// TinyUSB calls this weak function to get application-level host drivers
usbh_class_driver_t const *usbh_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &xbox_host_driver;
}

//--------------------------------------------------------------------+
// GIP Packet Processing (Core 1)
//--------------------------------------------------------------------+

static void __not_in_flash_func(xbox_host_process_gip_packet)(const uint8_t *data, uint16_t len) {
    if (len < GIP_HEADER_SIZE) return;

    const gip_header_t *hdr = (const gip_header_t *)data;

    switch (hdr->command) {
        case GIP_CMD_INPUT:
            // Input report from controller -> accumulator
            if (len >= GIP_HEADER_SIZE + sizeof(xbox_input_report_t)) {
                const xbox_input_report_t *report =
                    (const xbox_input_report_t *)(data + GIP_HEADER_SIZE);

                uint32_t save = spin_lock_blocking(gamepad_state.lock);
                gamepad_state.physical = *report;
                gamepad_state.physical_updated = true;
                spin_unlock(gamepad_state.lock, save);

                // If we receive input reports while auth was proxying,
                // the console has accepted us — auth is complete.
                if (auth_state == XBOX_AUTH_PROXYING) {
                    auth_state = XBOX_AUTH_COMPLETE;
                    neopixel_set_status_override(STATUS_CONSOLE_READY);
                }
            }
            break;

        case GIP_CMD_ACK:
            // ACK from controller - no action needed
            break;

        case GIP_CMD_ANNOUNCE:
        case GIP_CMD_STATUS:
        case GIP_CMD_IDENTIFY:
        case GIP_CMD_AUTH:
        case GIP_CMD_SERIAL_NUM:
            // Auth/descriptor responses: queue for Core 0 to forward to console
            if (hdr->command == GIP_CMD_AUTH) {
                if (auth_state == XBOX_AUTH_IDLE) {
                    auth_state = XBOX_AUTH_PROXYING;
                    neopixel_set_status_override(STATUS_CONSOLE_AUTH);
                }
            }
            gip_queue_push(&host_to_device_queue, data, (uint8_t)len);
            break;

        default:
            // Unknown commands: queue for passthrough
            gip_queue_push(&host_to_device_queue, data, (uint8_t)len);
            break;
    }
}

//--------------------------------------------------------------------+
// GIP Command Helpers
//--------------------------------------------------------------------+

static bool xbox_host_send_power_on(void) {
    if (!xbox_mounted || !xbox_ep_out || usbh_edpt_busy(xbox_dev_addr, xbox_ep_out)) return false;
    // GIP power-on command: command=0x05, flags=0x20, sequence, length=1, payload=0x00
    uint8_t *pkt = xfer_buf_out;
    pkt[0] = GIP_CMD_POWER;
    pkt[1] = GIP_FLAG_INTERNAL;  // Internal flag
    pkt[2] = gip_seq_host++;
    pkt[3] = 1;                  // Payload length
    pkt[4] = 0x00;               // Power on

    return usbh_edpt_xfer(xbox_dev_addr, xbox_ep_out, pkt, GIP_HEADER_SIZE + 1);
}

static void xbox_host_submit_in_xfer(void) {
    if (xbox_mounted && xbox_ep_in) {
        usbh_edpt_xfer(xbox_dev_addr, xbox_ep_in, xfer_buf_in, sizeof(xfer_buf_in));
    }
}

static bool xbox_host_send_keepalive(void) {
    if (!xbox_mounted || !xbox_ep_out || usbh_edpt_busy(xbox_dev_addr, xbox_ep_out)) return false;
    // GIP keepalive: command=0x03, internal flag
    uint8_t *pkt = xfer_buf_out;
    pkt[0] = GIP_CMD_STATUS;
    pkt[1] = GIP_FLAG_INTERNAL;
    pkt[2] = gip_seq_host++;
    pkt[3] = 0; // No payload

    return usbh_edpt_xfer(xbox_dev_addr, xbox_ep_out, pkt, GIP_HEADER_SIZE);
}

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

void xbox_host_init(void) {
    // Driver init is handled by the TinyUSB callback
    // This can be used for any pre-init setup
}

void __not_in_flash_func(xbox_host_task)(void) {
    if (!xbox_mounted) return;

    if (!xbox_ep_out || usbh_edpt_busy(xbox_dev_addr, xbox_ep_out)) return;
    if (power_on_pending) {
        power_on_pending = !xbox_host_send_power_on();
        return;
    }

    // Keep a popped packet until submission succeeds; never drain a busy endpoint.
    if (!pending_out_len) {
        gip_queue_pop(&device_to_host_queue, xfer_buf_out, &pending_out_len);
    }
    if (pending_out_len) {
        if (usbh_edpt_xfer(xbox_dev_addr, xbox_ep_out, xfer_buf_out, pending_out_len)) {
            pending_out_len = 0;
        }
        return;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((now - last_keepalive_ms) >= GIP_KEEPALIVE_INTERVAL_MS &&
        xbox_host_send_keepalive()) {
        last_keepalive_ms = now;
    }
}

xbox_auth_state_t xbox_host_get_auth_state(void) {
    return auth_state;
}

xbox_gamepad_state_t *xbox_host_get_gamepad(void) {
    return &gamepad_state;
}

gip_packet_queue_t *xbox_host_get_h2d_queue(void) {
    return &host_to_device_queue;
}

gip_packet_queue_t *xbox_host_get_d2h_queue(void) {
    return &device_to_host_queue;
}

bool xbox_host_check_and_clear_reenum(void) {
    if (xbox_reenum_pending) {
        xbox_reenum_pending = false;
        return true;
    }
    return false;
}
