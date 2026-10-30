<div align="center">

# PIOKMbox

**USB HID Passthrough & KMBox Serial Interface for RP2350**

[![Platform](https://img.shields.io/badge/platform-RP2350-blue?logo=raspberrypi&logoColor=white)](#hardware-requirements)
[![Language](https://img.shields.io/badge/language-C-555?logo=c&logoColor=white)](#development)
[![License](https://img.shields.io/badge/license-open--source-green)](#license)
[![Protocol](https://img.shields.io/badge/protocol-KMBox%20B%2B%20%7C%20Ferrum%20%7C%20Macku-orange)](#kmbox-compatibility)

A high-performance dual-role USB HID firmware that creates a **transparent USB passthrough device** while providing **KMBox-compatible serial control** for mouse and keyboard automation with advanced humanization and optional visual feedback via ILI9341 display.

```
Mouse/Keyboard ──► [RP2350 Board 1] ──► PC
                       ▲  UART (crossed)
                       ▼
                   [RP2350 Board 2] ──► ILI9341 TFT Display
                       ▲
                       │ USB CDC
                   [PC Tool / Script]
```

</div>

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Hardware Requirements](#hardware-requirements)
- [Quick Start](#quick-start)
- [Using KMBox Commands](#using-kmbox-commands)
- [Movement Humanization](#movement-humanization)
- [KMBox Bridge — Visual Feedback & Autopilot](#kmbox-bridge--visual-feedback--autopilot)
- [Architecture](#architecture)
- [Status Indicators](#status-indicators)
- [Development](#development)
- [Performance Metrics](#performance-metrics)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## Overview

PIOKMbox turns an RP2350 board into a transparent man-in-the-middle USB HID device. Your PC sees the original mouse or keyboard — not the microcontroller — while serial commands let you inject precise, humanized input alongside physical devices.

| Capability          | Description                                                                                                                           |
|:--------------------|:--------------------------------------------------------------------------------------------------------------------------------------|
| **USB Passthrough** | Mirrors the connected device's VID/PID, manufacturer, and product name. The PC never sees the RP2350.                                 |
| **Serial Control**  | Accepts KMBox, Macku binary, and Ferrum-compatible commands over UART to inject mouse movements, clicks, and keyboard input.          |
| **Humanization**    | Movement-aware jitter, velocity suppression, overshoot simulation, and Bezier easing — all configurable across four intensity levels. |
| **Visual Feedback** | Optional ILI9341 TFT display shows real-time latency graphs, connection status, and thermal metrics.                                  |

> **Thermal note:** This firmware runs at 240 MHz+ for optimal performance. Monitor temperature during extended sessions, especially with the TFT display enabled.

---

## Key Features

- **Transparent USB Passthrough** — PC sees the original device, not the RP2350
- **KMBox Protocol Compatibility** — works with existing KMBox B+, Ferrum, and Macku tools
- **Dual-Core Architecture** — dedicated cores for USB host and device operations
- **Movement Humanization** — 4 configurable modes (OFF / LOW / MEDIUM / HIGH) with adaptive jitter
- **Low Latency** — < 50 µs binary protocol, < 100 µs text commands
- **Hardware Watchdog** — automatic recovery from USB stack failures
- **Visual Status** — NeoPixel RGB feedback and optional ILI9341 TFT display
- **Xbox Controller Support** — GIP protocol passthrough

---

## Hardware Requirements

### Recommended Setup

**Dual Adafruit Metro RP2350 + ILI9341 Display**

#### Board 1: USB Proxy (Main KMBox)

| Item      | Detail                                        |
|:----------|:----------------------------------------------|
| **Board** | Adafruit Metro RP2350                         |
| **Role**  | USB HID passthrough + KMBox command execution |
| **USB-A** | Physical mouse or keyboard                    |
| **USB-C** | To PC (appears as the passthrough device)     |
| **UART**  | TX/RX to Board 2 (crossed), shared GND        |

#### Board 2: Bridge / Autopilot (Optional)

| Item      | Detail                                            |
|:----------|:--------------------------------------------------|
| **Board** | Adafruit Metro RP2350                             |
| **Role**  | Computer vision tracking + ILI9341 display driver |
| **USB-C** | To PC (for serial input commands)                 |
| **UART**  | TX/RX to Board 1 (crossed), shared GND            |
| **SPI**   | ILI9341 display + optional touch controller       |

#### ILI9341 TFT Display (Optional)

| Spec       | Value                                                   |
|:-----------|:--------------------------------------------------------|
| Resolution | 320 x 240                                               |
| Interface  | SPI (hardware accelerated)                              |
| Features   | Real-time latency graphs, status display, touch support |

### Serial Wire Configuration

> **UART wires must be crossed between boards.**

```
Board 1 (Proxy)           Board 2 (Bridge)
    TX  ─────────────────►  RX
    RX  ◄─────────────────  TX
    GND ◄────────────────►  GND
```

### Power

- USB bus powered (5 V)
- Typical draw: 150–300 mA (varies with display)
- TFT display adds ~80–150 mA
- No external power supply required

---

## Quick Start

### 1. Clone and Build

```bash
git clone --recursive https://github.com/ramseymcgrath/RaspberryKMBox.git
cd RaspberryKMBox

# If you already cloned without --recursive:
git submodule update --init --recursive

# Build options:
./build.sh metro          # Main KMBox for Metro RP2350
./build.sh bridge-metro   # Bridge for Metro RP2350 with display
./build.sh dual-metro     # Build both targets
./build.sh all            # Build all configurations
```

### 2. Flash Firmware

| Board                   | Steps                                                                                                                      |
|:------------------------|:---------------------------------------------------------------------------------------------------------------------------|
| **Board 1** (USB Proxy) | Hold **BOOTSEL** while connecting USB-C to PC. Drag `build-metro/PIOKMbox.uf2` to the mounted **RP2350** drive.            |
| **Board 2** (Bridge)    | Hold **BOOTSEL** while connecting USB-C to PC. Drag `bridge/build-metro/kmbox_bridge.uf2` to the mounted **RP2350** drive. |

Both boards reboot automatically after flashing.

### 3. Wire the Boards

Connect UART **crossed** (TX→RX, RX→TX) with a shared GND. See [Serial Wire Configuration](#serial-wire-configuration) above.

For display wiring, see [`bridge/README.md`](bridge/README.md).

### 4. Connect Devices

1. **Mouse / Keyboard** → Board 1 USB-A port
2. **Board 1 USB-C** → PC (passthrough device)
3. **Board 2 USB-C** → PC (serial input commands)

### 5. Verify Operation

- **NeoPixel** changes color based on connected devices (see [Status Indicators](#status-indicators))
- **Mouse / Keyboard** should work normally through the PC
- **Display** shows connection status and latency (if bridge installed)
- **Serial** — Board 1 accepts KMBox commands via UART

---

## Using KMBox Commands

### Connection

| Parameter | Value                                                      |
|:----------|:-----------------------------------------------------------|
| Interface | UART (hardware serial, crossed between boards)             |
| Baud Rate | 115200 (configurable; speeds are uncapped in USB CDC mode) |
| Protocol  | KMBox-compatible text and binary commands                  |

### Text Commands

```text
km.move(100, 50)      # Relative mouse move (+X right, +Y down)
km.left(1)            # Press left button
km.left(0)            # Release left button
km.click(0)           # Left click (0=left, 1=right, 2=middle)
km.wheel(5)           # Scroll up (negative = down)
km.lock_mx(1)         # Lock X axis (ignore physical mouse X)
km.lock_my(1)         # Lock Y axis (ignore physical mouse Y)
km.unlock_mx()        # Unlock X axis
km.unlock_my()        # Unlock Y axis
```

### Fast Binary Protocol

For ultra-low latency (< 50 µs), use 8-byte binary packets:

```python
# Fast mouse move (0x01 command)
packet = bytes([0x01, x_lo, x_hi, y_lo, y_hi, buttons, wheel, 0x00])
serial.write(packet)
```

- Bypasses text parsing for minimal latency
- Fixed 8-byte packets for predictable timing
- Supports 1000+ commands/sec

### Monitor Mode

Real-time button state queries for automation:

```text
km.monitor(1)         # Enable monitoring
km.isdown_left()      # Query left button (returns 0 or 1)
km.isdown_right()     # Query right button
km.isdown_middle()    # Query middle button
km.isdown_side1()     # Query side button 1
km.isdown_side2()     # Query side button 2
km.monitor(0)         # Disable monitoring
```

### KMBox Compatibility

Fully compatible with **KMBox B+**, **Ferrum**, and **Macku** protocols:

| Feature                                     |  Status   |
|:--------------------------------------------|:---------:|
| Mouse control (movement, buttons, scroll)   | Supported |
| Axis locking (X/Y movement, button masking) | Supported |
| Monitor mode (real-time button state)       | Supported |
| Fast binary protocol (< 50 µs)              | Supported |
| Smooth injection (velocity matching)        | Supported |
| Movement humanization (Bezier easing)       | Supported |

---

## Movement Humanization

An advanced anti-detection system that simulates natural human mouse movement through adaptive jitter, velocity suppression, and overshoot simulation. All features are hardware-accelerated with < 10 cycle overhead per pixel.

### How It Works

**Movement-Aware Scaling** — intensity adapts to movement distance:

| Distance  | Jitter Scale | Behavior                                         |
|:----------|:-------------|:-------------------------------------------------|
| 0–20 px   | 0.7–0.8x     | Simulates hand tremor during precise positioning |
| 20–60 px  | 0.3–0.7x     | Balances precision and speed                     |
| 60–110 px | 0.1–0.3x     | Reduced jitter for intentional movements         |
| 110+ px   | 0.05–0.09x   | Minimal jitter — keeps fast flicks snappy        |

**Velocity-Based Suppression** — jitter fades as movement slows, preventing a "shaky" cursor after motion completes. Mimics natural hand settling.

**Physical Input Protection** — humanization applies only to synthetic injections. Physical mouse and keyboard input passes through untouched.

### Modes

Control via button press (GPIO 7) or serial command:

| Mode       | Jitter    | Overshoot  | Onset Delay | Use Case                           |
|:-----------|:----------|:-----------|:------------|:-----------------------------------|
| **OFF**    | None      | Disabled   | None        | Testing, maximum precision         |
| **LOW**    | ± 0.06 px | Disabled   | 0–1 frames  | Competitive gaming, fast reactions |
| **MEDIUM** | ± 0.17 px | 5% chance  | 1–3 frames  | **Default** — general use          |
| **HIGH**   | ± 0.33 px | 10% chance | 2–6 frames  | Maximum stealth                    |

<details>
<summary><strong>Detailed mode parameters</strong></summary>

**OFF** — Linear movement, no variations. Max 16 px/frame (fixed). Best for testing and high-speed automation.

**LOW** — Barely perceptible jitter at sensor noise floor. ± 1% delivery error. Max 15–17 px/frame (randomized per session). Accumulator clamp: ± 4 px.

**MEDIUM** (Default) — Matches physical mouse sensor noise (~± 0.17 px). ± 2% delivery error. Max 13–19 px/frame (randomized per session). 5% overshoot chance on 15–120 px moves (max 0.5 px). Accumulator clamp: ± 3 px.

**HIGH** — Upper bound of sensor noise (~± 0.33 px). ± 3% delivery error. Max 10–22 px/frame (randomized per session). 10% overshoot chance on 15–120 px moves (max 1.0 px). Accumulator clamp: ± 2 px (tightest).

</details>

### Additional Techniques

| Technique                     | Description                                                                                                                 |
|:------------------------------|:----------------------------------------------------------------------------------------------------------------------------|
| **Bezier Easing**             | Cubic ease-in-out for large movements; quadratic ease-out for quick corrections. Auto-selected by movement characteristics. |
| **Micro-Jitter**              | ± 1–2 px hand tremor simulation. Context-aware (more during mid-movement). 40% chance per frame.                            |
| **Overshoot & Correction**    | 5–10% chance to overshoot by 0.5–1.0 px, smoothly corrected over 2–4 frames. Only on moves > 15 px.                         |
| **Per-Session Randomization** | Base parameters vary on init. ± 1 px per move, ± 1 frame for moves > 3 frames. Prevents statistical fingerprinting.         |

### Button Control (GPIO 7)

| Action                  | Result                                                    |
|:------------------------|:----------------------------------------------------------|
| **Short press** (< 3 s) | Cycle humanization mode. LED: Red → Yellow → Green → Cyan |
| **Long press** (>= 3 s) | Reset USB stacks                                          |

For full technical details, see [HUMANIZATION.md](HUMANIZATION.md).

---

## KMBox Bridge — Visual Feedback & Autopilot

An optional companion firmware for a second **Adafruit Metro RP2350** with an ILI9341 TFT display. Provides real-time visual feedback and computer vision-based autopilot capabilities.

```
┌──────────┐  USB CDC   ┌───────────────────┐  UART (crossed)  ┌──────────────┐
│ PC Tool  │◄──────────►│  Metro RP2350     │◄────────────────►│ KMBox Metro  │
│ (input)  │            │  (Bridge/Display) │                   │ (USB Proxy)  │
└──────────┘            └───────────────────┘                   └──────────────┘
                                 │
                                 ├── ILI9341 TFT (SPI)
                                 └── Touch Controller (optional)
```

### Bridge Features

| Feature                    | Description                                                       |
|:---------------------------|:------------------------------------------------------------------|
| **ILI9341 Display**        | 320 x 240 real-time status, latency graphs, connection indicators |
| **USB CDC Interface**      | Receives serial commands from PC for tracking and automation      |
| **Color Tracking**         | Hardware-accelerated blob detection and centroid calculation      |
| **UART Relay**             | Bidirectional serial between bridge and main KMBox                |
| **Touch Support**          | Optional XPT2046 / FT6206 for interactive controls                |
| **Temperature Monitoring** | Real-time thermal tracking with visual gauges                     |

### Bridge Quick Setup

1. `./build.sh dual-metro` — builds both boards
2. Wire UART crossed (TX→RX, RX→TX) + GND between boards
3. Connect ILI9341 to Bridge Metro SPI pins (see [`bridge/README.md`](bridge/README.md))
4. Flash Board 1: `build-metro/PIOKMbox.uf2`
5. Flash Board 2: `bridge/build-metro/kmbox_bridge.uf2`

---

## Architecture

### Dual-Core Design

```
┌─────────────────────────────────────────────────┐
│                   RP2350                        │
│                                                 │
│  ┌─────────────────┐   ┌─────────────────────┐  │
│  │     Core 0      │   │      Core 1         │  │
│  │                 │   │                     │  │
│  │  TinyUSB Device │   │  TinyUSB Host       │  │
│  │  (HID → PC)     │   │  (PIO-USB ← Mouse) │  │
│  │  KMBox Parser   │   │  Report Forwarding  │  │
│  │  Smooth Inject  │   │  VID/PID Caching    │  │
│  └─────────────────┘   └─────────────────────┘  │
│           ▲                      │               │
│           └──── Shared Memory ───┘               │
└─────────────────────────────────────────────────┘
```

1. **Core 1** runs TinyUSB host on PIO-USB to communicate with your physical mouse/keyboard
2. Firmware reads HID report descriptors and caches VID/PID/strings from attached devices
3. **Core 0** exposes a TinyUSB HID device to the PC, mirroring the attached device's identity
4. On VID/PID change, the device re-enumerates to reflect the new identity
5. Physical HID input and KMBox serial commands are combined with intelligent axis locking

### USB Passthrough

- **Transparent Identity** — PC sees original mouse/keyboard, not the RP2350
- **Report Mirroring** — all HID reports forwarded with < 1 ms latency
- **Dynamic Re-enumeration** — automatically adapts when devices change
- **String Descriptors** — manufacturer and product names mirrored

### Serial Command Injection

- **Dual Input** — physical and synthetic input coexist seamlessly
- **Axis Locking** — selective X/Y/button filtering for precise control
- **Smooth Injection** — velocity-matched movement with frame-perfect timing
- **Priority Handling** — physical input takes priority; synthetic fills gaps

---

## Status Indicators

### NeoPixel Colors

| Color   | State                      |
|:--------|:---------------------------|
| Blue    | Starting up                |
| Green   | USB device only            |
| Orange  | USB host only              |
| Cyan    | Both USB stacks active     |
| Magenta | Mouse connected            |
| Yellow  | Keyboard connected         |
| Pink    | Mouse + keyboard connected |
| Red     | Error state                |
| Purple  | Suspended                  |

### Humanization Mode LED

| Color  | Mode                  |
|:-------|:----------------------|
| Red    | OFF (no humanization) |
| Yellow | LOW (minimal)         |
| Green  | MEDIUM (default)      |
| Cyan   | HIGH (maximum)        |

### LED Patterns

| Pattern    | Meaning                   |
|:-----------|:--------------------------|
| Fast blink | Device connected / active |
| Slow blink | Device suspended or error |
| Solid      | Normal operation          |

---

## Development

### Project Layout

```
RaspberryKMBox/
├── PIOKMbox.c                # Main firmware — core orchestration
├── usb_hid.*                 # HID device/host, VID/PID mirroring
├── kmbox_serial_handler.*    # KMBox UART command integration
├── smooth_injection.*        # Humanized movement engine
├── humanization_lut.*        # Precomputed jitter lookup tables
├── led_control.*             # LED & WS2812 NeoPixel control
├── watchdog.*                # Hardware/software watchdog
├── bridge_handler.*          # Bridge communication protocol
├── xbox_device.*, xbox_host.*# Xbox controller passthrough
├── ws2812.pio                # PIO program for NeoPixel
├── defines.h, config.h       # Configuration and pin definitions
├── lib/
│   ├── Pico-PIO-USB/         # PIO USB library (submodule)
│   ├── kmbox-commands/       # KMBox command parser
│   ├── fast-protocol/        # Binary protocol definitions
│   ├── wire-protocol/        # Wire format utilities
│   ├── dma-uart/             # DMA-accelerated UART
│   └── led-utils/            # LED control abstractions
├── bridge/                   # Bridge firmware
│   ├── main.c                # Bridge entry point
│   ├── ili9341.c             # ILI9341 TFT display driver
│   ├── latency_tracker.*     # Performance monitoring
│   ├── core1_translator.*    # CV processing
│   ├── bridge_client.py      # Python control client
│   └── ferrum_translator.c   # Ferrum protocol support
├── tools/                    # Development utilities
│   ├── generate_lut.py       # Generate humanization tables
│   ├── kmbox_stress_test.py  # Stress testing
│   └── logitech_hid_dump.py  # HID device analysis
├── boards/                   # Board definitions
│   └── adafruit_metro_rp2350.h
├── build.sh                  # Multi-target build script
└── CMakeLists.txt            # Build configuration
```

### Build Targets

```bash
./build.sh pico2            # RP2350 (Pico 2)
./build.sh metro            # Metro RP2350 (main KMBox)
./build.sh bridge           # Bridge (XIAO RP2350)
./build.sh bridge-metro     # Bridge (Metro RP2350)
./build.sh dual-metro       # Both Metro boards
./build.sh all              # All configurations
```

Append `clean` to force a rebuild: `./build.sh all clean`

### Host Regression Checks

These checks need Python 3 and a C compiler with address and undefined-behavior
sanitizers. They run production functions against mocked peripherals; they do
not replace firmware builds or testing on the boards.

```bash
python3 tests/test_xbox_transfers.py
python3 tests/test_bridge_parsers.py
python3 tests/test_uart_timing.py
python3 tests/test_display_io.py
python3 tests/test_bridge_protocol.py
python3 tests/test_proxy_tx.py
python3 tests/test_proxy_rx.py
python3 tests/test_dma_rx.py
```

Build and flash the bridge and proxy from the same revision. Both bridge targets
use eight-byte fast packets. Humanization configuration uses opcode `0x10`, and
info requests use `0x11` to avoid the carriage-return opcode `0x0D`; info replies
retain `0x0D`. The variable-length wire-protocol library is not the proxy's UART
parser and must not be mixed into that stream.

The default software TFT path advances refreshes one DMA row per main-loop pass.
Rendering runs in the main loop, while the timer interrupt only schedules it.
The optional `TFT_HW_ACCEL` path and explicit splash/error refreshes remain
synchronous.

The proxy's protocol UART has stdio disabled. Command replies use an explicit
output callback into a shared record queue; debug `printf` calls do not write
onto that UART. The queue supports seven pending records of up to 256 bytes,
including any CRLF suffix. A full queue rejects whole records and accounts for
their bytes in `kmbox_get_tx_dropped_bytes()`. Startup and DMA-allocation failure
use polled RX and IRQ TX, preserving pending data when DMA starts.

DMA RX on both boards uses RP2350 self-trigger mode with a decrementing count,
so complete ring-buffer laps cannot appear empty. Copies are checked against a
second coherent count/address snapshot before delivery. A loss discards pending
input and partial parser state; the proxy also releases injected controls and
cancels queued/timed movement. Reception resumes after at least 1 ms without DMA
progress and an empty UART FIFO. The sender must provide that gap at a packet
boundary; continuous traffic is discarded during recovery.

The bridge exposes overrun events through `hw_uart_get_stats()` and discarded
bytes/ambiguous polls through `hw_uart_get_rx_loss_stats()`. The proxy exposes
all three through `kmbox_get_rx_loss_stats()` on Core 0. If a scheduling pause
could hide an entire hardware count period, reception fails closed and the byte
count is a lower bound. These counters describe discarded data, not retransmission
or flow control. No additional DMA channels or RX interrupts are required.

### Build Configuration

Presets defined in `defines.h`:

| Preset                     | Description                |
|:---------------------------|:---------------------------|
| `BUILD_CONFIG_DEVELOPMENT` | Default — verbose logging  |
| `BUILD_CONFIG_PRODUCTION`  | Optimized, minimal logging |
| `BUILD_CONFIG_TESTING`     | Extended diagnostics       |
| `BUILD_CONFIG_DEBUG`       | Full debug symbols         |

### Clock Speeds

| Target              | Frequency | Notes                       |
|:--------------------|:----------|:----------------------------|
| Main KMBox (RP2350) | 300 MHz   | Optimized for PIO-USB       |
| Bridge              | 280 MHz   | Balanced for display + UART |

### Prerequisites

- [Pico SDK](https://github.com/raspberrypi/pico-sdk) 2.2.0+
- CMake 3.13+
- `arm-none-eabi-gcc` 14.2+
- Git (for submodules)

---

## Performance Metrics

Typical results on Metro RP2350 @ 300 MHz:

| Operation             | Latency     | Notes                 |
|:----------------------|:------------|:----------------------|
| USB passthrough       | < 1 ms      | Report forwarding     |
| Text command          | < 100 µs    | Parsing + execution   |
| Binary command        | < 50 µs     | Direct execution      |
| Humanization overhead | < 10 cycles | Per-pixel calculation |
| Display update        | 16–33 ms    | 30–60 FPS typical     |
| UART transmission     | 87 µs       | 8 bytes @ 115200 baud |

---

## Troubleshooting

<details>
<summary><strong>USB Issues</strong></summary>

**Device not recognized:**
1. Verify 5 V power to USB host port
2. Check D+/D- wiring (GPIO 16/17)
3. Try a different USB cable (some are power-only)
4. Check debug UART for device support messages

**Re-enumeration loops:**
- Usually caused by an unstable attached device
- Check USB cable quality and power supply stability
- Review debug logs on GPIO 0/1 @ 115200 baud

**Passthrough not working:**
- LED should show device connected state
- Open debug UART to verify device detection
- Devices with complex HID descriptors may need adjustments

</details>

<details>
<summary><strong>Serial Communication</strong></summary>

**No response to KMBox commands:**
1. Verify UART wires are crossed (TX→RX, RX→TX)
2. Check baud rate (115200 default)
3. Ensure common ground connection
4. Test with: `km.move(10, 10)`

**Display not updating:**
1. Verify SPI connections to ILI9341
2. Check bridge firmware is flashed correctly
3. Review bridge debug output
4. Ensure TFT power (3.3 V or 5 V depending on module)

</details>

<details>
<summary><strong>Performance</strong></summary>

**High latency:**
- Check CPU clock speeds in CMakeLists.txt
- Set humanization mode to OFF for lowest latency
- Monitor temperature (thermal throttling)
- Reduce display update rate if using bridge

**Movement feels sluggish:**
- Try OFF or LOW humanization mode
- Check mouse polling rate (1000 Hz recommended)
- Verify physical mouse sensor quality

</details>

<details>
<summary><strong>Build Issues</strong></summary>

**CMake errors:**

```bash
# Ensure Pico SDK is installed and path is set
export PICO_SDK_PATH=/path/to/pico-sdk

# Clean and rebuild
./build.sh metro clean
```

**Flash failures:**
- Hold BOOTSEL button firmly during USB connect
- Try a different USB port or cable
- Verify the .uf2 file isn't corrupted

</details>

---

## Advanced Usage

### Custom Humanization Profiles

Edit `humanization_lut.c` to create custom jitter profiles. The LUT defines jitter multipliers based on movement distance. Regenerate with `tools/generate_lut.py`.

### Serial Protocol Extensions

Extend KMBox commands in `lib/kmbox-commands/`:

1. Define command structure
2. Add parser in `kmbox_serial_handler.c`
3. Implement handler logic
4. Update protocol documentation

### Display Customization

Modify the bridge display in `bridge/tft_display.c` — color schemes, widgets, refresh rates, and touch controls. See [`bridge/README.md`](bridge/README.md) for the display API.

---

## Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Test thoroughly on hardware
4. Document changes in code and README
5. Submit a pull request with a detailed description

See [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines.

---

## License

Main project files follow standard open-source practices. Libraries under `lib/` retain their respective licenses:

| Library      | License                        |
|:-------------|:-------------------------------|
| Pico-PIO-USB | See `lib/Pico-PIO-USB/LICENSE` |
| TinyUSB      | MIT                            |
| Pico SDK     | BSD 3-Clause                   |

---

## Acknowledgments

- [Raspberry Pi Foundation](https://www.raspberrypi.org/) — Pico SDK and documentation
- [TinyUSB](https://github.com/hathach/tinyusb) — USB stack
- [Sekigon-gonnoc](https://github.com/sekigon-gonnoc/Pico-PIO-USB) — Pico-PIO-USB
- [Adafruit](https://www.adafruit.com/) — RP2350 hardware
- KMBox community — protocol documentation

---

## Support

| Channel       | Link                                                                              |
|:--------------|:----------------------------------------------------------------------------------|
| Bug Reports   | [GitHub Issues](https://github.com/ramseymcgrath/RaspberryKMBox/issues)           |
| Discussions   | [GitHub Discussions](https://github.com/ramseymcgrath/RaspberryKMBox/discussions) |
| Documentation | See individual `.md` files for detailed topics                                    |

---

<sub>This project is for educational and accessibility purposes. Users are responsible for complying with applicable terms of service and regulations.</sub>
