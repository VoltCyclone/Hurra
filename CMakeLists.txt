# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(PICO_SDK_PATH $ENV{HOME}/.pico-sdk/sdk/2.2.0-fresh)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Board selection: metro (Adafruit Metro RP2350) or pico2 (RP2350 only)
set(PICO_BOARD adafruit_metro_rp2350 CACHE STRING "Board type")

# Tell the SDK where to find custom board headers (e.g. adafruit_metro_rp2350.h)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)

# Detect board type and set appropriate pin configurations
if(PICO_BOARD STREQUAL "adafruit_metro_rp2350")
    set(USB_HOST_DP_PIN 32 CACHE STRING "USB Host D+ pin")
    set(USB_HOST_DM_PIN 33 CACHE STRING "USB Host D- pin")
    set(USB_HOST_5V_PIN 29 CACHE STRING "USB Host 5V power pin")
    set(LED_PIN 23 CACHE STRING "Status LED pin")
    set(NEOPIXEL_PIN 25 CACHE STRING "NeoPixel data pin")
    set(NEOPIXEL_POWER_PIN 255 CACHE STRING "NeoPixel power pin (255=not used)")
    message(STATUS "Configuring for Adafruit Metro RP2350 - USB Host on GPIO32/33")
else()
    # Default: Pico/Pico2 with common GPIO16/17 for PIO USB
    set(USB_HOST_DP_PIN 16 CACHE STRING "USB Host D+ pin")
    set(USB_HOST_DM_PIN 17 CACHE STRING "USB Host D- pin")
    set(USB_HOST_5V_PIN 18 CACHE STRING "USB Host 5V power pin")
    set(LED_PIN 25 CACHE STRING "Status LED pin")
    set(NEOPIXEL_PIN 21 CACHE STRING "NeoPixel data pin")
    set(NEOPIXEL_POWER_PIN 20 CACHE STRING "NeoPixel power pin")
    message(STATUS "Configuring for ${PICO_BOARD} - USB Host on GPIO16/17")
endif()

# UART instance: 0 = uart0 (GPIO 0/1), 1 = uart1 (GPIO 8/9)
set(KMBOX_UART_INSTANCE 0 CACHE STRING "UART instance for bridge communication (0 or 1)")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PIOKMbox C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(PIOKMbox
    PIOKMbox.c
    led_control.c
    usb_hid.c
    watchdog.c
    state_management.c
    kmbox_serial_handler.c
    smooth_injection.c
    humanization_fpu.c
    dcp_helpers.S
    xbox_host.c
    xbox_device.c
)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(PIOKMbox ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

pico_set_program_name(PIOKMbox "PIOKMbox")
pico_set_program_version(PIOKMbox "0.1")

# Modify the below lines to enable/disable output over UART/USB
# UART0 is used for KMBox serial input via RP2350 USB Bridge
# USB CDC is disabled to avoid breaking HID enumeration
# The protocol UART is owned by the response queue; debug stdio must not write it.
pico_enable_stdio_uart(PIOKMbox 0)
pico_enable_stdio_usb(PIOKMbox 0)

# Optimization flags for release performance at 240MHz
# -O3: Aggressive inlining and vectorization
# -ffast-math: Enable FPU VFMA (fused multiply-add) instructions on Cortex-M33
# -funroll-loops: Unroll tight inner loops (helps DMA/UART processing)
# -ffunction-sections -fdata-sections: Enable dead code elimination with --gc-sections
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -ffast-math -funroll-loops -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -funroll-loops -ffunction-sections -fdata-sections")
target_link_options(PIOKMbox PRIVATE -Wl,--gc-sections)

# Add the standard library to the build
target_link_libraries(PIOKMbox
        pico_stdlib
        tinyusb_device
        tinyusb_host
        tinyusb_board
        hardware_pio
        hardware_dma
        hardware_watchdog
        hardware_uart
        hardware_spi
        hardware_irq
        hardware_adc
        pico_unique_id
        pico_multicore
        hardware_dcp
        pico_flash
        pico_rand
        pico_time
        m)

# Add PIO USB library
add_subdirectory(lib/Pico-PIO-USB)

# Add KMBox Commands library
add_subdirectory(lib/kmbox-commands)

# Shared libraries
add_subdirectory(lib/hid-defs)
add_subdirectory(lib/fast-protocol)
add_subdirectory(lib/wire-protocol)
add_subdirectory(lib/peri-clock)
add_subdirectory(lib/dma-uart)
add_subdirectory(lib/led-utils)

# PIO USB library support (now enabled)
message(STATUS "PIO USB library support enabled")
# Add compile definitions for TinyUSB and pin configurations
target_compile_definitions(PIOKMbox PRIVATE
        CFG_TUSB_CONFIG_FILE="tusb_config.h"
        BOARD_TUH_RHPORT=1
        BOARD_TUD_RHPORT=0
        PIO_USB_USE_TINYUSB
        BUILD_CONFIG=BUILD_CONFIG_DEVELOPMENT
        PIO_USB_AVAILABLE=1
        CFG_TUH_RPI_PIO_USB=1
        CPU_FREQ=240000
        KMBOX_UART_INSTANCE=${KMBOX_UART_INSTANCE}
        PIN_USB_HOST_DP=${USB_HOST_DP_PIN}
        PIN_USB_HOST_DM=${USB_HOST_DM_PIN}
        PIN_USB_5V=${USB_HOST_5V_PIN}
        PIN_LED=${LED_PIN}
        PIN_NEOPIXEL=${NEOPIXEL_PIN}
        NEOPIXEL_POWER=${NEOPIXEL_POWER_PIN}
        ENABLE_XBOX_PASSTHROUGH=1
)

# Link PIO USB library
target_link_libraries(PIOKMbox pico_pio_usb)

# Link KMBox Commands library
target_link_libraries(PIOKMbox kmbox_commands)

# Link shared libraries
target_link_libraries(PIOKMbox hid_defs fast_protocol wire_protocol peri_clock dma_uart led_utils)

# Add PIO USB HCD implementation directly
target_sources(PIOKMbox PRIVATE
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
 )

# Add the standard include files to the build
target_include_directories(PIOKMbox PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
)

pico_add_extra_outputs(PIOKMbox)
