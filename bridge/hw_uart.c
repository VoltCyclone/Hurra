/**
 * Hardware UART Driver for KMBox Bridge
 *
 * High-performance hardware UART implementation with DMA support.
 *
 * Clock: We keep clk_peri at the system clock (240MHz) because 3Mbaud
 * divides exactly: 240M / (16 * 5) = 3.0M.  Keeping clk_peri high
 * also benefits SPI throughput for the TFT display (40MHz vs 24MHz).
 */

#include "hw_uart.h"
#include "dma_uart.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/structs/uart.h"
#include <stdio.h>
#include <string.h>

// UART instance
#define HW_UART uart0

// DMA channels
static int tx_dma_chan = -1;
static int rx_dma_chan = -1;

// One staging buffer is sufficient: it is only reused after DMA is idle.
static uint8_t tx_buffer[HW_UART_TX_BUFFER_SIZE] __attribute__((aligned(4)));

// RX circular buffer (must be aligned to size for DMA ring mode)
static uint8_t rx_buffer[HW_UART_RX_BUFFER_SIZE] __attribute__((aligned(HW_UART_RX_BUFFER_SIZE)));
static dma_rx_state_t rx_state;
static uint32_t rx_reported_loss_epoch;

// Statistics
static volatile uint32_t total_tx_bytes = 0;
static volatile uint32_t total_rx_bytes = 0;

bool hw_uart_init(uint tx_pin, uint rx_pin, uint baud) {
    if (baud == 0) return false;
    // NOTE: We intentionally skip peri_clock_configure_stable() here.
    // 3Mbaud divides exactly from 240MHz (div=5), and keeping clk_peri
    // at 240MHz gives the TFT SPI bus 40MHz instead of 24MHz.
    uint32_t peri_freq = clock_get_hz(clk_peri);
    printf("[HW_UART] clk_peri: %lu Hz (%.2f MHz)\n",
           peri_freq, peri_freq / 1000000.0f);
    
    // Set up GPIO pins with correct UART function select BEFORE uart_init
    // CRITICAL: Must use UART_FUNCSEL_NUM to get correct function select
    // for this specific UART instance and pin combination
    gpio_set_function(tx_pin, UART_FUNCSEL_NUM(HW_UART, tx_pin));
    gpio_set_function(rx_pin, UART_FUNCSEL_NUM(HW_UART, rx_pin));
    gpio_pull_up(rx_pin);
    
    // Initialize UART - this automatically enables FIFOs and sets 8N1 format
    uint actual_baud = uart_init(HW_UART, baud);
    
    // Calculate and display baud rate accuracy
    int32_t baud_error_ppm = (int32_t)(((int64_t)actual_baud - baud) * 1000000 / baud);
    printf("[HW_UART] Requested: %u, Actual: %u, Error: %ld ppm\n", baud, actual_baud, baud_error_ppm);
    
    // Warn if baud rate error is too high (>2% is problematic)
    if (baud_error_ppm > 20000 || baud_error_ppm < -20000) {
        printf("[HW_UART] WARNING: Baud rate error >2%%, communication may be unreliable!\n");
    }
    
    // Set RX FIFO trigger to 1 byte (1/8 threshold) for responsive DMA
    uart_hw_t *uart_hw = uart_get_hw(HW_UART);
    uart_hw->ifls = (uart_hw->ifls & ~UART_UARTIFLS_RXIFLSEL_BITS) | (0 << UART_UARTIFLS_RXIFLSEL_LSB);
    
    // Disable flow control
    uart_set_hw_flow(HW_UART, false, false);
    
    // Disable UART interrupts (we use DMA)
    uart_set_irq_enables(HW_UART, false, false);
    
    // Drain any garbage
    while (uart_is_readable(HW_UART)) {
        uart_getc(HW_UART);
    }
    
    // Claim TX DMA channel
    tx_dma_chan = dma_claim_unused_channel(false);
    if (tx_dma_chan < 0) {
        printf("[HW_UART] Failed to claim TX DMA channel\n");
        return false;
    }
    
    // Configure TX DMA
    dma_channel_config tx_cfg = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    channel_config_set_dreq(&tx_cfg, uart_get_dreq(HW_UART, true));
    channel_config_set_high_priority(&tx_cfg, true);
    
    dma_channel_configure(
        tx_dma_chan,
        &tx_cfg,
        &uart_get_hw(HW_UART)->dr,
        NULL,
        0,
        false
    );
    
    // TX completion is polled from the DMA BUSY bit. No per-packet IRQ or
    // software flag is needed, and a delayed IRQ cannot mark a new TX idle.

    // Claim RX DMA channel
    rx_dma_chan = dma_claim_unused_channel(false);
    if (rx_dma_chan < 0) {
        printf("[HW_UART] Failed to claim RX DMA channel\n");
        dma_channel_unclaim(tx_dma_chan);
        tx_dma_chan = -1;
        return false;
    }
    
    // Configure RX DMA with circular buffer
    dma_channel_config rx_cfg = dma_channel_get_default_config(rx_dma_chan);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, true);
    
    uint dreq_num = uart_get_dreq(HW_UART, false);
    channel_config_set_dreq(&rx_cfg, dreq_num);
    
    uint ring_size_bits = __builtin_ctz(HW_UART_RX_BUFFER_SIZE);
    channel_config_set_ring(&rx_cfg, true, ring_size_bits);
    channel_config_set_irq_quiet(&rx_cfg, true);
    channel_config_set_high_priority(&rx_cfg, true);
    
    printf("[HW_UART] RX DMA: ch=%d dreq=%u ring_bits=%u buf_size=%u\n", 
           rx_dma_chan, dreq_num, ring_size_bits, HW_UART_RX_BUFFER_SIZE);
    
    // Print UART hardware register addresses for debugging
    printf("[HW_UART] UART base=%p DR=%p\n", uart_hw, &uart_hw->dr);
    
    dma_rx_init(&rx_state, HW_UART_RX_BUFFER_SIZE - 1, 0, 0, actual_baud, time_us_64());
    rx_state.fifo_status = &uart_hw->fr;
    rx_reported_loss_epoch = 0;
    // Self-triggered count reloads preserve continuous reception and expose progress.
    dma_channel_configure(
        rx_dma_chan,
        &rx_cfg,
        rx_buffer,
        &uart_hw->dr,  // Read from UART data register
        dma_encode_transfer_count_with_self_trigger(DMA_RX_TRANSFER_COUNT),
        true
    );
    
    // Verify DMA is actually configured and running
    printf("[HW_UART] DMA configured: read_addr=%p write_addr=%p count=%lu\n",
           (void*)dma_channel_hw_addr(rx_dma_chan)->read_addr,
           (void*)dma_channel_hw_addr(rx_dma_chan)->write_addr,
           dma_channel_hw_addr(rx_dma_chan)->transfer_count);
    
    printf("[HW_UART] Init complete: TX_DMA=%d, RX_DMA=%d\n", tx_dma_chan, rx_dma_chan);
    
    return true;
}

bool hw_uart_send(const uint8_t *data, size_t len) {
    if (!data || len == 0 || len > HW_UART_TX_BUFFER_SIZE) {
        return false;
    }
    
    if (tx_dma_chan < 0) {
        for (size_t i = 0; i < len; i++) {
            uart_putc_raw(HW_UART, data[i]);
        }
        total_tx_bytes += len;
        return true;
    }
    
    if (dma_channel_is_busy(tx_dma_chan)) return false;

    memcpy(tx_buffer, data, len);
    dma_channel_set_read_addr(tx_dma_chan, tx_buffer, false);
    dma_channel_set_trans_count(tx_dma_chan, len, true);
    
    total_tx_bytes += len;
    return true;
}

void hw_uart_putc(uint8_t c) {
    // Control text must follow queued binary DMA data, never interleave with it.
    hw_uart_tx_wait();
    uart_putc_raw(HW_UART, c);
    total_tx_bytes++;
}

void hw_uart_puts(const char *str) {
    if (!str) return;
    hw_uart_tx_wait();
    size_t len = strlen(str);
    uart_write_blocking(HW_UART, (const uint8_t *)str, len);
    total_tx_bytes += len;
}

bool hw_uart_rx_available(void) {
    return hw_uart_rx_count() > 0;
}

size_t hw_uart_rx_count(void) {
    return rx_dma_chan < 0 ? 0 : dma_rx_poll(&rx_state, rx_dma_chan);
}

int hw_uart_getc(void) {
    uint8_t byte;
    return hw_uart_read(&byte, 1) == 1 ? byte : -1;
}

size_t hw_uart_read(uint8_t *buf, size_t maxlen) {
    if (rx_dma_chan < 0) return 0;
    size_t n = dma_rx_read(&rx_state, rx_dma_chan, rx_buffer, buf, maxlen);
    total_rx_bytes += n;
    return n;
}

bool hw_uart_rx_take_loss(void) {
    bool lost = rx_reported_loss_epoch != rx_state.loss_epoch;
    rx_reported_loss_epoch = rx_state.loss_epoch;
    return lost;
}

void hw_uart_get_rx_loss_stats(uint64_t *discarded_bytes, uint32_t *ambiguous_polls) {
    if (discarded_bytes) *discarded_bytes = rx_state.discarded_bytes;
    if (ambiguous_polls) *ambiguous_polls = rx_state.ambiguous_polls;
}

bool hw_uart_tx_ready(void) {
    return tx_dma_chan < 0 || !dma_channel_is_busy(tx_dma_chan);
}

void hw_uart_tx_wait(void) {
    while (!hw_uart_tx_ready()) {
        tight_loop_contents();
    }
}

void hw_uart_get_stats(uint32_t *tx_bytes, uint32_t *rx_bytes, uint32_t *overflows) {
    if (tx_bytes) *tx_bytes = total_tx_bytes;
    if (rx_bytes) *rx_bytes = total_rx_bytes;
    if (overflows) *overflows = rx_state.overruns;
}
