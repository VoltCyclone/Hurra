/**
 * DMA Ring Buffer Helpers for UART RX
 *
 * Shared inline utilities for reading from a DMA-driven circular receive
 * buffer. RP2350 self-trigger mode exposes a decrementing byte counter while
 * continuously reloading. The tracker distinguishes complete buffer laps,
 * validates copies, and discards damaged input until an idle gap.
 *
 * Used by both the KMBox serial handler and the bridge HW UART driver.
 */

#ifndef DMA_UART_H
#define DMA_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "hardware/dma.h"
#include "hardware/regs/uart.h"
#include "hardware/sync.h"
#include "pico/time.h"

// RP2350 self-trigger mode reloads this count while retaining ring addresses.
// A power of two makes count reloads and 32-bit software totals wrap safely.
#define DMA_RX_TRANSFER_COUNT (1u << 27)
#define DMA_RX_TRANSFER_MASK (DMA_RX_TRANSFER_COUNT - 1u)
#define DMA_RX_IDLE_US 1000u

typedef struct {
    uint32_t produced;
    uint32_t consumed;
    uint32_t remaining;
    uint32_t mask;
    uint32_t loss_epoch;
    uint32_t overruns;
    uint32_t ambiguous_polls;
    uint64_t discarded_bytes; // Known discarded bytes; excludes unknown count laps.
    uint64_t sampled_us;
    uint64_t progress_us;
    uint64_t max_poll_gap_us;
    volatile const uint32_t *fifo_status;
    bool resync;
    bool sample_valid;
} dma_rx_state_t;

static inline void dma_rx_init(dma_rx_state_t *s, uint32_t mask,
                               uint32_t read_pos, uint32_t write_pos,
                               uint32_t baud, uint64_t now_us) {
    memset(s, 0, sizeof(*s));
    s->mask = mask;
    s->consumed = read_pos;
    s->produced = read_pos + ((write_pos - read_pos) & mask);
    s->remaining = 0; // TRANSFER_COUNT & TRANSFER_MASK at the initial reload.
    s->sampled_us = s->progress_us = now_us;
    // Conservative upper bound: eight bit times per received byte, plus room
    // for a full UART FIFO. A stale sample cannot silently alias a count lap.
    s->max_poll_gap_us = baud ? ((uint64_t)(DMA_RX_TRANSFER_COUNT - 64u) * 8000000u) / baud : 0;
}

/** Sample a live count, retaining absolute unread length across ring laps. */
static inline uint32_t dma_rx_update(dma_rx_state_t *s, uint32_t remaining,
                                     uint64_t now_us) {
    s->sample_valid = true;
    remaining &= DMA_RX_TRANSFER_MASK; // Strip mode bits and normalize reload/zero.
    uint32_t delta = (s->remaining - remaining) & DMA_RX_TRANSFER_MASK;
    bool ambiguous = now_us - s->sampled_us >= s->max_poll_gap_us;
    s->remaining = remaining;
    s->sampled_us = now_us;
    s->produced += delta;
    if (delta) s->progress_us = now_us;
    uint32_t unread = s->produced - s->consumed;
    // Reserve a slot for a possibly in-flight DMA write. Full is never empty.
    if (ambiguous || unread > s->mask) {
        s->overruns++;
        if (ambiguous) s->ambiguous_polls++;
        s->loss_epoch++;
        s->resync = true;
        s->progress_us = now_us;
    }
    if (s->resync) {
        s->discarded_bytes += unread;
        s->consumed = s->produced;
        if (!delta && now_us - s->progress_us >= DMA_RX_IDLE_US) s->resync = false;
        return 0;
    }
    return unread;
}

static inline uint32_t dma_rx_poll(dma_rx_state_t *s, int channel) {
    // WRITE_ADDR updates on completed writes. Match it to the count so an
    // in-flight bus beat or a torn snapshot cannot publish unwritten bytes.
    // Buffers must be aligned to mask+1, as required by DMA ring addressing.
    for (unsigned attempt = 0; attempt < 3; attempt++) {
        uint64_t before = time_us_64();
        uint32_t remaining = dma_channel_hw_addr(channel)->transfer_count & DMA_RX_TRANSFER_MASK;
        uintptr_t wa = dma_channel_hw_addr(channel)->write_addr;
        bool fifo_pending = s->fifo_status && !(*s->fifo_status & UART_UARTFR_RXFE_BITS);
        uint64_t after = time_us_64();
        if ((dma_channel_hw_addr(channel)->transfer_count & DMA_RX_TRANSFER_MASK) != remaining) continue;
        uint32_t delta = (s->remaining - remaining) & DMA_RX_TRANSFER_MASK;
        if ((wa & s->mask) != ((s->produced + delta) & s->mask)) continue;
        if (fifo_pending) s->progress_us = after;
        __dmb();
        uint32_t avail = dma_rx_update(s, remaining, after);
        // Bracket the reads: a scheduling stall on either side must not hide a lap.
        s->sampled_us = before;
        return avail;
    }
    s->sample_valid = false;
    return 0; // Retry next task pass rather than spin or publish uncertain data.
}

/** Copy without consuming. Recheck after copying so a concurrent lap is rejected. */
static inline bool dma_rx_peek_copy(dma_rx_state_t *s, int channel,
                                    const uint8_t *buffer, uint8_t *dest,
                                    size_t offset, size_t len) {
    uint32_t avail = dma_rx_poll(s, channel);
    if (!s->sample_valid || s->resync || offset > avail || len > avail - offset) return false;
    uint32_t start = s->consumed;
    uint32_t epoch = s->loss_epoch;
    if (len) {
        uint32_t pos = (start + (uint32_t)offset) & s->mask;
        size_t first = s->mask + 1u - pos;
        if (first > len) first = len;
        memcpy(dest, buffer + pos, first);
        if (len > first) memcpy(dest + first, buffer, len - first);
    }
    __dmb();
    dma_rx_poll(s, channel);
    return s->sample_valid && !s->resync && s->loss_epoch == epoch && s->consumed == start;
}

static inline size_t dma_rx_read(dma_rx_state_t *s, int channel,
                                 const uint8_t *buffer, uint8_t *dest, size_t maxlen) {
    if (!dest || !maxlen) return 0;
    uint32_t avail = dma_rx_poll(s, channel);
    size_t len = avail < maxlen ? avail : maxlen;
    if (!len || !dma_rx_peek_copy(s, channel, buffer, dest, 0, len)) return 0;
    s->consumed += (uint32_t)len;
    return len;
}

// Legacy index-only helpers below cannot detect producer laps. Receive paths
// must use the tracked API above; these remain for index arithmetic compatibility.

/** Get the current DMA write position in the ring buffer. */
static inline uint32_t dma_ring_write_pos(int dma_chan,
                                           const uint8_t *buf,
                                           uint32_t mask) {
    if (dma_chan < 0) return 0;
    uintptr_t wa = (uintptr_t)dma_channel_hw_addr(dma_chan)->write_addr;
    return (uint32_t)((wa - (uintptr_t)buf) & mask);
}

/** Number of unread bytes in the ring buffer. */
static inline uint32_t dma_ring_available(uint32_t read_pos,
                                           uint32_t write_pos,
                                           uint32_t mask) {
    return (write_pos - read_pos) & mask;
}

/** Peek at a byte at `offset` from the current read position. */
static inline uint8_t dma_ring_peek(const uint8_t *buf,
                                     uint32_t read_pos,
                                     uint32_t offset,
                                     uint32_t mask) {
    return buf[(read_pos + offset) & mask];
}

/**
 * Copy up to `maxlen` bytes out of the ring buffer into `dest`.
 * Updates `*read_pos` to reflect consumed bytes.
 * Returns number of bytes actually copied.
 */
static inline size_t dma_ring_read(const uint8_t *buf,
                                    uint32_t *read_pos,
                                    uint32_t write_pos,
                                    uint32_t mask,
                                    uint8_t *dest,
                                    size_t maxlen) {
    uint32_t avail = dma_ring_available(*read_pos, write_pos, mask);
    size_t to_read = (avail < maxlen) ? avail : maxlen;
    if (to_read == 0) return 0;

    uint32_t rp   = *read_pos;
    uint32_t size = mask + 1;
    size_t first  = size - rp;
    if (first > to_read) first = to_read;

    memcpy(dest, &buf[rp], first);
    if (to_read > first) {
        memcpy(dest + first, buf, to_read - first);
    }

    *read_pos = (rp + (uint32_t)to_read) & mask;
    return to_read;
}

#endif // DMA_UART_H
