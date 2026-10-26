"""Small SDK register/time stubs shared by the DMA receive regression tests."""
from pathlib import Path


def write_headers(directory):
    directory = Path(directory)
    (directory / 'hardware').mkdir(exist_ok=True)
    (directory / 'pico').mkdir(exist_ok=True)
    (directory / 'hardware/regs').mkdir(exist_ok=True)
    (directory / 'hardware/regs/uart.h').write_text('#pragma once\n#define UART_UARTFR_RXFE_BITS (1u << 4)\n')
    (directory / 'hardware/dma.h').write_text('''
#pragma once
#include <stdint.h>
static struct {uintptr_t write_addr; uint32_t transfer_count;} mock_dma;
#define dma_channel_hw_addr(channel) ((void)(channel), &mock_dma)
''')
    (directory / 'hardware/sync.h').write_text('''
#pragma once
static inline void __dmb(void) {__asm__ volatile ("" ::: "memory");}
''')
    (directory / 'pico/time.h').write_text('''
#pragma once
#include <stdint.h>
static uint64_t mock_us;
static uint64_t mock_time_step;
static void (*mock_time_hook)(void);
static inline uint64_t time_us_64(void) {
    uint64_t now = mock_us;
    mock_us += mock_time_step;
    if (mock_time_hook) mock_time_hook();
    return now;
}
''')
