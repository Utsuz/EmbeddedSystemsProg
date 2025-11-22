#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// === Public API ===

// Initialize UART (select port, pins, baudrate)
void uart_driver_init(int uart_num, int tx_pin, int rx_pin, uint32_t baudrate);

// Send a raw buffer
void uart_driver_write(const uint8_t *data, size_t len);

// Check if data is available to read
bool uart_driver_available(void);

// Read one byte (blocking)
uint8_t uart_driver_read_byte(void);

// Read into buffer (blocking until len received)
size_t uart_driver_read(uint8_t *buf, size_t len);

// Allowing uart to wait for all bytes to be read
size_t uart_driver_read_timed(uint8_t *buf, size_t len, uint32_t timeout_ms);
