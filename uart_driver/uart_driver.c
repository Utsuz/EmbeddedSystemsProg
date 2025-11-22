#include "uart_driver.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

// === Internal State ===
static uart_inst_t *uart_inst = NULL;

void uart_driver_init(int uart_num, int tx_pin, int rx_pin, uint32_t baudrate) {
    uart_inst = (uart_num == 1) ? uart1 : uart0;
    uart_init(uart_inst, baudrate);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
}

void uart_driver_write(const uint8_t *data, size_t len) {
    if (!uart_inst) return;
    uart_write_blocking(uart_inst, data, len);
}

bool uart_driver_available(void) {
    if (!uart_inst) return false;
    return uart_is_readable(uart_inst);
}

uint8_t uart_driver_read_byte(void) {
    if (!uart_inst) return 0;
    return uart_getc(uart_inst);
}

size_t uart_driver_read(uint8_t *buf, size_t len) {
    if (!uart_inst) return 0;
    for (size_t i = 0; i < len; i++) {
        buf[i] = uart_getc(uart_inst);
    }
    return len;
}

size_t uart_driver_read_timed(uint8_t *buf, size_t len, uint32_t timeout_ms) {
    if (!uart_inst) return 0;
    
    size_t received_count = 0;
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    
    while (received_count < len && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        if (uart_is_readable(uart_inst)) {
            buf[received_count++] = uart_getc(uart_inst);
            // Reset timeout on activity to handle long, valid transfers
            deadline = delayed_by_ms(get_absolute_time(), timeout_ms); 
        } else {
            // Use a brief sleep to allow FreeRTOS scheduling
            sleep_us(100); 
        }
    }
    return received_count;
}
