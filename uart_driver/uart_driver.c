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
