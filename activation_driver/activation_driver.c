#include "activation_driver.h"
#include "uart_driver.h"
#include "pico/stdlib.h"
#include <string.h>
#include <stdio.h>

// === Basic UART send/receive ===
void activation_driver_init(int uart_num, int tx_pin, int rx_pin, uint32_t baudrate) {
    uart_driver_init(uart_num, tx_pin, rx_pin, baudrate);
}

void activation_send(const char *message) {
    uart_driver_write((const uint8_t *)message, strlen(message));
}

int activation_receive(char *buffer, size_t max_len) {
    size_t idx = 0;
    while (uart_driver_available() && idx < max_len - 1) {
        buffer[idx++] = uart_driver_read_byte();
        sleep_us(100);
    }
    if (idx > 0) {
        buffer[idx] = '\0';
        return idx;
    }
    return 0;
}

int activation_receive_line(char *out, size_t max_len) {
    static char linebuf[256];
    static size_t len = 0;

    while (uart_driver_available() && len < sizeof(linebuf) - 1) {
        char c = (char)uart_driver_read_byte();
        linebuf[len++] = c;

        if (c == '\n') {
            size_t copy = len < max_len ? len : max_len - 1;
            memcpy(out, linebuf, copy);
            out[copy] = '\0';
            len = 0;
            return (int)copy;
        }
    }
    return 0;
}

