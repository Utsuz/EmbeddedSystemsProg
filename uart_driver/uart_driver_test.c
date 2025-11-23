#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "uart_driver.h"

// === Configuration ===
#define UART_PORT_NUM   1     // use UART1
#define UART_TX_PIN     8     // GP8 = TX
#define UART_RX_PIN     9     // GP9 = RX
#define UART_BAUDRATE   115200
#define BTN_SEND        20    // press GP20 to send message

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("\n=== UART Driver Function Test ===\n");
    printf("Press GP20 to send message.\n");

    // Initialize UART using driver
    uart_driver_init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE);

    // Initialize button
    gpio_init(BTN_SEND);
    gpio_pull_up(BTN_SEND);

    bool prev_btn = true;
    uint32_t msg_counter = 0;

    while (true) {
        // ==== Send on button press ====
        bool now_btn = gpio_get(BTN_SEND);
        if (!now_btn && prev_btn) {   // falling edge
            char msg[64];
            msg_counter++;
            snprintf(msg, sizeof(msg), "Hello %lu from Pico!\n", (unsigned long)msg_counter);
            uart_driver_write((const uint8_t*)msg, strlen(msg));
            printf("[Local] Sent: %s", msg);
        }
        prev_btn = now_btn;

        // ==== Check for incoming data ====
        while (uart_driver_available()) {
            uint8_t ch = uart_driver_read_byte();
            putchar(ch); // print received char over USB serial
        }

        sleep_ms(10);
    }
}
