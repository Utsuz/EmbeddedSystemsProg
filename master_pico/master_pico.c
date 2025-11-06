#include "pico/stdlib.h"
#include "activation_driver.h"
#include "uart_driver.h"
#include <stdio.h>
#include <string.h>

#define UART_PORT_NUM   1
#define UART_TX_PIN     8
#define UART_RX_PIN     9
#define UART_BAUDRATE   115200
#define BTN_PIN         20
#define TIMEOUT_MS      5000

typedef enum {
    STATE_HANDSHAKE,
    STATE_IDLE,
    STATE_RECEIVING
} comm_state_t;

int main() {
    stdio_init_all();
    printf("=== Master Pico ===\n");

    activation_driver_init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE);

    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    char recv_buf[128];
    comm_state_t state = STATE_HANDSHAKE;

    while (1) {
        switch (state) {
            // --- HANDSHAKE PHASE ---
            case STATE_HANDSHAKE:
                activation_send("HELLO\n");
                if (activation_receive_line(recv_buf, sizeof(recv_buf))) {
                    if (strncmp(recv_buf, "HI", 2) == 0) {
                        printf("[Master] Handshake OK\n");
                        state = STATE_IDLE;
                    }
                }
                sleep_ms(500);
                break;

            // --- IDLE PHASE ---
            case STATE_IDLE:
                // Button pressed → Request data
                if (!gpio_get(BTN_PIN)) {
                    printf("[Master] Button pressed → Requesting data\n");
                    activation_send("GET_DATA\n");
                    sleep_ms(300);
                    state = STATE_RECEIVING;
                }

                // Always listen for GET_TIME or re-handshake
                if (activation_receive_line(recv_buf, sizeof(recv_buf))) {
                    if (strncmp(recv_buf, "GET_TIME", 8) == 0) {
                        absolute_time_t now = get_absolute_time();
                        uint64_t ms = to_ms_since_boot(now);
                        char msg[64];
                        snprintf(msg, sizeof(msg), "TIME %llu\n", ms);
                        activation_send(msg);
                        printf("[Master] Sent current time: %llu\n", ms);
                    } else if (strncmp(recv_buf, "HELLO", 5) == 0) {
                        activation_send("HI\n");
                    }
                }
                break;

            // --- RECEIVING PHASE ---
            case STATE_RECEIVING: {
                absolute_time_t start_time = get_absolute_time();
                bool got_response = false;

                while (absolute_time_diff_us(get_absolute_time(), start_time) > -(TIMEOUT_MS * 1000)) {
                    if (activation_receive_line(recv_buf, sizeof(recv_buf))) {
                        got_response = true;

                        if (strncmp(recv_buf, "DATA", 4) == 0) {
                            printf("[Master] %s", recv_buf);
                        } else if (strncmp(recv_buf, "DONE", 4) == 0) {
                            printf("[Master] Data transfer complete.\n");
                            state = STATE_IDLE;
                            break;
                        } else if (strncmp(recv_buf, "ACK_EMPTY", 9) == 0) {
                            printf("[Master] Slave not ready yet. Returning to idle.\n");
                            state = STATE_IDLE;
                            break;
                        } else if (strncmp(recv_buf, "GET_TIME", 8) == 0) {
                            absolute_time_t now = get_absolute_time();
                            uint64_t ms = to_ms_since_boot(now);
                            char msg[64];
                            snprintf(msg, sizeof(msg), "TIME %llu\n", ms);
                            activation_send(msg);
                            printf("[Master] Sent current time: %llu\n", ms);
                        }

                        // Restart timeout timer after valid line
                        start_time = get_absolute_time();
                    }

                    sleep_ms(50);

                    // If transfer finished, break early
                    if (state == STATE_IDLE) break;
                }

                if (state != STATE_IDLE) {
                    if (!got_response) {
                        printf("[Master] No response from slave (timeout after %d ms). Returning to idle.\n", TIMEOUT_MS);
                    } else {
                        printf("[Master] Incomplete exchange timed out, returning to idle.\n");
                    }
                    state = STATE_IDLE;
                }
                break;
            }
        }
        sleep_ms(50);
    }
}

