
#include "pico/stdlib.h"
#include "activation_driver.h"
#include "uart_driver.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>

#define UART_PORT_NUM   1
#define UART_TX_PIN     8
#define UART_RX_PIN     9
#define UART_BAUDRATE   115200
#define BTN_PIN         20

typedef enum {
    STATE_HANDSHAKE,
    STATE_IDLE,
    STATE_WAIT_TIME
} comm_state_t;

static uint64_t simulated_times[5];
static int time_count = 0;

int main() {
    stdio_init_all();
    printf("=== Slave Pico ===\n");

    activation_driver_init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE);

    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    char recv_buf[128];
    comm_state_t state = STATE_HANDSHAKE;

    while (1) {
        switch (state) {

            // HANDSHAKE PHASE
            case STATE_HANDSHAKE:
                if (activation_receive_line(recv_buf, sizeof(recv_buf))) {
                    if (strncmp(recv_buf, "HELLO", 5) == 0) {
                        activation_send("HI\n");
                        printf("[Slave] Handshake OK\n");
                        state = STATE_IDLE;
                    }
                }
                break;

            // IDLE PHASE
            case STATE_IDLE:
                // Button pressed → Request time
                if (!gpio_get(BTN_PIN)) {
                    printf("[Slave] Button pressed → Requesting time\n");
                    activation_send("GET_TIME\n");
                    state = STATE_WAIT_TIME;
                }

                // Listen for GET_DATA requests or re-handshake
                if (activation_receive_line(recv_buf, sizeof(recv_buf))) {
                    if (strncmp(recv_buf, "GET_DATA", 8) == 0) {
                        printf("[Slave] Received GET_DATA from master\n");
                        if (time_count > 0) {
                            for (int i = 0; i < time_count; i++) {
                                char msg[64];
                                snprintf(msg, sizeof(msg), "DATA %" PRIu64 "\n", simulated_times[i]);
                                activation_send(msg);
                                sleep_ms(200);
                            }
                            activation_send("DONE\n");
                            printf("[Slave] Sent DONE\n");
                            time_count = 0; // reset after sending
                        } else {
                            printf("[Slave] No data stored, ignoring GET_DATA\n");
                            activation_send("ACK_EMPTY\n");
                        }
                    } else if (strncmp(recv_buf, "HELLO", 5) == 0) {
                        activation_send("HI\n");
                    }
                }
                break;

            // WAITING FOR TIME PHASE
            case STATE_WAIT_TIME: {
                absolute_time_t deadline = make_timeout_time_ms(5000);
                bool got_time = false;

                while (!got_time && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
                    if (activation_receive_line(recv_buf, sizeof(recv_buf))) {
                        uint64_t base_time = 0;

                        // Parse "TIME <unix_epoch>"
                        if (sscanf(recv_buf, "TIME %" SCNu64, &base_time) == 1) {
                            printf("[Slave] Received Unix time: %" PRIu64 "\n", base_time);

                            // Optional: show human-readable time
                            time_t t = (time_t) base_time;
                            printf("[Slave] Human time: %s", ctime(&t));

                            // Reset and simulate 5 increments
                            time_count = 0;
                            for (int i = 0; i < 5; i++) {
                                simulated_times[i] = base_time + (i + 1); // 1s increments
                                printf("[Slave] Simulated %" PRIu64 " (+%ds)\n", simulated_times[i], i + 1);
                                sleep_ms(1000);
                                time_count++;
                            }

                            got_time = true;
                            state = STATE_IDLE;
                        } else {
                            printf("[Slave] Failed to parse TIME string: %s\n", recv_buf);
                        }
                    }
                    sleep_ms(50);
                }

                if (!got_time) {
                    printf("[Slave] TIME wait timed out, back to IDLE\n");
                    state = STATE_IDLE;
                }
            } break;
        }

        sleep_ms(50);
    }
}
