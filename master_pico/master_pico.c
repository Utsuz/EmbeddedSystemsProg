#include "pico/stdlib.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h" 
#include "task.h"
#include "semphr.h"

#include "usb/usb.h"
#include "ntp_driver.h"
#include "activation_driver.h"
#include "uart_driver.h"

#include <time.h>
#include <stdio.h>
#include <string.h>

#define UART_PORT_NUM   1
#define UART_TX_PIN     8
#define UART_RX_PIN     9
#define UART_BAUDRATE   115200
#define BTN_PIN         20
#define TIMEOUT_MS      5000

#define NTP_REFRESH_INTERVAL_MS   (10 * 60 * 1000)  // 10 minutes
#define NTP_FORCE_RESYNC_AGE_US   (15 * 60 * 1000000) // 15 minutes

typedef enum {
    STATE_HANDSHAKE,
    STATE_IDLE,
    STATE_RECEIVING
} comm_state_t;

static SemaphoreHandle_t ntp_mutex;
static time_t last_ntp_time = 0;
static absolute_time_t last_sync_time;

// FreeRTOS Wi-Fi + NTP
static void wifi_ntp_task(void *pvParameters) {
    stdio_init_all();
    printf("=== Wi-Fi + NTP Background Task ===\n");

    if (cyw43_arch_init_with_country(CYW43_COUNTRY_SINGAPORE)) {
        printf("[NTP] Wi-Fi init failed!\n");
        vTaskDelete(NULL);
    }

    cyw43_arch_enable_sta_mode();
    printf("[NTP] Connecting to Wi-Fi SSID: %s\n", WIFI_SSID);

    if (cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("[NTP] Wi-Fi connection failed.\n");
        vTaskDelete(NULL);
    }

    printf("[NTP] Connected to Wi-Fi!\n");

    for (;;) {
        time_t now = ntp_get_time();
        if (now > 0) {
            xSemaphoreTake(ntp_mutex, portMAX_DELAY);
            last_ntp_time = now;
            last_sync_time = get_absolute_time();
            xSemaphoreGive(ntp_mutex);
            printf("[NTP] Synced time: %s", ctime(&now));
        } else {
            printf("[NTP] Sync failed.\n");
        }

        vTaskDelay(pdMS_TO_TICKS(NTP_REFRESH_INTERVAL_MS));
    }
}

// UART + Activation Driver Task
static void uart_activation_task(void *pvParameters) {
    char recv_buf[128];
    comm_state_t state = STATE_HANDSHAKE;

    activation_driver_init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE);
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);

    printf("=== Master Pico (UART + NTP integrated) ===\n");

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
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            // --- IDLE PHASE ---
            case STATE_IDLE:
                if (!gpio_get(BTN_PIN)) {
                    printf("[Master] Button pressed → Requesting data\n");
                    activation_send("GET_DATA\n");
                    vTaskDelay(pdMS_TO_TICKS(300));
                    state = STATE_RECEIVING;
                }

                if (activation_receive_line(recv_buf, sizeof(recv_buf))) {
                    // Slave asking for current time
                    if (strncmp(recv_buf, "GET_TIME", 8) == 0) {
                        time_t current_time = 0;
                        bool need_resync = false;

                        xSemaphoreTake(ntp_mutex, portMAX_DELAY);
                        if (last_ntp_time == 0) {
                            need_resync = true;
                        } else {
                            // Check age of cached time
                            int64_t age_us = absolute_time_diff_us(get_absolute_time(), last_sync_time);
                            if (age_us < -NTP_FORCE_RESYNC_AGE_US) {
                                need_resync = true;
                            }
                        }
                        xSemaphoreGive(ntp_mutex);

                        if (need_resync) {
                            printf("[Master] Cached NTP time stale → refreshing...\n");
                            time_t now = ntp_get_time();
                            if (now > 0) {
                                xSemaphoreTake(ntp_mutex, portMAX_DELAY);
                                last_ntp_time = now;
                                last_sync_time = get_absolute_time();
                                xSemaphoreGive(ntp_mutex);
                                printf("[Master] Re-synced: %s", ctime(&now));
                            } else {
                                printf("[Master] Failed to refresh NTP time.\n");
                            }
                        }

                        // compute current time using elapsed ticks
                        xSemaphoreTake(ntp_mutex, portMAX_DELAY);
                        if (last_ntp_time > 0) {
                            uint64_t delta_ms = to_ms_since_boot(get_absolute_time()) -
                                                to_ms_since_boot(last_sync_time);
                            current_time = last_ntp_time + (delta_ms / 1000);
                        }
                        xSemaphoreGive(ntp_mutex);

                        if (current_time > 0) {
                            char msg[64];
                            snprintf(msg, sizeof(msg), "TIME %" PRIu64 "\n", (uint64_t) current_time);
                            activation_send(msg);
                            printf("[Master] Sent current NTP time: %s", ctime(&current_time));
                        } else {
                            activation_send("TIME_ERR\n");
                            printf("[Master] No valid time to send.\n");
                        }
                    }
                    else if (strncmp(recv_buf, "HELLO", 5) == 0) {
                        activation_send("HI\n");
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(50));
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
                            // Handle GET_TIME here again if needed
                            time_t current_time = 0;
                            xSemaphoreTake(ntp_mutex, portMAX_DELAY);
                            if (last_ntp_time > 0) {
                                uint64_t delta_ms = to_ms_since_boot(get_absolute_time()) -
                                                    to_ms_since_boot(last_sync_time);
                                current_time = last_ntp_time + (delta_ms / 1000);
                            }
                            xSemaphoreGive(ntp_mutex);

                            if (current_time > 0) {
                                char msg[64];
                                snprintf(msg, sizeof(msg), "TIME %ld\n", current_time);
                                activation_send(msg);
                                printf("[Master] Sent current NTP time: %s", ctime(&current_time));
                            }
                        }
                        start_time = get_absolute_time();
                    }

                    vTaskDelay(pdMS_TO_TICKS(50));
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

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main() {
    stdio_init_all();
    ntp_mutex = xSemaphoreCreateMutex();

    // Start background Wi-Fi/NTP sync and UART protocol task
    xTaskCreate(wifi_ntp_task, "wifi_ntp_task", 4096, NULL, 2, NULL);
    xTaskCreate(uart_activation_task, "uart_activation_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1) tight_loop_contents();
}

