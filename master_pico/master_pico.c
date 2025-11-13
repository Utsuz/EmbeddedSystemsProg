#include "pico/stdlib.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h" 
#include "task.h"
#include "semphr.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/uart.h"

#include "usb/usb.h"
#include "ntp_driver.h"
#include "activation_driver.h"
#include "uart_driver.h"
#include "flash_helpers.h"

#include <time.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#define UART_PORT_NUM   1
#define UART_TX_PIN     8
#define UART_RX_PIN     9
#define UART_BAUDRATE   115200
#define BTN_PIN         20
#define TIMEOUT_MS      5000

#define NTP_REFRESH_INTERVAL_MS   (10 * 60 * 1000)  // 10 minutes
#define NTP_FORCE_RESYNC_AGE_US   (15 * 60 * 1000000) // 15 minutes

// --- Master Dump Flash Region Definitions ---
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)

#define MASTER_DUMP_REGION_SIZE (32 * 1024)      // 32 KB for the received blob
#define MASTER_DUMP_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - MASTER_DUMP_REGION_SIZE)
#define MASTER_DUMP_XIP_BASE (XIP_BASE + MASTER_DUMP_FLASH_OFFSET)

#define MASTER_DUMP_MAGIC 0x4D534452u
#define MAX_COMPACT_DUMP_SIZE MASTER_DUMP_REGION_SIZE

// Header structure for the saved data (consistent with slave compact_hdr_t pattern)
typedef struct __attribute__((packed)) {
    uint32_t magic;      /* 'MSDR' for Master Dump Received */
    uint32_t length;     /* number of valid bytes in the dump */
    uint8_t  reserved[FLASH_PAGE_SIZE - 8];
} master_dump_hdr_t;

typedef enum {
    STATE_HANDSHAKE,
    STATE_IDLE,
    STATE_RECEIVING
} comm_state_t;

static SemaphoreHandle_t ntp_mutex;
static time_t last_ntp_time = 0;
static absolute_time_t last_sync_time;
static uint8_t received_data_buffer[MAX_COMPACT_DUMP_SIZE];
static uint32_t received_data_length = 0;

// Master Save Function
void master_save_compact_dump(const uint8_t *data, uint32_t length) {
    if (length == 0 || length > MASTER_DUMP_REGION_SIZE - FLASH_PAGE_SIZE) {
        printf("[Master Flash] ERROR: Data length (%u) invalid or too large.\n", (unsigned)length);
        return;
    }

    // Erase the entire region
    flash_erase_sectors(MASTER_DUMP_FLASH_OFFSET, MASTER_DUMP_REGION_SIZE);

    // Create and Write Header
    master_dump_hdr_t hdr;
    memset(&hdr, 0xFF, sizeof(hdr));
    hdr.magic = MASTER_DUMP_MAGIC;
    hdr.length = length;
    flash_program_block(MASTER_DUMP_FLASH_OFFSET, &hdr, FLASH_PAGE_SIZE);

    // Write Data (starting right after the header page)
    uint32_t written = 0;
    while (written < length) {
        uint8_t page[FLASH_PAGE_SIZE];
        size_t chunk = (length - written > FLASH_PAGE_SIZE) ? FLASH_PAGE_SIZE : (length - written);
        memset(page, 0xFF, sizeof page);
        memcpy(page, &data[written], chunk);

        uint32_t dst = MASTER_DUMP_FLASH_OFFSET + FLASH_PAGE_SIZE + written;
        flash_program_block(dst, page, FLASH_PAGE_SIZE);
        written += (uint32_t)chunk;
    }

    printf("[Master Flash] SUCCESS: Saved %u bytes to flash offset 0x%X.\n", (unsigned)length, (unsigned)MASTER_DUMP_FLASH_OFFSET);
}

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
                    received_data_length = 0; // Reset bufffer tracking
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
                char initial_response[64];
                absolute_time_t start_time = get_absolute_time();
                
                // 1. Wait for the Slave's initial text response (LENGTH, ACK_EMPTY, or GET_TIME)
                if (activation_receive_line(initial_response, sizeof(initial_response))) {
                    uint32_t expected_length = 0;

                    if (sscanf(initial_response, "LENGTH %" PRIu32, &expected_length) == 1) {
                        printf("[Master] Slave preparing to send %u bytes of compact data.\n", (unsigned)expected_length);
                        received_data_length = 0; // Reset buffer counter

                        if (expected_length > MAX_COMPACT_DUMP_SIZE) {
                            printf("[Master] ERROR: Expected length %u exceeds MAX_DUMP_SIZE. Aborting.\n", (unsigned)expected_length);
                            state = STATE_IDLE;
                            break;
                        }

                        // 2. Start Binary Data Reception
                        uint32_t received_count = 0;
                        start_time = get_absolute_time(); // Reset timeout for binary read

                        // Read raw bytes until expected_length is met or timeout
                        while (received_count < expected_length &&
                                absolute_time_diff_us(get_absolute_time(), start_time) > -(TIMEOUT_MS * 1000)) {

                            // Use non-blocking read to allow FreeRTOS context switching
                            if (uart_driver_available()) {
                                received_data_buffer[received_count++] = uart_driver_read_byte();
                                start_time = get_absolute_time(); // Reset timeout on activity
                            } else {
                                // Wait briefly to avoid busy-waiting, allowing other tasks to run
                                vTaskDelay(pdMS_TO_TICKS(1)); 
                            }
                        }
                        received_data_length = received_count;
                        // End Binary Data Reception

                        if (received_data_length == expected_length) {
                            printf("[Master] SUCCESS: Received %u / %u bytes. Waiting for DONE message.\n",
                                   (unsigned)received_data_length, (unsigned)expected_length);

                            // Wait for the Slave's final "DONE" text message
                            if (activation_receive_line(initial_response, sizeof(initial_response)) &&
                                strncmp(initial_response, "DONE", 4) == 0) {
                                printf("[Master] Data transfer complete and acknowledged by Slave.\n");
                            } else {
                                printf("[Master] WARNING: Received data but no proper DONE signal. Response: %s\n", initial_response);
                            }

                            // Save the received data to internal flash
                            master_save_compact_dump(received_data_buffer, received_data_length);

                            state = STATE_IDLE;
                            break;

                        } else {
                            printf("[Master] ERROR: Binary transfer failed. Expected %u, got %u. Returning to idle.\n",
                                   (unsigned)expected_length, (unsigned)received_data_length);
                            state = STATE_IDLE;
                            break;
                        }

                    } else if (strncmp(initial_response, "ACK_EMPTY", 9) == 0) {
                        printf("[Master] Slave log is empty. Returning to idle.\n");
                        state = STATE_IDLE;
                        break;
                    } else if (strncmp(initial_response, "GET_TIME", 8) == 0) {
                        // Handle GET_TIME if the Slave asks for time during the transfer phase
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
                            snprintf(msg, sizeof(msg), "TIME %" PRIu64 "\n", (uint64_t)current_time);
                            activation_send(msg);
                            printf("[Master] Sent current NTP time: %s", ctime(&current_time));
                        }
                    } else {
                        printf("[Master] Unexpected response in RECEIVING state: %s. Returning to idle.\n", initial_response);
                        state = STATE_IDLE;
                        break;
                    }
                }

                // If no response at all (initial text timeout)
                if (absolute_time_diff_us(get_absolute_time(), start_time) <= -(TIMEOUT_MS * 1000)) {
                    printf("[Master] No initial response from slave (timeout after %d ms). Returning to idle.\n", TIMEOUT_MS);
                    state = STATE_IDLE;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
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

