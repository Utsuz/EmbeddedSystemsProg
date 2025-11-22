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
#include "hmac_sha256.h"

#include <time.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/* ========================================= */
/* ===== 1. CONSTANTS AND DEFINITIONS ===== */
/* ========================================= */

#define UART_PORT_NUM   1
#define UART_TX_PIN     8
#define UART_RX_PIN     9
#define UART_BAUDRATE   115200
#define BTN_PIN         20 // Request data
#define DUMP_BTN_PIN    21 // Dump saved data to PC
#define WIPE_BTN_PIN    22 // WIPE saved data
#define TIMEOUT_MS      5000
#define WIFI_MAX_RETRIES 5

#define NTP_REFRESH_INTERVAL_MS   (10 * 60 * 1000)  // 10 minutes
#define NTP_FORCE_RESYNC_AGE_US   (15 * 60 * 1000000) // 15 minutes

// -- Binary Protocol Definitions (Must match slave_pico.c) --
#define DONE_CHAR         0x06
#define CMD_HELLO         0x11
#define CMD_HI_ACK        0x12
#define CMD_GET_TIME      0x30 
#define CMD_TIME_RSP      0x31 
#define ACK_EMPTY_LOG     0x40
#define CMD_REQ_DATA      0x21
#define CMD_SEND_DATA_HDR 0x20
#define BINARY_HEADER_SIZE 3 // 1 byte CMD + 2 byte Length
#define BINARY_TIME_SIZE   9 // 1 byte CMD + 8 byte Unix time

// -- Security Key Definition (Must match slave_pico.c) --
#define HMAC_KEY "INF2004_TEMP_LOG_KEY"
#define HMAC_LEN 32

// --- Master Dump Flash Region Definitions ---
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#define MASTER_DUMP_REGION_SIZE (32 * 1024)
#define MASTER_DUMP_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - MASTER_DUMP_REGION_SIZE)
#define MASTER_DUMP_XIP_BASE (XIP_BASE + MASTER_DUMP_FLASH_OFFSET)
#define MASTER_DUMP_MAGIC 0x4D534452u
#define MAX_COMPACT_DUMP_SIZE (MASTER_DUMP_REGION_SIZE - FLASH_PAGE_SIZE)

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t length;
    uint8_t  reserved[FLASH_PAGE_SIZE - 8];
} master_dump_hdr_t;

typedef enum {
    STATE_HANDSHAKE,
    STATE_IDLE,
    STATE_RECEIVING
} comm_state_t;

/* ========================================= */
/* ===== 2. GLOBAL STATE & PROTOTYPES ===== */
/* ========================================= */

static SemaphoreHandle_t ntp_mutex;
static time_t last_ntp_time = 0;
static absolute_time_t last_sync_time;
static uint8_t received_data_buffer[MAX_COMPACT_DUMP_SIZE];
static uint32_t received_data_length = 0;

// Internal prototypes
static void print_hex(const uint8_t *b, size_t n, const char *prefix);
static void master_dump_saved_data(void);
static void master_wipe_saved_data(void);
static bool handle_user_buttons(comm_state_t *state_ptr);
static void handle_time_request(void);
static void handle_receiving_state(comm_state_t *state_ptr);
void master_save_compact_dump(const uint8_t *data, uint32_t length); // Retaining external visibility

/* ========================================= */
/* ===== 3. HELPER & UTILITY FUNCTIONS ===== */
/* ========================================= */

static inline const master_dump_hdr_t *xip_hdr_master_dump(void) {
    return (const master_dump_hdr_t *)(MASTER_DUMP_XIP_BASE);
}
static inline const uint8_t *xip_data_master_dump(void) {
    return (const uint8_t *)(MASTER_DUMP_XIP_BASE + FLASH_PAGE_SIZE);
}

static void print_hex(const uint8_t *b, size_t n, const char *prefix) {
  printf("%s", prefix);
  for (size_t i = 0; i < n; ++i) {
    printf("%02x", b[i]);
  }
  printf("\n");
}

static void master_wipe_saved_data(void) {
    printf("[Master Wipe] WARNING: Erasing Master Dump Flash Region (0x%X)...\n", 
           (unsigned)MASTER_DUMP_FLASH_OFFSET);
    flash_erase_sectors(MASTER_DUMP_FLASH_OFFSET, MASTER_DUMP_REGION_SIZE);

    master_dump_hdr_t hdr;
    memset(&hdr, 0xFF, sizeof(hdr));
    hdr.magic = 0x00000000u;
    hdr.length = 0;
    flash_program_block(MASTER_DUMP_FLASH_OFFSET, &hdr, FLASH_PAGE_SIZE);

    printf("[Master Wipe] SUCCESS: Data region wiped.\n");
}

static void master_dump_saved_data(void) {
    if (!usb_is_connected()) {
        printf("[Master Dump] USB not connected. Aborting dump.\n");
        return;
    }
    const master_dump_hdr_t *h = xip_hdr_master_dump();
    if (h->magic != MASTER_DUMP_MAGIC) {
        printf("[Master Dump] No valid data blob found in flash (Magic: 0x%X).\n", (unsigned)h->magic);
        usb_send("[DUMP] No valid data found in flash.\n");
        return;
    }
    const uint32_t length = h->length;
    if (length == 0 || length > MAX_COMPACT_DUMP_SIZE) {
        printf("[Master Dump] Invalid data length: %u bytes.\n", (unsigned)length);
        usb_send("[DUMP] Invalid data length recorded.\n");
        return;
    }
    printf("[Master Dump] Dumping %u bytes of compact data over USB...\n", (unsigned)length);
    char info_msg[128];
    snprintf(info_msg, sizeof(info_msg), "\n[DUMP_START] Compact blob size: %u bytes.\n", (unsigned)length);
    usb_send(info_msg);
    
    const uint8_t *data_ptr = xip_data_master_dump();
    usb_send_raw(data_ptr, length);
    
    usb_send("\n[DUMP_END] Transfer complete.\n\n");
    printf("[Master Dump] Dump successful.\n");
}

void master_save_compact_dump(const uint8_t *data, uint32_t length) {
    if (length == 0 || length > MAX_COMPACT_DUMP_SIZE) {
        printf("[Master Flash] ERROR: Data length (%u) invalid or too large.\n", (unsigned)length);
        return;
    }
    flash_erase_sectors(MASTER_DUMP_FLASH_OFFSET, MASTER_DUMP_REGION_SIZE);
    master_dump_hdr_t hdr;
    memset(&hdr, 0xFF, sizeof(hdr));
    hdr.magic = MASTER_DUMP_MAGIC;
    hdr.length = length;
    flash_program_block(MASTER_DUMP_FLASH_OFFSET, &hdr, FLASH_PAGE_SIZE);

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


/* ========================================= */
/* ===== 4. STATE HANDLERS (REFACTORED) ===== */
/* ========================================= */

// Handles GP21 (Dump) and GP22 (Wipe) buttons
static bool handle_user_buttons(comm_state_t *state_ptr) {
    if (*state_ptr != STATE_IDLE) return false;

    // --- Check for Dump Button Press (GP21) ---
    if (!gpio_get(DUMP_BTN_PIN)) {
        printf("[Master] Dump button (GP21) pressed → DUMPING SAVED DATA\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        master_dump_saved_data();
        while (!gpio_get(DUMP_BTN_PIN)) vTaskDelay(pdMS_TO_TICKS(50));
        vTaskDelay(pdMS_TO_TICKS(100));
        return true;
    }

    // --- Check for Wipe Button Press (GP22) ---
    if (!gpio_get(WIPE_BTN_PIN)) {
        printf("[Master] Wipe button (GP22) pressed → WIPING SAVED DATA\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        master_wipe_saved_data();
        while (!gpio_get(WIPE_BTN_PIN)) vTaskDelay(pdMS_TO_TICKS(50));
        vTaskDelay(pdMS_TO_TICKS(100));
        return true;
    }
    return false;
}

// Handles incoming GET_TIME requests from Slave
static void handle_time_request(void) {
    uint8_t recv_cmd = 0;
    
    // Check if any data is available; if not, exit cleanly.
    if (!uart_driver_available()) return;
    
    // Read the command byte directly (Replaces activation_receive_line and recv_buf)
    recv_cmd = uart_driver_read_byte();

    if (recv_cmd == CMD_GET_TIME) {
        time_t current_time = 0;
        bool need_resync = false;

        // --- 1. Check/Refresh NTP Time ---
        // (Existing mutex logic remains the same)
        xSemaphoreTake(ntp_mutex, portMAX_DELAY);
        if (last_ntp_time == 0) {
            need_resync = true;
        } else {
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

        // --- 2. Compute Current Time ---
        xSemaphoreTake(ntp_mutex, portMAX_DELAY);
        if (last_ntp_time > 0) {
            uint64_t delta_ms = to_ms_since_boot(get_absolute_time()) -
                                to_ms_since_boot(last_sync_time);
            current_time = last_ntp_time + (delta_ms / 1000);
        }
        xSemaphoreGive(ntp_mutex);

        // --- 3. Send Binary Time Response (CMD + 8-byte Unix time) ---
        if (current_time > 0) {
            uint8_t tx_buf[BINARY_TIME_SIZE]; // 1 byte CMD + 8 bytes time
            uint64_t time_val = (uint64_t)current_time;
            
            tx_buf[0] = CMD_TIME_RSP;
            memcpy(&tx_buf[1], &time_val, sizeof(uint64_t)); // Copy 8 bytes of time
            
            uart_driver_write(tx_buf, BINARY_TIME_SIZE);
            printf("[Master] Sent current NTP time: %s", ctime(&current_time));
        } else {
            // If time unavailable, send an error ACK
            uart_putc_raw(uart_get_instance(UART_PORT_NUM), ACK_EMPTY_LOG);
            printf("[Master] No valid time to send (Sent ACK_EMPTY).\n");
        }
    }
    // Handle binary handshake request
    else if (recv_cmd == CMD_HELLO) {
        uart_putc_raw(uart_get_instance(UART_PORT_NUM), CMD_HI_ACK);
        printf("[Master] Received HELLO, Sent HI ACK.\n");
    }
    // Handle any other stray bytes received by silently discarding them
    else if (recv_cmd != 0) {
        printf("[Master] WARN: Discarding stray byte 0x%02X in IDLE.\n", recv_cmd);
    }
}

// Handles the entire Binary Data Transfer process
static void handle_receiving_state(comm_state_t *state_ptr) {
    uint8_t header_buf[BINARY_HEADER_SIZE];
    uint32_t expected_length = 0;
    
    // Read Fixed Binary Header using timed read
    // Note: Assumes uart_driver_read_timed is available in uart_driver.h/c
    size_t bytes_read = uart_driver_read_timed(header_buf, BINARY_HEADER_SIZE, TIMEOUT_MS);

    if (bytes_read != BINARY_HEADER_SIZE) {
        printf("[Master] ERROR: Failed to read full binary header (Incomplete/Timeout: %zu/%d). Returning to idle.\n", bytes_read, BINARY_HEADER_SIZE);
        *state_ptr = STATE_IDLE;
        return;
    }

    if (header_buf[0] == CMD_SEND_DATA_HDR) {
        // Parse Length (Little-endian: LSB | MSB)
        expected_length = (uint32_t)(header_buf[1] | (header_buf[2] << 8)); 

        printf("[Master] Slave preparing to send %u bytes of compact data.\n", (unsigned)expected_length);
        received_data_length = 0; 

        if (expected_length == 0) {
            printf("[Master] Slave log empty. Returning to idle.\n");
            *state_ptr = STATE_IDLE;
            return;
        }
        if (expected_length > MAX_COMPACT_DUMP_SIZE) {
            printf("[Master] ERROR: Expected length %u exceeds MAX_DUMP_SIZE. Aborting.\n", (unsigned)expected_length);
            *state_ptr = STATE_IDLE;
            return;
        }

        // Read Binary Data Payload (Content to be hashed)
        received_data_length = uart_driver_read_timed(received_data_buffer, expected_length, TIMEOUT_MS);

        if (received_data_length == expected_length) {
            printf("[Master] SUCCESS: Received %u / %u bytes. Waiting for DONE signal.\n",
                   (unsigned)received_data_length, (unsigned)expected_length);

            // Read DONE_CHAR (Tail) using timed read
            uint8_t done_signal = 0;
            size_t done_read = uart_driver_read_timed(&done_signal, 1, TIMEOUT_MS); 

            if (done_read == 1 && done_signal == DONE_CHAR) {
                printf("[Master] Data transfer complete and acknowledged by Slave (ACK: 0x%02X).\n", DONE_CHAR);
                
                // Compute HMAC on the received Payload ONLY
                uint8_t computed_mac[HMAC_LEN];
                hmac_sha256((const uint8_t *)HMAC_KEY, strlen(HMAC_KEY), received_data_buffer, received_data_length, computed_mac);
                
                // Log the computed hash for physical comparison
                print_hex(computed_mac, HMAC_LEN, "[Master] Calculated HMAC: ");
                
                // Save the received data (Payload) to internal flash
                master_save_compact_dump(received_data_buffer, received_data_length);
                
            } else {
                printf("[Master] WARNING: Received data but no proper DONE signal. Received: 0x%02X (Count: %zu). Aborting HMAC.\n", done_signal, done_read);
            }

            *state_ptr = STATE_IDLE;
            return;

        } else {
            printf("[Master] ERROR: Binary transfer failed. Expected %u, got %u. Returning to idle.\n",
                   (unsigned)expected_length, (unsigned)received_data_length);
            *state_ptr = STATE_IDLE;
            return;
        }

    } else { // Handle unexpected header (e.g., garbage, ACK_EMPTY, or stray GET_TIME command)
        printf("[Master] Unexpected binary command 0x%02X. Returning to idle.\n", header_buf[0]);
        *state_ptr = STATE_IDLE;
        return;
    }
}


/* ========================================= */
/* ===== 5. TASKS AND MAIN ===== */
/* ========================================= */

static void wifi_ntp_task(void *pvParameters) {
    int retries = 0;
    static volatile bool wifi_initialized = false;
    printf("=== Wi-Fi + NTP Background Task ===\n");

    if (cyw43_arch_init_with_country(CYW43_COUNTRY_SINGAPORE)) {
        printf("[NTP] Wi-Fi init failed!\n");
        vTaskDelete(NULL);
    }
    
    // Implement robust Wi-Fi retry loop
    while (retries < WIFI_MAX_RETRIES) {
        printf("[NTP] Attempting Wi-Fi connection (Try %d/%d)...\n", retries + 1, WIFI_MAX_RETRIES);
        cyw43_arch_enable_sta_mode();
        printf("[NTP] Connecting to Wi-Fi SSID: %s\n", WIFI_SSID);

        if (cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 15000)) {
            printf("[NTP] Wi-Fi connection failed (Retrying...).\n");
            retries++;
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 seconds before retry
        } else {
            printf("[NTP] Connected to Wi-Fi!\n");
            wifi_initialized = true;
            break;
        }
    }

    if (!wifi_initialized) {
        printf("\n[NTP] ERROR: Failed to connect to Wi-Fi after %d attempts.\n", WIFI_MAX_RETRIES);
        printf("Please check network settings or restart to try again.\n");
        vTaskDelete(NULL);
    }

    // NTP Sync Loop
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

static void uart_activation_task(void *pvParameters) {
    comm_state_t state = STATE_HANDSHAKE;

    activation_driver_init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE);
    
    // Initialize GPIOs
    gpio_init(BTN_PIN); gpio_set_dir(BTN_PIN, GPIO_IN); gpio_pull_up(BTN_PIN);
    gpio_init(DUMP_BTN_PIN); gpio_set_dir(DUMP_BTN_PIN, GPIO_IN); gpio_pull_up(DUMP_BTN_PIN);
    gpio_init(WIPE_BTN_PIN); gpio_set_dir(WIPE_BTN_PIN, GPIO_IN); gpio_pull_up(WIPE_BTN_PIN);

    printf("=== Master Pico (UART + NTP integrated) ===\n");
    printf("GP%d: Request data. GP%d: Dump saved data. GP%d: WIPE saved data.\n", BTN_PIN, DUMP_BTN_PIN, WIPE_BTN_PIN);

    while (1) {
        
        // REFACTORED: Handle Dump and Wipe buttons immediately
        if (handle_user_buttons(&state)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        switch (state) {
            // --- HANDSHAKE PHASE ---
            case STATE_HANDSHAKE:
                uart_putc_raw(uart_get_instance(UART_PORT_NUM), CMD_HELLO);
                uint8_t response = 0;
                size_t read_count = uart_driver_read_timed(&response, 1, 500);

                if (read_count == 1 && response == CMD_HI_ACK) {
                    printf("[Master] Handshake OK\n");
                    state = STATE_IDLE;
                }                
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            // --- IDLE PHASE ---
            case STATE_IDLE:
                // Handle GET_DATA request (GP20 button)
                if (!gpio_get(BTN_PIN)) {
                    printf("[Master] Button (GP20) pressed → Requesting data\n");

                    while (uart_driver_available()){
                      uart_driver_read_byte();
                    }
          
                    uart_putc_raw(uart_get_instance(UART_PORT_NUM), CMD_REQ_DATA);
                    received_data_length = 0;
                    state = STATE_RECEIVING;
                }
                
                // REFACTORED: Handle incoming GET_TIME or other commands
                handle_time_request();
                
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            // --- RECEIVING PHASE ---
            case STATE_RECEIVING:
                // REFACTORED: Handled by dedicated function using binary protocol
                handle_receiving_state(&state);
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main() {
    stdio_init_all();
    
    ntp_mutex = xSemaphoreCreateMutex();

    xTaskCreate(wifi_ntp_task, "wifi_ntp_task", 4096, NULL, 2, NULL);
    xTaskCreate(uart_activation_task, "uart_activation_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1) tight_loop_contents();
}
