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
#define CMD_DUMP_HDR      0x50
#define CMD_DUMP_TIME     0x51
#define BINARY_DUMP_HEADER_SIZE 4
#define BINARY_HEADER_SIZE 3 // 1 byte CMD + 2 byte Length
#define BINARY_TIME_SIZE   9 // 1 byte CMD + 8 byte Unix time
#define UNIX_TIME_PAYLOAD_SIZE 8 // 8 byte from slave pico time payload

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

// Time when the slave last requested activation time (GET_TIME)
static uint64_t last_activation_unix_time = 0;

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
static uint64_t last_dump_unix_time = 0;

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

    printf("[Master] SUCCESS: Data region wiped.\n");
}

static void master_dump_saved_data(void) {
    if (!usb_is_connected()) {
        printf("[Master] USB not connected. Aborting dump.\n");
        return;
    }

    const master_dump_hdr_t *h = xip_hdr_master_dump();
    if (h->magic != MASTER_DUMP_MAGIC) {
        printf("[Master] No valid data blob found in flash (Magic: 0x%X).\n", (unsigned)h->magic);
        usb_send("[DUMP] No valid data found in flash.\n");
        return;
    }

    const uint32_t length = h->length;
    if (length == 0 || length > MAX_COMPACT_DUMP_SIZE) {
        printf("[Master] Invalid data length: %u bytes.\n", (unsigned)length);
        usb_send("[DUMP] Invalid data length recorded.\n");
        return;
    }

    // Pointer to compact data stored in flash
    const uint8_t *data_ptr = xip_data_master_dump();

    printf("[Master] DUMP START: Dumping %u bytes of compact data over USB...\n",
           (unsigned)length);

    // Send a TIME line for the Python script
    time_t base_time = 0;

    // NEW: try to read Unix time from header->reserved[0..7]
    uint64_t header_unix_time = 0;
    memcpy(&header_unix_time, h->reserved, sizeof(uint64_t));


    if (header_unix_time != 0 && header_unix_time != UINT64_MAX) {
        // Preferred: time persisted in flash when dump was saved
        base_time = (time_t)header_unix_time;
        printf("[Master Dump] Using header activation time: %llu\n",
               (unsigned long long)header_unix_time);
    }
    else if (last_dump_unix_time != 0) {
        // Use the time that came from the slave (stored when data was received)
        base_time = (time_t)last_dump_unix_time;
    } else if (last_ntp_time != 0) {
        // Fallback: compute current NTP-based time
        uint64_t delta_ms =
            to_ms_since_boot(get_absolute_time()) - to_ms_since_boot(last_sync_time);
        base_time = last_ntp_time + (delta_ms / 1000);
    }

    if (base_time != 0) {
        struct tm *tm = gmtime(&base_time);
        char timebuf[64];
        snprintf(timebuf, sizeof(timebuf),
                 "TIME %04d-%02d-%02d %02d:%02d:%02d\n",
                 tm->tm_year + 1900,
                 tm->tm_mon + 1,
                 tm->tm_mday,
                 tm->tm_hour,
                 tm->tm_min,
                 tm->tm_sec);
        usb_send(timebuf);
    } else {
        // If we really have no time, still send an empty TIME for the script
        usb_send("TIME 1970-01-01 00:00:00\n");
    }

    // Make sure the TIME line is pushed out before the binary
    stdio_flush();
    sleep_ms(50);

    // Send ONLY the raw compact blob (no custom headers)
    usb_send_raw(data_ptr, length);

    // Eend marker
    usb_send("\n[DUMP_END]\n");

    printf("[Master] Dump successful.\n");
}

void master_save_compact_dump(const uint8_t *data, uint32_t length) {
    if (length == 0 || length > MAX_COMPACT_DUMP_SIZE) {
        printf("[Master] ERROR: Data length (%u) invalid or too large.\n", (unsigned)length);
        return;
    }

    flash_erase_sectors(MASTER_DUMP_FLASH_OFFSET, MASTER_DUMP_REGION_SIZE);

    master_dump_hdr_t hdr;
    memset(&hdr, 0xFF, sizeof(hdr));
    hdr.magic = MASTER_DUMP_MAGIC;
    hdr.length = length;

    // Persist the activation time (if known) into header->reserved[0..7]
    if (last_activation_unix_time != 0) {
        memcpy(hdr.reserved, &last_activation_unix_time, sizeof(uint64_t));
        printf("[Master] Stored activation time %llu into dump header.\n",
               (unsigned long long)last_activation_unix_time);
    }

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
    printf("[Master] SUCCESS: Saved %u bytes to flash offset 0x%X.\n", (unsigned)length, (unsigned)MASTER_DUMP_FLASH_OFFSET);
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

        // --- Save activation time when slave requested GET_TIME ---
        if (current_time > 0) {
            last_activation_unix_time = (uint64_t)current_time;
            printf("[Master] Saved activation time: %llu\n",
                (unsigned long long)last_activation_unix_time);
        }

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

        if (expected_length < UNIX_TIME_PAYLOAD_SIZE) { // Check if payload is large enough for the time header
            printf("[Master] ERROR: Expected length %u too small for 8-byte time header. Aborting.\n", (unsigned)expected_length);
            *state_ptr = STATE_IDLE;
            return;
        }
        if (expected_length > MAX_COMPACT_DUMP_SIZE) {
            printf("[Master] ERROR: Expected length %u exceeds MAX_DUMP_SIZE. Aborting.\n", (unsigned)expected_length);
            *state_ptr = STATE_IDLE;
            return;
        }

        // Read Binary Data Payload (Content includes 8B time + compact data)
        received_data_length = uart_driver_read_timed(received_data_buffer, expected_length, TIMEOUT_MS);

        if (received_data_length == expected_length) {
            printf("[Master] SUCCESS: Received %u / %u bytes. Waiting for DONE signal.\n",
                   (unsigned)received_data_length, (unsigned)expected_length);

            // Read DONE_CHAR (Tail)
            uint8_t done_signal = 0;
            size_t done_read = uart_driver_read_timed(&done_signal, 1, TIMEOUT_MS); 

            if (done_read == 1 && done_signal == DONE_CHAR) {
                printf("[Master] Data transfer complete and acknowledged by Slave (ACK: 0x%02X).\n", DONE_CHAR);
                
                // --- TIME EXTRACTION AND SLICING LOGIC ---
                
                // 1. Extract 8-byte Unix time (Little-Endian assumed)
                uint64_t unix_time_val = 0;
                // Copy 8 bytes from the start of the buffer
                memcpy(&unix_time_val, received_data_buffer, UNIX_TIME_PAYLOAD_SIZE);
                
                // 2. Calculate the length and pointer for the Compact Data Blob
                const uint8_t *compact_data_ptr = &received_data_buffer[UNIX_TIME_PAYLOAD_SIZE];
                uint32_t compact_data_length = received_data_length - UNIX_TIME_PAYLOAD_SIZE;
                
                // 3. Log the extracted time (Human readable)
                time_t current_time_sec = (time_t)unix_time_val;
                printf("[Master] Extracted start time from Slave data: %s", ctime(&current_time_sec)); // Logs the time for Python script to capture
                last_dump_unix_time = unix_time_val;

                // 4. Compute HMAC on the received Compact Data Blob ONLY
                uint8_t computed_mac[HMAC_LEN];
                hmac_sha256((const uint8_t *)HMAC_KEY, strlen(HMAC_KEY), compact_data_ptr, compact_data_length, computed_mac);
                
                // Log the computed hash
                print_hex(computed_mac, HMAC_LEN, "[Master] Calculated HMAC: ");
                
                // 5. Save the Compact Data Blob (excluding time) to internal flash
                master_save_compact_dump(compact_data_ptr, compact_data_length);
                
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

    } else {
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
        
        // Handle Dump and Wipe buttons immediately
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
                
                // Handle incoming GET_TIME or other commands
                handle_time_request();
                
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            // --- RECEIVING PHASE ---
            case STATE_RECEIVING:
                // Handled by dedicated function using binary protocol
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
