#include "pico/stdlib.h"
#include "activation_driver.h"
#include "uart_driver.h"
#include "bmp388_driver.h"
#include "pico/time.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>

/* === UART handshake (same as your original) === */
#define UART_PORT_NUM   1
#define UART_TX_PIN     8
#define UART_RX_PIN     9
#define UART_BAUDRATE   115200

/* === I2C & BMP388 === */
#define EX_I2C_PORT   0
#define EX_SDA_PIN    4
#define EX_SCL_PIN    5
#define EX_ADDR       0x77

/* === Buttons ===
 * GP20: ask master for time (your original flow)
 * GP21: dump ACTIVE in compact-binary (to USB stdio)
 * GP22: start/stop sensor
 */
#define BTN_TIME    20
#define BTN_DUMP    21
#define BTN_TOGGLE  22

#ifndef SAMPLE_PERIOD_MS
#define SAMPLE_PERIOD_MS 5000
#endif
#ifndef EXCURSION_PERIOD_MS
#define EXCURSION_PERIOD_MS 200   /* fast sampling while in excursion */
#endif
#ifndef LOG_ERASE_ON_BOOT
#define LOG_ERASE_ON_BOOT 0
#endif

static inline uint32_t current_period_ms(void) {
    return bmp388_excursion_state() ? EXCURSION_PERIOD_MS : SAMPLE_PERIOD_MS;
}

typedef enum {
    STATE_HANDSHAKE,
    STATE_IDLE,
    STATE_WAIT_TIME
} comm_state_t;

// --- Helper Function to Dump Compact Data to Master UART (UART1) ---
static uint32_t dump_compact_to_uart(void) {
    const compact_hdr_t *h = xip_hdr_compact();
    
    // Check for valid blob (CB_MAGIC check and non-zero length)
    if (h->magic != CB_MAGIC || h->length == 0) { 
        printf("[COMPACT] No valid blob found (len: %u).\n", (unsigned)h->length);
        activation_send("ACK_EMPTY\n"); // Signal to Master that log is empty
        return 0;
    }

    uint32_t length = h->length;
    
    // 1. Tell the Master the length first (Text-based signal)
    char msg[64];
    snprintf(msg, sizeof(msg), "LENGTH %u\n", (unsigned)length);
    activation_send(msg);

    sleep_ms(250);  

    printf("[Slave] Sending %u bytes of compact data...\n", (unsigned)length);

    // 2. Send the raw binary data
    const uint8_t *p = xip_data_compact();
    // Use the raw UART put function for binary transfer
    for (uint32_t i = 0; i < length; i++) {
        uart_putc_raw(uart_get_instance(UART_PORT_NUM), p[i]);
    }
    
    // 3. Send the DONE signal (Text-based footer)
    activation_send("DONE\n");
    
    return length;
}

int main(void) {
    stdio_init_all();
    sleep_ms(1200);

    printf("=== Slave Pico (BMP388 logger) ===\n");

    /* --- Init UART (activation) --- */
    activation_driver_init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE);

    /* --- Init BMP388 + storage --- */
    bmp388_storage_init();    
    int rc = bmp388_init(EX_I2C_PORT, EX_SDA_PIN, EX_SCL_PIN, EX_ADDR);
    if (rc) {
        printf("BMP388 init failed: %d\n", rc);
        while (1) sleep_ms(1000);
    }

    /* --- Buttons --- */
    gpio_init(BTN_TIME);   gpio_set_dir(BTN_TIME,   GPIO_IN); gpio_pull_up(BTN_TIME);
    gpio_init(BTN_DUMP);   gpio_set_dir(BTN_DUMP,   GPIO_IN); gpio_pull_up(BTN_DUMP);
    gpio_init(BTN_TOGGLE); gpio_set_dir(BTN_TOGGLE, GPIO_IN); gpio_pull_up(BTN_TOGGLE);

    bool prev_dump   = true;
    bool prev_toggle = true;
    bool sensor_active = false;          

    absolute_time_t next = make_timeout_time_ms(SAMPLE_PERIOD_MS);

    /* --- State machine for handshake/time --- */
    char recv_buf[128];
    comm_state_t state = STATE_HANDSHAKE;

    printf("Buttons: GP20=request TIME, GP21=dump compact, GP22=start/stop\n");
    printf("USB serial: press 'A' to dump compact\n");

    while (true) {
        /* ========== background: periodic sampling ========== */
        if (sensor_active && absolute_time_diff_us(get_absolute_time(), next) <= 0) {
            bmp388_sample_t s;
            if (bmp388_read(&s) == 0) {
                uint32_t now_ms = to_ms_since_boot(get_absolute_time());
                int lr = bmp388_storage_append(now_ms, s.temperature_c);
                printf("T=%.2f C (t=%u)%s\n",
                       s.temperature_c, (unsigned)now_ms,
                       bmp388_excursion_state() ? " [FAST]" : "");
                if (lr) {
                    printf("Note: ACTIVE region is full; sample not saved.\n");
                }
            }
            /* Re-schedule from NOW so an excursion flip changes cadence immediately */
            next = delayed_by_ms(get_absolute_time(), current_period_ms());
        }

        /* ========== background: quick USB command for dump ========== */
        int c = getchar_timeout_us(0);
        if (c == 'A' || c == 'a') {
            stdio_flush();
            dump_active_compact_bit();   // emits to USB stdio
            stdio_flush();
        }

        /* ========== buttons ========== */
        // GP21: dump compact
        bool now_dump = gpio_get(BTN_DUMP);
        if (!now_dump && prev_dump) {
            stdio_flush();
            dump_active_compact_bit();   // emits to USB stdio
            stdio_flush();
        }
        prev_dump = now_dump;

        // GP22: toggle sensor
        bool now_toggle = gpio_get(BTN_TOGGLE);
        if (!now_toggle && prev_toggle) {
            if (sensor_active) {
                bmp388_sensorStop();
                sensor_active = false;
                printf("[BTN22] Sensor stopped.\n");
            } else {
                bmp388_sensorStart();
                sensor_active = true;
                printf("[BTN22] Sensor started.\n");
            }
        }
        prev_toggle = now_toggle;

        /* ========== original handshake/time state machine ========== */
        switch (state) {
            case STATE_HANDSHAKE:
                if (activation_receive_line(recv_buf, sizeof(recv_buf))) {
                    if (strncmp(recv_buf, "HELLO", 5) == 0) {
                        activation_send("HI\n");
                        printf("[Slave] Handshake OK\n");
                        state = STATE_IDLE;
                    }
                }
                break;

            case STATE_IDLE:
                // GP20: ask master for time 
                if (!gpio_get(BTN_TIME)) {
                    printf("[Slave] Button pressed → Requesting time\n");
                    activation_send("GET_TIME\n");
                    state = STATE_WAIT_TIME;
                }
                // Handle GET_DATA command from Master 
                if (activation_receive_line(recv_buf, sizeof(recv_buf))) {
                    if (strncmp(recv_buf, "GET_DATA", 8) == 0) {
                        printf("[Slave] Received GET_DATA. Stopping sensor, creating backup, and transferring...\n");
                        
                        // 1. Stop Logging/Sampling (per requirement)
                        bmp388_sensorStop();
                        sensor_active = false;
                        printf("[Slave] Sensor stopped.\n");

                        // 2. Create Compact Backup on Flash (per requirement)
                        bmp388_backup_compact_save();
                        printf("[Slave] Compact backup created on flash.\n");
                        
                        // 3. Dump Raw Compact Data over UART to Master
                        uint32_t len = dump_compact_to_uart();
                        
                        if (len > 0) {
                            printf("[Slave] Sent %u bytes of compact data over UART1.\n", (unsigned)len);
                        } else {
                            printf("[Slave] Log empty, sent ACK_EMPTY.\n");
                        }
                        
                    } else if (strncmp(recv_buf, "HELLO", 5) == 0) {
                        activation_send("HI\n");
                    }
                }
                break;

            case STATE_WAIT_TIME: {
                absolute_time_t deadline = make_timeout_time_ms(5000);
                bool got_time = false;
                while (!got_time && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
                    if (activation_receive_line(recv_buf, sizeof(recv_buf))) {
                        uint64_t base_time = 0;
                        if (sscanf(recv_buf, "TIME %" SCNu64, &base_time) == 1) {
                            printf("[Slave] Received Unix time: %" PRIu64 "\n", base_time);
                            time_t t = (time_t)base_time;
                            printf("[Slave] Human time: %s", ctime(&t));

                            // Start/continue sampling; logger uses ms_since_boot internally
                            bmp388_sensorStart();
                            sensor_active = true;
                            printf("[Slave] Sensor started (time synchronized).\n");
                            next = delayed_by_ms(get_absolute_time(), current_period_ms());
                            got_time = true;
                            state = STATE_IDLE;
                        } else {
                            printf("[Slave] Failed to parse TIME: %s\n", recv_buf);
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

        sleep_ms(20);
    }
}
