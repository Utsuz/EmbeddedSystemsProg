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
#ifndef LOG_ERASE_ON_BOOT
#define LOG_ERASE_ON_BOOT 0
#endif

typedef enum {
    STATE_HANDSHAKE,
    STATE_IDLE,
    STATE_WAIT_TIME
} comm_state_t;

int main(void) {
    stdio_init_all();
    sleep_ms(1200);

    printf("=== Slave Pico (BMP388 logger) ===\n");

    /* --- Init UART (activation) --- */
    activation_driver_init(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE);

    /* --- Init BMP388 + storage --- */
    bmp388_storage_init(LOG_ERASE_ON_BOOT);
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
    bool sensor_active = true;           // start sampling by default
    absolute_time_t next_sample = make_timeout_time_ms(SAMPLE_PERIOD_MS);

    /* --- State machine for handshake/time --- */
    char recv_buf[128];
    comm_state_t state = STATE_HANDSHAKE;

    printf("Buttons: GP20=request TIME, GP21=dump compact, GP22=start/stop\n");
    printf("USB serial: press 'A' to dump compact\n");

    while (true) {
        /* ========== background: periodic sampling ========== */
        if (sensor_active && absolute_time_diff_us(get_absolute_time(), next_sample) <= 0) {
            bmp388_sample_t s;
            if (bmp388_read(&s) == 0) {
                uint32_t now_ms = to_ms_since_boot(get_absolute_time());
                bmp388_storage_append(now_ms, s.temperature_c);
                printf("T=%.2f C (t=%u)\n", s.temperature_c, (unsigned)now_ms);
            }
            next_sample = delayed_by_ms(next_sample, SAMPLE_PERIOD_MS);
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
                // GP20: ask master for time (unchanged)
                if (!gpio_get(BTN_TIME)) {
                    printf("[Slave] Button pressed → Requesting time\n");
                    activation_send("GET_TIME\n");
                    state = STATE_WAIT_TIME;
                }
                // Optional: respond to simple text GET_DATA with a friendly ack
                if (activation_receive_line(recv_buf, sizeof(recv_buf))) {
                    if (strncmp(recv_buf, "GET_DATA", 8) == 0) {
                        // NOTE: binary dump currently goes to USB stdio.
                        // If you want the binary over UART1 to master, we’ll add a UART dumper.
                        activation_send("ACK_LOGGING\n");
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

                            // Start/continue sampling; logger uses ms_since_boot internally.
                            sensor_active = true;
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
