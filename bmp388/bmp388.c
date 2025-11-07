#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "bmp388_driver.h"

#define EX_I2C_PORT   0
#define EX_SDA_PIN    4
#define EX_SCL_PIN    5
#define EX_ADDR       0x77

// === Buttons ===
// GP20: BACKUP (copy ACTIVE -> BACKUP, clear ACTIVE)
// GP21: DUMP BACKUP (print what's saved)
// GP22: Binary dump (ACTIVE)
#define BTN_BACKUP 20
#define BTN_SHOW   21
#define BTN_TOGGLE 22   // GP22 → start/stop sensor

#ifndef SAMPLE_PERIOD_MS
#define SAMPLE_PERIOD_MS 5000
#endif
#ifndef LOG_ERASE_ON_BOOT
#define LOG_ERASE_ON_BOOT 0
#endif

// static void print_counts(void) {
//     uint32_t a = bmp388_storage_count();
//     uint32_t b = bmp388_backup_count();
//     printf("# ACTIVE=%lu, BACKUP=%lu\n", (unsigned long)a, (unsigned long)b);
// }

int main(void) {
    stdio_init_all();
    sleep_ms(1200);
    printf("\n=== BMP388 Logger (Dual-Region) ===\n");
    bmp388_storage_init(LOG_ERASE_ON_BOOT);

    int rc = bmp388_init(EX_I2C_PORT, EX_SDA_PIN, EX_SCL_PIN, EX_ADDR);
    if (rc) { printf("Init failed: %d\n", rc); while (1) sleep_ms(1000); }

    gpio_init(BTN_BACKUP); gpio_pull_up(BTN_BACKUP);
    gpio_init(BTN_SHOW);   gpio_pull_up(BTN_SHOW);
    gpio_init(BTN_TOGGLE); gpio_pull_up(BTN_TOGGLE);

    // bool prev_backup = true;
    bool prev_show   = true;
    bool prev_toggle = true;
    bool sensor_active = true;


    absolute_time_t next = make_timeout_time_ms(SAMPLE_PERIOD_MS);

    printf("Buttons: GP20=BACKUP, GP21=SHOW BACKUP, GP22=BINARY ACTIVE\n");
    printf("Serial:  D=Dump ACTIVE (compact text), B=Dump BACKUP, R=Backup now, X=Clear BACKUP, C=Counts\n");

    while (true) {
        /* --- serial commands (optional) --- */
        int c = getchar_timeout_us(0);
        // if (c == 'D' || c == 'd') dump_active_compact();
        // if (c == 'B' || c == 'b') dump_backup();
        // if (c == 'R' || c == 'r') { bmp388_storage_rotate_to_backup(); printf("Rotated ACTIVE -> BACKUP.\n"); }
        // if (c == 'X' || c == 'x') { bmp388_backup_clear(); printf("BACKUP cleared.\n"); }
        // if (c == 'S' || c == 's') bmp388_backup_compact_save();
        // if (c == 'B' || c == 'b') dump_backup_compact_raw();
        if (c == 'A' || c == 'a') dump_active_compact_bit();

        /* --- periodic sample + append to ACTIVE --- */
        if (absolute_time_diff_us(get_absolute_time(), next) <= 0) {
            bmp388_sample_t s;
            if (bmp388_read(&s) == 0) {
                uint32_t now_ms = to_ms_since_boot(get_absolute_time());
                printf("T=%.2f C (t=%u)\n", s.temperature_c, (unsigned)now_ms);
                int lr = bmp388_storage_append(now_ms, s.temperature_c);
                if (lr) printf("Note: ACTIVE was full -> rotated to BACKUP; ACTIVE restarted.\n");
            }
            next = delayed_by_ms(next, SAMPLE_PERIOD_MS);
        }

        // /* --- Button: GP20 → BACKUP now --- (active low) */
        // bool now_backup = gpio_get(BTN_BACKUP);
        // if (!now_backup && prev_backup) {
        //     bmp388_storage_rotate_to_backup();
        //     if (sensor_active) {
        //         bmp388_sensorStop();
        //         printf("[BTN22] Sensor stopped.\n");
        //         sensor_active = false;
        //     } else {
        //         bmp388_sensorStart();
        //         printf("[BTN22] Sensor started.\n");
        //         sensor_active = true;
        //     }
        //     printf("[BTN] Rotated ACTIVE -> BACKUP.\n");
        // }
        // prev_backup = now_backup;

        /* --- Button: GP21 → SHOW BACKUP --- (active low) */
        bool now_show = gpio_get(BTN_SHOW);
        if (!now_show && prev_show) {
            stdio_flush();
            dump_active_compact_bit();
            stdio_flush();
        }
        prev_show = now_show;

        bool now_toggle = gpio_get(BTN_TOGGLE);
        if (!now_toggle && prev_toggle) {  // pressed (LOW edge)
            if (sensor_active) {
                bmp388_sensorStop();
                printf("[BTN22] Sensor stopped.\n");
                sensor_active = false;
            } else {
                bmp388_sensorStart();
                printf("[BTN22] Sensor started.\n");
                sensor_active = true;
            }
        }
        prev_toggle = now_toggle;
    }
}
