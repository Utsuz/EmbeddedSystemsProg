#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "bmp388_driver.h"

#define EX_I2C_PORT   0
#define EX_SDA_PIN    4
#define EX_SCL_PIN    5
#define EX_ADDR       0x77

// === Buttons ===
#define BTN_ROTATE 20   // GP20 → rotate ACTIVE→BACKUP
#define BTN_DUMP   21   // GP21 → dump BACKUP to serial

#ifndef SAMPLE_PERIOD_MS
#define SAMPLE_PERIOD_MS 5000
#endif
#ifndef LOG_ERASE_ON_BOOT
#define LOG_ERASE_ON_BOOT 0
#endif

static void dump_active(void) {
    uint32_t n = bmp388_storage_count();
    printf("\n# ACTIVE index,time_ms,temp_C\n");
    for (uint32_t i=0;i<n;i++) {
        uint32_t t; float temp;
        if (bmp388_storage_read(i, &t, &temp) == 0)
            printf("%lu,%lu,%.2f\n", (unsigned long)i, (unsigned long)t, temp);
    }
    printf("# ACTIVE total %lu\n\n", (unsigned long)n);
}
static void dump_backup(void) {
    uint32_t n = bmp388_backup_count();
    printf("\n# BACKUP index,time_ms,temp_C\n");
    for (uint32_t i=0;i<n;i++) {
        uint32_t t; float temp;
        if (bmp388_backup_read(i, &t, &temp) == 0)
            printf("%lu,%lu,%.2f\n", (unsigned long)i, (unsigned long)t, temp);
    }
    printf("# BACKUP total %lu\n\n", (unsigned long)n);
}

int main(void) {
    stdio_init_all();
    sleep_ms(1200);
    printf("\n=== BMP388 Logger (Dual-Region) ===\n");
    bmp388_storage_init(LOG_ERASE_ON_BOOT);

    int rc = bmp388_init(EX_I2C_PORT, EX_SDA_PIN, EX_SCL_PIN, EX_ADDR);
    if (rc) { printf("Init failed: %d\n", rc); while (1) sleep_ms(1000); }

    gpio_init(BTN_ROTATE);
    gpio_pull_up(BTN_ROTATE);
    gpio_init(BTN_DUMP);
    gpio_pull_up(BTN_DUMP);

    bool prev_rotate = true;
    bool prev_dump   = true;

    absolute_time_t next = make_timeout_time_ms(SAMPLE_PERIOD_MS);

    printf("Commands: D=dump ACTIVE, B=dump BACKUP, R=rotate now, X=clear backup, C=count\n");
    

    while (true) {
        int c = getchar_timeout_us(0);
        if (c == 'D' || c == 'd') dump_active();
        else if (c == 'B' || c == 'b') dump_backup();
        else if (c == 'R' || c == 'r') {
            printf("Rotating ACTIVE -> BACKUP...\n");
            bmp388_storage_rotate_to_backup();
            printf("Done. ACTIVE cleared. BACKUP count=%u\n", bmp388_backup_count());
        }
        else if (c == 'X' || c == 'x') {
            printf("Clearing BACKUP...\n");
            bmp388_backup_clear();
            printf("Done.\n");
        }
        else if (c == 'C' || c == 'c') {
            printf("ACTIVE=%u, BACKUP=%u\n",
                (unsigned)bmp388_storage_count(), (unsigned)bmp388_backup_count());
        }
        bool button_now = gpio_get(BTN_ROTATE);
        if (!button_now && prev_rotate) {   // just pressed (went LOW)
            printf("Button pressed -> rotating ACTIVE → BACKUP...\n");
            bmp388_storage_rotate_to_backup();
            printf("Rotate complete. ACTIVE=%u BACKUP=%u\n",
                   (unsigned)bmp388_storage_count(),
                   (unsigned)bmp388_backup_count());
        }
        prev_rotate = button_now;
        // ---- Check Button B (Dump BACKUP) ----
        bool now_dump = gpio_get(BTN_DUMP);
        if (!now_dump && prev_dump) {  // pressed (LOW edge)
            printf("[BTN21] Dumping BACKUP log...\n");
            dump_backup();
        }
        prev_dump = now_dump;
        if (absolute_time_diff_us(get_absolute_time(), next) <= 0) {
            bmp388_sample_t s;
            if (bmp388_read(&s) == 0) {
                uint32_t now_ms = to_ms_since_boot(get_absolute_time());
                printf("T=%.2f C (t=%u)\n", s.temperature_c, (unsigned)now_ms);
                int lr = bmp388_storage_append(now_ms, s.temperature_c);
                if (lr) printf("Note: log rotated (full). ACTIVE restarted.\n");
            }
            next = delayed_by_ms(next, SAMPLE_PERIOD_MS);
        }
    }
}
