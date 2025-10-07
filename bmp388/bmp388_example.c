#include <stdio.h>
#include "pico/stdlib.h"
#include "bmp388_driver.h"

#define EX_I2C_PORT   0
#define EX_SDA_PIN    4
#define EX_SCL_PIN    5
#define EX_ADDR       0x77

// ← Change this to adjust read rate (milliseconds)
#ifndef SAMPLE_PERIOD_MS
#define SAMPLE_PERIOD_MS 5000   // 5 seconds
#endif

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    printf("USB up. Starting BMP388 temperature example...\n");

    int rc = bmp388_init(EX_I2C_PORT, EX_SDA_PIN, EX_SCL_PIN, EX_ADDR);
    if (rc != 0) {
        printf("Init failed: %d\n", rc);
        while (1) sleep_ms(1000);
    }

    while (true) {
        bmp388_sample_t s;
        int r = bmp388_read(&s);
        if (r == 0) {
            printf("T=%.2f °C\n", s.temperature_c);
        } else if (r == -3) {
            // Not ready yet — harmless if we woke up early. Try once more after a short wait.
            sleep_ms(50);
            if (bmp388_read(&s) == 0) printf("T=%.2f °C\n", s.temperature_c);
            else                       printf("Read error: -3\n");
        } else {
            printf("Read error: %d\n", r);
        }

        sleep_ms(SAMPLE_PERIOD_MS);
    }
}
