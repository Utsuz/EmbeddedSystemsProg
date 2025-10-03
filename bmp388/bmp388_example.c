#include <stdio.h>
#include "pico/stdlib.h"
#include "bmp388_driver.h"

#define EX_I2C_PORT   0
#define EX_SDA_PIN    4   // GP4 (Maker Pi Pico Grove SDA)
#define EX_SCL_PIN    5   // GP5 (Maker Pi Pico Grove SCL)
#define EX_ADDR       0x77

int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    printf("USB up. Starting BMP388 minimal example...\n");

    int rc = bmp388_init(EX_I2C_PORT, EX_SDA_PIN, EX_SCL_PIN, EX_ADDR);
    if (rc != 0) {
        printf("Init failed: %d\n", rc);
        while (1) sleep_ms(2000);
    }

    while (true) {
        bmp388_sample_t s;
        int r = bmp388_read(&s);
        if (r == 0) {
            printf("T=%.2f °C",
                   s.temperature_c);
        } else {
            printf("Read error: %d\n", r);
        }
        sleep_ms(2000);
    }
}
