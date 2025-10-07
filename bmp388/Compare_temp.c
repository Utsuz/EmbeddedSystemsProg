#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "bmp388_driver.h"

#define EX_I2C_PORT   0
#define EX_SDA_PIN    4
#define EX_SCL_PIN    5
#define EX_ADDR       0x77  // your probe showed 0x77

// Change this to adjust read rate (milliseconds)
#ifndef SAMPLE_PERIOD_MS
#define SAMPLE_PERIOD_MS 5000   // 5 seconds
#endif

// Optional: tweak this if you’ve calibrated your Pico’s internal sensor
#ifndef PICO_TEMP_OFFSET_C
#define PICO_TEMP_OFFSET_C 0.0f
#endif

// --- Pico internal temperature (ADC4) ---
// Formula from RP2040 datasheet:
//   T(°C) = 27 - (V_sense - 0.706) / 0.001721
// where V_sense = ADC_read * Vref / 4096.0  (12-bit ADC)
static float pico_internal_temp_c(void) {
    const float VREF = 3.3f;
    // take a few samples and median-filter to reduce noise
    uint16_t s[5];
    for (int i = 0; i < 5; i++) {
        s[i] = adc_read();
        sleep_us(1000);
    }
    // simple insertion sort for 5 elements
    for (int i = 1; i < 5; i++) {
        uint16_t key = s[i], j = i - 1;
        while (j >= 0 && s[j] > key) { s[j+1] = s[j]; j--; }
        s[j+1] = key;
    }
    uint16_t raw = s[2]; // median
    float v_sense = (raw * VREF) / 4096.0f;
    float t_c = 27.0f - (v_sense - 0.706f) / 0.001721f;
    return t_c + PICO_TEMP_OFFSET_C;
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    printf("USB up. Starting BMP388 + Pico temp compare...\n");

    // --- I2C sensor init ---
    int rc = bmp388_init(EX_I2C_PORT, EX_SDA_PIN, EX_SCL_PIN, EX_ADDR);
    if (rc != 0) {
        printf("BMP388 init failed: %d\n", rc);
        while (1) sleep_ms(1000);
    }

    // --- Pico internal temperature sensor init ---
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4); // ADC4 is internal temperature sensor

    // ---- ONE IMMEDIATE BLOCKING READ (so you see output right away) ----
    {
        const uint32_t first_timeout_ms = 1500;   // wait up to 1.5s for first sample
        uint32_t waited = 0;
        bmp388_sample_t s;
        int r;
        while ((r = bmp388_read(&s)) == -3 && waited < first_timeout_ms) {
            sleep_ms(10);
            waited += 10;
        }
        float pico_c = pico_internal_temp_c();
        if (r == 0) {
            float diff = s.temperature_c - pico_c;
            printf("[first] BMP388: %.2f °C | Pico: %.2f °C | Δ=%.2f °C\n",
                   s.temperature_c, pico_c, diff);
        } else {
            printf("[first] Read error: %d | Pico: %.2f °C\n", r, pico_c);
        }
    }

    // ---- REGULAR LOOP (every SAMPLE_PERIOD_MS) ----
    while (true) {
        bmp388_sample_t s;
        int r = bmp388_read(&s);
        float pico_c = pico_internal_temp_c();

        if (r == 0) {
            float diff = s.temperature_c - pico_c;
            printf("BMP388: %.2f °C | Pico: %.2f °C | Δ=%.2f °C\n",
                   s.temperature_c, pico_c, diff);
        } else if (r == -3) {
            // Not ready yet — harmless. Try once more after a short wait.
            sleep_ms(50);
            if (bmp388_read(&s) == 0) {
                float diff = s.temperature_c - pico_c;
                printf("BMP388: %.2f °C | Pico: %.2f °C | Δ=%.2f °C\n",
                       s.temperature_c, pico_c, diff);
            } else {
                printf("Read error: -3\n");
            }
        } else {
            printf("Read error: %d | Pico: %.2f °C\n", r, pico_c);
        }

        sleep_ms(SAMPLE_PERIOD_MS); // default 5000ms; change with -DSAMPLE_PERIOD_MS=xxx
    }
}
