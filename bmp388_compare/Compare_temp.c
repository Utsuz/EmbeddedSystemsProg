#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"       // <-- add
#include "bmp388_driver.h"

#define EX_I2C_PORT   0
#define EX_SDA_PIN    4
#define EX_SCL_PIN    5
#define EX_ADDR       0x77      // your probe showed 0x77

// Change this to adjust read rate (milliseconds)
#ifndef SAMPLE_PERIOD_MS
#define SAMPLE_PERIOD_MS 5000   // 5 seconds
#endif

// ---------- RP2040 on-chip temperature helpers ----------
/*
 * RP2040 datasheet typical calibration:
 *   T(°C) = 27 - (Vsen - 0.706) / 0.001721
 * ADC: 12-bit (0..4095), Vref ≈ 3.3 V on Pico boards.
 * Note: on-chip reading runs hot (CPU/self-heating). Expect +2..+8 °C vs ambient.
 */
static float read_onchip_temp_c(void) {
    // Ensure ADC is ready and temp sensor enabled once
    static bool adc_ready = false;
    if (!adc_ready) {
        adc_init();
        adc_set_temp_sensor_enabled(true);  // routes internal diode to ADC
        adc_ready = true;
    }

    adc_select_input(4); // Temp sensor is ADC channel 4
    // Simple average of a few samples to reduce noise
    const int N = 8;
    uint32_t acc = 0;
    for (int i = 0; i < N; ++i) {
        acc += adc_read();  // 12-bit result
        sleep_us(50);
    }
    float raw = (float)acc / (float)N;

    // Convert raw to voltage
    const float ADC_VREF = 3.3f;           // Pico board uses 3.3 V as ADC reference
    const float ADC_MAX  = 4095.0f;        // 12-bit
    float v_sense = raw * (ADC_VREF / ADC_MAX);

    // Convert to °C using typical formula
    float temp_c = 27.0f - (v_sense - 0.706f) / 0.001721f;
    return temp_c;
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    printf("USB up. BMP388 + RP2040 on-chip temperature compare\n");

    int rc = bmp388_init(EX_I2C_PORT, EX_SDA_PIN, EX_SCL_PIN, EX_ADDR);
    if (rc != 0) {
        printf("Init failed: %d\n", rc);
        while (1) sleep_ms(1000);
    }

    while (true) {
        bmp388_sample_t s;
        int r = bmp388_read(&s);

        float chip_c = read_onchip_temp_c();

        if (r == 0) {
            float delta = chip_c - s.temperature_c;
            printf("BMP388=%.2f °C | On-chip=%.2f °C | Δ=%+.2f °C\n",
                   s.temperature_c, chip_c, delta);
        } else if (r == -3) {
            // Not ready yet — try once more quickly
            sleep_ms(50);
            if (bmp388_read(&s) == 0) {
                float delta = chip_c - s.temperature_c;
                printf("BMP388=%.2f °C | On-chip=%.2f °C | Δ=%+.2f °C\n",
                       s.temperature_c, chip_c, delta);
            } else {
                printf("Read error: -3\n");
            }
        } else {
            printf("Read error: %d\n", r);
        }

        sleep_ms(SAMPLE_PERIOD_MS);
    }
}
