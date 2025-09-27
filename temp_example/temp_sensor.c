#include "temp_sensor.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"

static inline float _adc_raw_to_celsius(uint16_t raw) {
    const float vref = 3.3f;                    // ADC reference voltage
    const float conversion = vref / (1 << 12);  // 12-bit ADC
    float v = raw * conversion;                 // volts at sensor output
    // RP2040 typical: 27°C at 0.706 V, slope −1.721 mV/°C
    return 27.0f - (v - 0.706f) / 0.001721f;
}

int temp_sensor_init(void) {
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4); // ADC4 = internal temperature sensor
    return 0;
}

int temp_sensor_read_celsius(int samples, float* temp_c_out) {
    if (!temp_c_out || samples <= 0) return -1;

    uint32_t acc = 0;
    for (int i = 0; i < samples; ++i) {
        acc += adc_read();
        tight_loop_contents(); // small hint for power/idle
    }
    uint16_t raw = (uint16_t)(acc / (uint32_t)samples);
    *temp_c_out = _adc_raw_to_celsius(raw);
    return 0;
}

void temp_sensor_shutdown(void) {
    adc_set_temp_sensor_enabled(false);
}
