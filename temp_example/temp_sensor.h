#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialise ADC and enable internal temperature sensor (ADC4).
// Returns 0 on success, nonzero on error.
int temp_sensor_init(void);

// Read temperature in Celsius. `samples` is a small average count (e.g., 8).
// Returns 0 on success; temp_c_out filled with result.
int temp_sensor_read_celsius(int samples, float* temp_c_out);

// Optionally disable the temp sensor.
void temp_sensor_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif // TEMP_SENSOR_H
