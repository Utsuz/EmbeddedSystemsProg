#ifndef BMP388_DRIVER_H
#define BMP388_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float temperature_c;
} bmp388_sample_t;

/* Initialize BMP388 on the given I2C port/pins/address.
 * i2c_port: 0 or 1
 * sda_pin/scl_pin: Pico GPIO numbers
 * i2c_addr: 0x76 or 0x77
 * Returns 0 on success, <0 on error.
 */
int bmp388_init(int i2c_port, uint32_t sda_pin, uint32_t scl_pin, uint8_t i2c_addr);

/* Read latest sample (temperature only for now).
 * Returns 0 on success, <0 on error.
 */
int bmp388_read(bmp388_sample_t *out);

#endif
