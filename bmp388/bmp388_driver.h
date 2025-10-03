#ifndef BMP388_DRIVER_H
#define BMP388_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float temperature_c;
} bmp388_sample_t;

int bmp388_init(int i2c_port, uint32_t sda_pin, uint32_t scl_pin, uint8_t i2c_addr);
int bmp388_read(bmp388_sample_t *out);

#endif
