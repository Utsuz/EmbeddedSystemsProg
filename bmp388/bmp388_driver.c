#include "bmp388_driver.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>

static i2c_inst_t *g_i2c = i2c0;
static uint8_t g_addr = 0x77;
static bool g_init = false;

static int i2c_reg_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    int ret = i2c_write_timeout_us(g_i2c, g_addr, buf, 2, false, 2000);
    return (ret < 0) ? ret : 0;
}

static int i2c_reg_read(uint8_t reg, uint8_t *buf, size_t len) {
    int w = i2c_write_timeout_us(g_i2c, g_addr, &reg, 1, true, 2000);
    if (w < 0) return w;
    int r = i2c_read_timeout_us(g_i2c, g_addr, buf, len, false, 2000);
    return (r < 0) ? r : 0;
}

int bmp388_init(int i2c_port, uint32_t sda_pin, uint32_t scl_pin, uint8_t i2c_addr) {
    g_i2c = (i2c_port == 1) ? i2c1 : i2c0;
    g_addr = i2c_addr;

    i2c_init(g_i2c, 100 * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // Read chip ID
    uint8_t id = 0;
    if (i2c_reg_read(0x00, &id, 1) < 0) return -1;
    printf("BMP388 CHIP_ID = 0x%02X\n", id);
    if (id != 0x50) return -2;

    // Soft reset
    i2c_reg_write(0x7E, 0xB6);
    sleep_ms(20);

    // OSR: temp x2, press x4
    i2c_reg_write(0x1C, 0x15);  // 0b00010101

    // ODR: set to 50Hz
    i2c_reg_write(0x1D, 0x09);

    // PWR_CTRL: enable temp+press + NORMAL mode
    i2c_reg_write(0x1B, 0x33);

    sleep_ms(40); // give sensor time to start

    g_init = true;
    return 0;
}

int bmp388_read(bmp388_sample_t *out) {
    if (!g_init) return -1;

    // Wait until data ready
    uint8_t status = 0;
    for (int tries=0; tries<10; tries++) {
        if (i2c_reg_read(0x03, &status, 1) < 0) return -2;
        if ((status & 0x60) == 0x60) break;
        sleep_ms(10);
    }
    if ((status & 0x60) != 0x60) return -3;

    uint8_t buf[6];
    if (i2c_reg_read(0x04, buf, 6) < 0) return -4;

    int32_t p_raw = ((int32_t)buf[2] << 16) | ((int32_t)buf[1] << 8) | buf[0];
    p_raw &= 0xFFFFF;
    int32_t t_raw = ((int32_t)buf[5] << 16) | ((int32_t)buf[4] << 8) | buf[3];
    t_raw &= 0xFFFFF;

    if (p_raw == 0 && t_raw == 0) return -5;

    // Simple scaling (not calibrated!)
    float T = t_raw / 51200.0f;
    float P = p_raw / 16.0f;

    double ratio = (double)P / 101325.0;
    double alt_m = 44330.0 * (1.0 - pow(ratio, 0.1903));

    out->temperature_c = T;
    return 0;
}
