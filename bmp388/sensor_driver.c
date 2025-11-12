// bmp388_sensor.c
#include "bmp388_driver.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ===== BMP388 register subset ===== */
#define BMP388_REG_CHIP_ID      0x00
#define BMP388_CHIP_ID          0x50
#define BMP388_REG_ERR_REG      0x02
#define BMP388_REG_STATUS       0x03
#define BMP388_REG_DATA         0x04
#define BMP388_REG_PWR_CTRL     0x1B
#define BMP388_REG_OSR          0x1C
#define BMP388_REG_ODR          0x1D
#define BMP388_REG_CMD          0x7E
#define BMP388_CMD_SOFTRESET    0xB6
#define BMP388_REG_CALIB_T1     0x31

/* ===== ODR presets (slow for baseline, fast for excursion) =====
 * These values are practical defaults. If you have a preferred mapping
 * from the datasheet, feel free to adjust.
 */
#define ODR_BASELINE  0x0A   /* slower output data rate (baseline) */
#define ODR_FAST      0x08   /* faster output data rate (excursion) */

static i2c_inst_t *g_i2c = i2c0;
static uint8_t g_addr = 0x77;
static bool g_ready = false;
static uint8_t g_cur_odr = ODR_BASELINE;

/* ===== Excursion detection settings (2–8 °C, 5 stable samples to recover) ===== */
static float    g_t_low_c   = 22.0f;
static float    g_t_high_c  = 23.5f;
static uint32_t g_stable_needed = 1;
static bool     g_in_excursion  = false;
static bool     g_sensor_active = false;
static uint32_t g_stable_count  = 0;

/* Temperature calibration */
typedef struct {
    float par_t1, par_t2, par_t3, t_lin;
} bmp388_tcal_t;
static bmp388_tcal_t g_tcal = {0};

static int i2c_reg_write_u8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    int r = i2c_write_timeout_us(g_i2c, g_addr, buf, 2, false, 2000);
    return (r < 0) ? r : 0;
}
static int i2c_reg_read(uint8_t reg, uint8_t *buf, size_t len) {
    int w = i2c_write_timeout_us(g_i2c, g_addr, &reg, 1, true, 2000);
    if (w < 0) return w;
    int r = i2c_read_timeout_us(g_i2c, g_addr, buf, len, false, 2000);
    return (r < 0) ? r : 0;
}

static int read_temp_calib(void) {
    uint8_t b[5];
    int r = i2c_reg_read(BMP388_REG_CALIB_T1, b, sizeof b);
    if (r < 0) return r;

    uint16_t T1u = (uint16_t)((b[1] << 8) | b[0]);
    int16_t  T2s = (int16_t)((b[3] << 8) | b[2]);
    int8_t   T3s = (int8_t)b[4];

    g_tcal.par_t1 = (float)T1u / 512.0f;
    g_tcal.par_t2 = (float)T2s / 16384.0f;
    g_tcal.par_t3 = (float)T3s / 281474976710656.0f;
    g_tcal.t_lin  = 0.0f;

    printf("T1=%.3f  T2=%.9f  T3=%.12f\n",
           g_tcal.par_t1, g_tcal.par_t2, g_tcal.par_t3);
    return 0;
}

static float compensate_temperature(uint32_t t_raw) {
    float p1 = ((float)t_raw / 16384.0f) - (g_tcal.par_t1 / 1024.0f);
    g_tcal.t_lin = p1 * g_tcal.par_t2 + (p1 * p1) * g_tcal.par_t3;
    /* project-specific offset to align with your environment */
    g_tcal.t_lin -= 14.0f;
    return g_tcal.t_lin;
}

/* ===== ODR helper ===== */
static void set_odr_if_needed(uint8_t odr_val) {
    if (g_cur_odr == odr_val) return;
    if (i2c_reg_write_u8(BMP388_REG_ODR, odr_val) == 0) {
        g_cur_odr = odr_val;
        printf("[BMP388] ODR set to 0x%02X (%s)\n",
               g_cur_odr, (g_cur_odr == ODR_FAST ? "FAST" : "BASELINE"));
        /* give sensor a moment to settle at new ODR */
        sleep_ms(5);
    } else {
        printf("[BMP388] WARN: failed to set ODR=0x%02X\n", odr_val);
    }
}

/* ===== Excursion helpers =====
 * Returns true if state changed (entered or exited excursion)
 */
static bool excursion_update_from_temp(float celsius) {
    bool prev = g_in_excursion;

    if (!g_in_excursion) {
        if (celsius < g_t_low_c || celsius > g_t_high_c) {
            g_in_excursion = true;
            g_stable_count = 0;
        }
    } else {
        if (celsius >= g_t_low_c && celsius <= g_t_high_c) {
            if (++g_stable_count >= g_stable_needed) {
                g_in_excursion = false;
                g_stable_count = 0;
            }
        } else {
            g_stable_count = 0;
        }
    }
    return (g_in_excursion != prev);
}

/* ==== Public sensor API (matches bmp388_driver.h) ==== */
int bmp388_init(int i2c_port, uint32_t sda_pin, uint32_t scl_pin, uint8_t i2c_addr) {
    g_i2c  = (i2c_port == 1) ? i2c1 : i2c0;
    g_addr = i2c_addr;

    i2c_init(g_i2c, 100 * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    sleep_ms(5);

    uint8_t id = 0;
    if (i2c_reg_read(BMP388_REG_CHIP_ID, &id, 1) < 0) return -1;
    if (id != BMP388_CHIP_ID) return -2;

    i2c_reg_write_u8(BMP388_REG_CMD, BMP388_CMD_SOFTRESET);
    sleep_ms(30);
    i2c_reg_write_u8(BMP388_REG_OSR, 0x01);
    i2c_reg_write_u8(BMP388_REG_ODR, ODR_BASELINE);  // start in baseline ODR
    g_cur_odr = ODR_BASELINE;
    i2c_reg_write_u8(BMP388_REG_PWR_CTRL, 0x33);
    sleep_ms(50);

    int rc = read_temp_calib();
    if (rc < 0) return rc;

    g_ready = true;
    return 0;
}

int bmp388_read(bmp388_sample_t *out) {
    if (!g_ready) return -1;

    uint8_t st = 0;
    for (int i=0; i<200; i++) {
        if (i2c_reg_read(BMP388_REG_STATUS, &st, 1) < 0) return -2;
        if (st & 0x40) break;
        sleep_ms(5);
    }
    if ((st & 0x40) == 0) return -3;

    uint8_t buf[6];
    if (i2c_reg_read(BMP388_REG_DATA, buf, sizeof buf) < 0) return -4;

    uint32_t t_raw = ((uint32_t)buf[5] << 12) | ((uint32_t)buf[4] << 4) | ((uint32_t)buf[3] >> 4);
    if (t_raw == 0) return -5;

    out->temperature_c = compensate_temperature(t_raw);

    /* Update excursion state on every reading */
    bool changed = excursion_update_from_temp(out->temperature_c);
    if (changed) {
        if (g_in_excursion) {
            printf("** EXCURSION detected (T=%.2f °C). Using FAST ODR. **\n", out->temperature_c);
            set_odr_if_needed(ODR_FAST);
        } else {
            printf("** RECOVERY to normal range (T=%.2f °C). Using BASELINE ODR. **\n", out->temperature_c);
            set_odr_if_needed(ODR_BASELINE);
        }
    }

    return 0;
}

/* ===== Optional public helpers (add prototypes to bmp388_driver.h if needed) =====
   void bmp388_excursion_config(float t_low_c, float t_high_c, uint32_t stable_samples);
   bool bmp388_excursion_state(void);
   void bmp388_excursion_reset(void);
*/
void bmp388_excursion_config(float t_low_c, float t_high_c, uint32_t stable_samples) {
    g_t_low_c = t_low_c;
    g_t_high_c = t_high_c;
    g_stable_needed = (stable_samples == 0) ? 1 : stable_samples;
}
bool bmp388_excursion_state(void) {
    return g_in_excursion;
}
void bmp388_excursion_reset(void) {
    g_in_excursion = false;
    g_stable_count = 0;
    set_odr_if_needed(ODR_BASELINE);
}

/* Sensor control functions. Default: False */

void bmp388_sensorStop(void) {
    if (!g_ready || !g_sensor_active) return;

    // 0x1B [PWR_CTRL]: bit0=press_en, bit1=temp_en, bit4=sensor_en
    // Writing 0x00 disables measurement (standby)
    if (i2c_reg_write_u8(BMP388_REG_PWR_CTRL, 0x00) == 0) {
        g_sensor_active = false;
        printf("[BMP388] Sensor stopped (standby mode).\n");
    } else {
        printf("[BMP388] WARN: Failed to enter standby mode!\n");
    }
}

void bmp388_sensorStart(void) {
    if (!g_ready) {
        printf("[BMP388] Cannot start — sensor not initialized!\n");
        return;
    }
    if (g_sensor_active) return;

    // Re-enable both pressure & temperature measurements (0x33 = press+temp+sensor_en)
    if (i2c_reg_write_u8(BMP388_REG_PWR_CTRL, 0x33) == 0) {
        sleep_ms(50);
        set_odr_if_needed(g_cur_odr);  // restore last known ODR (baseline or fast)
        g_sensor_active = true;
        printf("[BMP388] Sensor started (ODR=0x%02X).\n", g_cur_odr);
    } else {
        printf("[BMP388] WARN: Failed to start sensor!\n");
    }
}
