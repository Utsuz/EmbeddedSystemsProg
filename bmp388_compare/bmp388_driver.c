#include "bmp388_driver.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

/* ===== BMP388 register subset ===== */
#define BMP388_REG_CHIP_ID      0x00
#define BMP388_CHIP_ID          0x50
#define BMP388_REG_ERR_REG      0x02
#define BMP388_REG_STATUS       0x03
#define BMP388_REG_DATA         0x04  // P3 then T3 (6 bytes total)
#define BMP388_REG_PWR_CTRL     0x1B
#define BMP388_REG_OSR          0x1C
#define BMP388_REG_ODR          0x1D
#define BMP388_REG_CMD          0x7E
#define BMP388_CMD_SOFTRESET    0xB6

/* Calibration (temperature) block */
#define BMP388_REG_CALIB_T1     0x31  // T1_LSB at 0x31 .. T3 at 0x35

/* Driver state */
static i2c_inst_t *g_i2c = i2c0;
static uint8_t g_addr = 0x77;
static bool g_ready = false;

/* Bosch float-model coefficients for temperature */
typedef struct {
    float par_t1;  // T1 / 2^8
    float par_t2;  // T2 / 2^28  (NOTE: using 2^28 fixes low temp on your trims)
    float par_t3;  // T3 / 2^48
    float t_lin;   // saved for pressure path (unused here)
} bmp388_tcal_t;

static bmp388_tcal_t g_tcal = {0};

/* ===== I2C helpers ===== */
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

/* ===== Calibration ===== */
static int read_temp_calib(void) {
    uint8_t b[5] = {0};
    int r = i2c_reg_read(BMP388_REG_CALIB_T1, b, sizeof b);
    if (r < 0) return r;

    /* raw NVM */
    uint16_t T1 = (uint16_t)((uint16_t)b[1] << 8 | b[0]);
    uint16_t T2 = (uint16_t)((uint16_t)b[3] << 8 | b[2]);
    int8_t   T3 = (int8_t)b[4];

    if (T1 == 0 || T2 == 0) return -10;  // sanity

    /* Convert to float coefficients (this matches your part’s trims) */
    g_tcal.par_t1 = (float)T1 / 256.0f;                 // 2^8
    g_tcal.par_t2 = (float)T2 / 268435456.0f;           // 2^28  (key fix)
    g_tcal.par_t3 = (float)T3 / 281474976710656.0f;     // 2^48
    g_tcal.t_lin  = 0.0f;

    return 0;
}

/* Bosch float reference (returns °C).
 * t_raw20 is 20-bit unsigned temperature.
 */
static float compensate_temperature(uint32_t t_raw20) {
    float partial1 = (float)t_raw20 - g_tcal.par_t1;
    float partial2 = partial1 * g_tcal.par_t2;
    g_tcal.t_lin   = partial2 + (partial1 * partial1) * g_tcal.par_t3;
    return g_tcal.t_lin;
}

/* ===== Public API ===== */
int bmp388_init(int i2c_port, uint32_t sda_pin, uint32_t scl_pin, uint8_t i2c_addr) {
    g_i2c  = (i2c_port == 1) ? i2c1 : i2c0;
    g_addr = i2c_addr;

    /* Bring up I2C @ 100k first */
    i2c_init(g_i2c, 100 * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    sleep_ms(5);

    /* Verify CHIP_ID */
    uint8_t id = 0;
    if (i2c_reg_read(BMP388_REG_CHIP_ID, &id, 1) < 0) return -1;
    printf("BMP388 CHIP_ID = 0x%02X\n", id);
    if (id != BMP388_CHIP_ID) return -2;

    /* Soft reset and basic config */
    i2c_reg_write_u8(BMP388_REG_CMD, BMP388_CMD_SOFTRESET);
    sleep_ms(30);

    /* modest OSR/ODR; enable temp+press, NORMAL mode */
    i2c_reg_write_u8(BMP388_REG_OSR, 0x01);      // temp x2, press x1
    i2c_reg_write_u8(BMP388_REG_ODR, 0x09);      // ~25–50 Hz
    i2c_reg_write_u8(BMP388_REG_PWR_CTRL, 0x33); // sensors on, NORMAL
    sleep_ms(50);

    /* Read temperature calibration */
    int rc = read_temp_calib();
    if (rc < 0) {
        printf("Calibration read failed: %d\n", rc);
        return rc;
    }

    g_ready = true;
    return 0;
}

int bmp388_read(bmp388_sample_t *out) {
    if (!g_ready) return -1;

    /* Wait for TEMP ready (bit 5). Don’t require pressure. */
    uint8_t st = 0;
    // Wait up to ~1s for a new temp sample (200 * 5ms)
    for (int i = 0; i < 200; i++) {
        if (i2c_reg_read(BMP388_REG_STATUS, &st, 1) < 0) return -2;
        if (st & 0x20) break;              // TEMP_DRDY
        sleep_ms(5);
    }
    if ((st & 0x20) == 0) return -3;


    /* Burst read 6 data bytes: P(3), T(3) */
    uint8_t buf[6];
    if (i2c_reg_read(BMP388_REG_DATA, buf, sizeof buf) < 0) return -4;

    /* Unpack 20-bit temps (unsigned) */
    uint32_t t_raw = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];
    t_raw &= 0xFFFFF;
    if (t_raw == 0) return -5;

    out->temperature_c = compensate_temperature(t_raw);
    return 0;
}
