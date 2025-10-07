// SPDX-License-Identifier: MIT
// BMP388 quick probe on Maker Pi Pico (Rev 1.2) using i2c0 @ GP4/GP5

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT       i2c0
#define I2C_SDA_PIN    4      // GP4 on Maker Pi Pico bottom GROVE
#define I2C_SCL_PIN    5      // GP5 on Maker Pi Pico bottom GROVE
#define I2C_BAUD       100000 // 100 kHz is safe for first bring-up

// BMP388 addresses (selected by SDO pin: GND=0x76, VDDIO=0x77)
#define BMP388_ADDR_LOW   0x76
#define BMP388_ADDR_HIGH  0x77
#define BMP388_REG_CHIP_ID 0x00
#define BMP388_CHIP_ID     0x50

static int i2c_probe_addr(uint8_t addr, uint32_t timeout_us) {
    // Try a zero-length write; device that NACKs will return PICO_ERROR_GENERIC
    int ret = i2c_write_timeout_us(I2C_PORT, addr, NULL, 0, false, timeout_us);
    return ret; // >=0 means something responded
}

static int i2c_read_u8(uint8_t addr, uint8_t reg, uint8_t *val) {
    int ret = i2c_write_blocking(I2C_PORT, addr, &reg, 1, true); // no STOP
    if (ret < 0) return ret;
    ret = i2c_read_blocking(I2C_PORT, addr, val, 1, false);      // with STOP
    return ret;
}

static void scan_bus(void) {
    printf("I2C sweeping (timeout-safe)...\n");
    printf("Scanning i2c0 on SDA GP%d / SCL GP%d @ %ukHz...\n",
           I2C_SDA_PIN, I2C_SCL_PIN, I2C_BAUD/1000);

    // Check idle line levels (external pull-ups)
    bool sda_high = gpio_get(I2C_SDA_PIN);
    bool scl_high = gpio_get(I2C_SCL_PIN);
    printf("  Line levels: SDA=%s SCL=%s\n",
           sda_high ? "HIGH" : "LOW", scl_high ? "HIGH" : "LOW");

    int found = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        int ret = i2c_probe_addr(addr, 800); // short timeout
        if (ret >= 0) {
            printf("  Found device at 0x%02X\n", addr);
            found++;
        }
        sleep_us(200);
    }
    if (!found) printf("  (no devices)\n");
}

int main() {
    stdio_init_all();
    sleep_ms(800); // allow USB CDC to come up

    printf("USB up. I2C bring-up...\n");

    // I2C init and pin mux
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    // Use pull-ups in case your cable/module lacks them (Maker Pi already has)
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    scan_bus();

    // Try BMP388 at both legal addresses
    const uint8_t try_addrs[2] = { BMP388_ADDR_LOW, BMP388_ADDR_HIGH };
    bool ok = false;

    for (int i = 0; i < 2; i++) {
        uint8_t addr = try_addrs[i];
        uint8_t id   = 0x00;

        printf("Probing BMP388 at 0x%02X...\n", addr);
        if (i2c_probe_addr(addr, 2000) < 0) {
            printf("  No ACK at 0x%02X (check SDO pin / wiring)\n", addr);
            continue;
        }
        int r = i2c_read_u8(addr, BMP388_REG_CHIP_ID, &id);
        if (r < 0) {
            printf("  Read failed at 0x%02X (err %d)\n", addr, r);
            continue;
        }
        printf("  CHIP_ID=0x%02X\n", id);
        if (id == BMP388_CHIP_ID) {
            printf("  ✅ BMP388 detected at 0x%02X\n", addr);
            ok = true;
            break;
        } else {
            printf("  ⚠️ Unexpected ID (want 0x50). Is this a different sensor?\n");
        }
    }

    if (!ok) {
        printf("\nBMP388 init failed: device not found or wrong CHIP_ID.\n");
        printf("Checklist:\n");
        printf(" - GND ↔ GND (black)\n");
        printf(" - VCC ↔ 3V3 (red) — NOT 5V\n");
        printf(" - SDA ↔ GP4 (green)\n");
        printf(" - SCL ↔ GP5 (orange)\n");
        printf(" - SDO pin sets I2C address (GND=0x76, VDDIO=0x77)\n");
        printf(" - Do NOT use SWCLK/SWDIO for I2C\n");
    }

    // Sit here so logs stay visible
    while (true) {
        tight_loop_contents();
    }
}
