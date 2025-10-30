#ifndef BMP388_DRIVER_H
#define BMP388_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float temperature_c;
} bmp388_sample_t;

/* Sensor API */
int bmp388_init(int i2c_port, uint32_t sda_pin, uint32_t scl_pin, uint8_t i2c_addr);
int bmp388_read(bmp388_sample_t *out);

/* ===== Flash logging: ultra-compact (4 bytes/record) =====
 * Record = 4 bytes:
 *   - dtime_10ms: uint16 time delta since previous sample in 10 ms units (0..65535 -> 0..655.35 s)
 *   - temp_c_centi: int16 temperature in centi-degC (e.g., 23.45 °C -> 2345)
 * Header keeps a base start time in ms so you can reconstruct absolute times.
 */
void     bmp388_storage_init(bool erase_all);
int      bmp388_storage_append(uint32_t time_ms, float temperature_c);
uint32_t bmp388_storage_count(void);
/* Reconstruct record i (0-based). out_ms is absolute ms (from boot),
 * rebuilt by summing deltas up to i (O(i)). Returns 0 on success. */
int      bmp388_storage_read(uint32_t index, uint32_t *out_ms, float *out_temp);
void     bmp388_storage_erase_all(void);

/* ===== Backup (dual-region rotation) =====
 * BACKUP stores the most recent full ACTIVE log.
 */
void     bmp388_storage_rotate_to_backup(void);        /* Force rotation: copy ACTIVE -> BACKUP, then erase ACTIVE */
uint32_t bmp388_backup_count(void);                    /* Number of records in BACKUP (0 if none) */
int      bmp388_backup_read(uint32_t index, uint32_t *out_ms, float *out_temp);  /* Read a record from BACKUP */
void     bmp388_backup_clear(void);                    /* Erase BACKUP region */
#endif
