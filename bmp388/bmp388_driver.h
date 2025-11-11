#ifndef BMP388_DRIVER_H
#define BMP388_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h> 

typedef struct {
    float temperature_c;
} bmp388_sample_t;

/* Sensor API */
int bmp388_init(int i2c_port, uint32_t sda_pin, uint32_t scl_pin, uint8_t i2c_addr);
int bmp388_read(bmp388_sample_t *out);

/* Control Functions */
void bmp388_sensorStart(void);
void bmp388_sensorStop(void);

/* ===== Storage (ACTIVE) =====
 * Append fixed-size records (delta time + temperature) to the ACTIVE region.
 */
int      bmp388_storage_init(void);                     /* Initialize header if blank; returns 0 on success */
uint32_t bmp388_storage_count(void);                    /* Number of records in ACTIVE */
int      bmp388_storage_append(uint32_t time_ms, float temperature_c); /* Append a record; returns 0 on success */
int      bmp388_storage_read(uint32_t index, uint32_t *out_ms, float *out_temp);
void     bmp388_storage_erase_all(void);

/* ===== Backup (dual-region rotation) =====
 * BACKUP stores the most recent full ACTIVE log.
 * (Keep these if you still use the old rotation; safe to ignore otherwise.)
 */
void     bmp388_storage_rotate_to_backup(void);
uint32_t bmp388_backup_count(void);
int      bmp388_backup_read(uint32_t index, uint32_t *out_ms, float *out_temp);
void     bmp388_backup_clear(void);

void dump_active(void); /* legacy text dump, safe to ignore */

/* ==== NEW: Shared compact-bit encoder & helpers ==== */
typedef void (*cbit_emit_fn)(uint8_t byte, void *ctx);

/* Builds the exact compact-bit stream and emits each byte via callback. */
size_t   bmp388_compact_encode(cbit_emit_fn emit, void *ctx);

/* UART dumper (identical bytes to encoder) */
void     dump_active_compact_bit(void);

/* Flash save / load of the exact compact-bit stream */
void     bmp388_backup_compact_save(void);
void     dump_backup_compact_raw(void);

#endif
