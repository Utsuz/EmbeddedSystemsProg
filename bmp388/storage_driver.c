// storage_driver.c — ACTIVE logging only + NEW binary compact dump (reads from RAM mirror, not storage)
// - Removes BACKUP and all old/text dump methods
// - Keeps ACTIVE storage (append) but dump_active_compact_bit() now uses an in-RAM log (no bmp388_storage_read())

#include "bmp388_driver.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* ===== Tunables ===== */
#ifndef LOG_REGION_SIZE
#define LOG_REGION_SIZE   (128 * 1024)    /* size of ACTIVE log region */
#endif

/* On-flash header signature */
#define LOG_MAGIC         0x424D5039u     /* 'BMP9' */
#define LOG_VERSION       2u

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

/* Flash layout (ACTIVE only at end of flash)
 * [ ... firmware ... ][ ACTIVE (LOG_REGION_SIZE) ]
 */
#define LOG_FLASH_OFFSET  (PICO_FLASH_SIZE_BYTES - LOG_REGION_SIZE) /* ACTIVE base */
#define LOG_XIP_BASE      (XIP_BASE + LOG_FLASH_OFFSET)

/* ===== Excursion thresholds (toggle markers in binary dump) ===== */
#ifndef EXCUR_T_LOW_C
#define EXCUR_T_LOW_C   22.0f
#endif
#ifndef EXCUR_T_HIGH_C
#define EXCUR_T_HIGH_C  24.5f
#endif
static inline bool is_excursion(float c) { return (c < EXCUR_T_LOW_C) || (c > EXCUR_T_HIGH_C); }

/* ===== Opcodes for binary compact dump ===== */
#define OPC_TS   0xF0  /* timestamp marker (we emit ONLY ONCE at start) */
#define OPC_E    0xE5  /* excursion toggle opcode */
#define OPC_END  0xEE  /* end marker */

/* ===== 4-byte compact record (delta-time + temperature) for ACTIVE storage ===== */
typedef struct __attribute__((packed)) {
    uint16_t dtime_10ms;     /* elapsed time since previous record, in 10 ms units */
    int16_t  temp_c_centi;   /* temperature in centi-degC */
} log_rec4_t;

#define RECORD_SIZE       ((uint32_t)sizeof(log_rec4_t))
#define RECORDS_PER_PAGE  (FLASH_PAGE_SIZE / RECORD_SIZE)

/* Header (first page in region) */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t capacity_bytes;   /* bytes for data (excl. header page) */
    uint32_t count;            /* number of records */
    uint32_t base_time_ms;     /* absolute ms for record #0 */
    uint32_t reserved32[7];    /* pad; remainder of the page is 0xFF */
} log_header_t;

/* ===== Runtime state for ACTIVE ===== */
static log_header_t g_hdr;
static uint8_t  g_page_buf[FLASH_PAGE_SIZE];
static uint32_t g_page_fill = 0;     /* bytes in current RAM page */
static uint32_t g_write_pos = 0;     /* bytes written to data area */
static uint32_t g_last_time_ms = 0;
static bool     g_have_anchor = false;

/* ===== NEW: In-RAM mirror log (used by dumper; avoids bmp388_storage_read) =====
   Choose a capacity that suits your RAM budget. If it fills, it overwrites oldest (ring). */
#ifndef RAMLOG_CAP
#define RAMLOG_CAP  8192u   /* number of samples kept in RAM */
#endif

typedef struct {
    uint32_t t_ms[RAMLOG_CAP];
    float    temp[RAMLOG_CAP];
    uint32_t head;   /* index of oldest element */
    uint32_t count;  /* # of valid elements (<= RAMLOG_CAP) */
} ramlog_t;

static ramlog_t g_ramlog = {0};

static inline void ramlog_clear(void) {
    g_ramlog.head = 0;
    g_ramlog.count = 0;
}

static inline void ramlog_add(uint32_t t_ms, float temp) {
    uint32_t pos;
    if (g_ramlog.count < RAMLOG_CAP) {
        pos = (g_ramlog.head + g_ramlog.count) % RAMLOG_CAP;
        g_ramlog.count++;
    } else {
        /* overwrite oldest */
        pos = g_ramlog.head;
        g_ramlog.head = (g_ramlog.head + 1) % RAMLOG_CAP;
    }
    g_ramlog.t_ms[pos] = t_ms;
    g_ramlog.temp[pos] = temp;
}

static inline uint32_t ramlog_count(void) {
    return g_ramlog.count;
}

/* Get ith element in chronological order: i=0 oldest ... i=count-1 newest */
static inline void ramlog_get(uint32_t i, uint32_t *t_ms, float *temp) {
    uint32_t pos = (g_ramlog.head + i) % RAMLOG_CAP;
    if (t_ms) *t_ms = g_ramlog.t_ms[pos];
    if (temp) *temp = g_ramlog.temp[pos];
}

/* ===== XIP helpers ===== */
static inline const log_header_t *xip_hdr_active(void)   { return (const log_header_t *)(LOG_XIP_BASE); }
static inline const uint8_t      *xip_data_active(void)  { return (const uint8_t *)(LOG_XIP_BASE + FLASH_PAGE_SIZE); }

/* ===== Flash helpers ===== */
static void flash_program_block(uint32_t flash_off, const void *data, size_t len) {
    uint32_t irq = save_and_disable_interrupts();
    flash_range_program(flash_off, (const uint8_t*)data, len);
    restore_interrupts(irq);
}
static void flash_erase_sectors(uint32_t flash_off, size_t len) {
    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(flash_off, len);
    restore_interrupts(irq);
}
static void write_header_to_flash_active(const log_header_t *h) {
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    memcpy(page, h, sizeof *h);
    flash_program_block(LOG_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
}

/* ===== ACTIVE init / erase ===== */
static void storage_reset_state_after_erase(void) {
    memset(&g_hdr, 0, sizeof g_hdr);
    g_hdr.magic          = LOG_MAGIC;
    g_hdr.version        = LOG_VERSION;
    g_hdr.capacity_bytes = LOG_REGION_SIZE - FLASH_PAGE_SIZE; /* first page is header */
    g_hdr.count          = 0;
    g_hdr.base_time_ms   = 0;

    g_write_pos    = 0;
    g_page_fill    = 0;
    g_last_time_ms = 0;
    g_have_anchor  = false;

    memset(g_page_buf, 0xFF, sizeof g_page_buf);
    write_header_to_flash_active(&g_hdr);

    /* Also clear the RAM log */
    ramlog_clear();
}

void bmp388_storage_erase_all(void) {
    flash_erase_sectors(LOG_FLASH_OFFSET, LOG_REGION_SIZE);
    storage_reset_state_after_erase();
}

void bmp388_storage_init(bool erase_all) {
    const log_header_t *h = xip_hdr_active();
    bool ok = (h->magic == LOG_MAGIC) && (h->version == LOG_VERSION);

    if (erase_all || !ok) {
        bmp388_storage_erase_all();
        return;
    }

    g_hdr       = *h;
    g_write_pos = g_hdr.count * RECORD_SIZE;
    g_page_fill = g_write_pos % FLASH_PAGE_SIZE;
    memset(g_page_buf, 0xFF, sizeof g_page_buf);

    g_have_anchor  = (g_hdr.count > 0) || (g_hdr.base_time_ms != 0);
    g_last_time_ms = g_hdr.base_time_ms;

    /* NOTE: We do NOT reconstruct RAM log from flash here (reader is bugged).
       RAM log will accumulate from new appends after boot. */
}

/* ===== Page flush ===== */
static int flush_page_if_full(void) {
    if (g_page_fill < FLASH_PAGE_SIZE) return 0;

    uint32_t bytes_before_this_page = g_write_pos - FLASH_PAGE_SIZE;
    uint32_t page_index = bytes_before_this_page / FLASH_PAGE_SIZE;
    uint32_t flash_off = LOG_FLASH_OFFSET + FLASH_PAGE_SIZE + page_index * FLASH_PAGE_SIZE;

    flash_program_block(flash_off, g_page_buf, FLASH_PAGE_SIZE);
    memset(g_page_buf, 0xFF, sizeof g_page_buf);
    g_page_fill = 0;

    /* persist header (count) after each page flush */
    write_header_to_flash_active(&g_hdr);
    return 0;
}

/* ===== ACTIVE append (also mirrors to RAM log) ===== */
int bmp388_storage_append(uint32_t time_ms, float temperature_c) {
    if (!g_have_anchor) {
        g_hdr.base_time_ms = time_ms;
        g_last_time_ms     = time_ms;
        g_have_anchor      = true;
        write_header_to_flash_active(&g_hdr);
    }

    /* If ACTIVE is full, just wrap by erasing (since BACKUP is removed). */
    if (g_write_pos + RECORD_SIZE > g_hdr.capacity_bytes) {
        bmp388_storage_erase_all();
        // After erase, anchor will be set by next append
        return -1;
    }

    uint32_t delta_ms = (time_ms >= g_last_time_ms) ? (time_ms - g_last_time_ms) : 0;
    uint32_t delta_10 = delta_ms / 10u;  if (delta_10 > 0xFFFFu) delta_10 = 0xFFFFu;

    int32_t t_centi = (int32_t)(temperature_c * 100.0f);
    if (t_centi >  32767) t_centi =  32767;
    if (t_centi < -32768) t_centi = -32768;

    log_rec4_t rec;
    rec.dtime_10ms   = (uint16_t)delta_10;
    rec.temp_c_centi = (int16_t)t_centi;

    memcpy(&g_page_buf[g_page_fill], &rec, sizeof rec);
    g_page_fill  += sizeof rec;
    g_write_pos  += sizeof rec;
    g_hdr.count  += 1;
    g_last_time_ms = g_last_time_ms + (uint32_t)rec.dtime_10ms * 10u;

    /* Mirror to RAM log for robust dumping */
    ramlog_add(time_ms, temperature_c);

    return flush_page_if_full();
}

/* ===== Corrected 0.5 °C bucket mapper =====
   Centers buckets on 22.5 °C, 23.0 °C, 23.5 °C, 24.0 °C, 24.5 °C, etc.
   Each bucket covers ±0.25 °C around its midpoint.
   Below 22.25 °C → bucket 0, above 28.25 °C → bucket 12.
*/
static inline uint8_t bucket_0p5_22_28(float c) {
    const float minC = 22.5f;   // midpoint of first bucket
    const float step = 0.5f;    // step between buckets
    int idx;

    if (c < (minC - step)) {    // below 22.0 °C
        idx = 0;
    } else if (c >= (minC + 12 * step)) { // above 28.5 °C
        idx = 12;
    } else {
        // round to nearest bucket center
        idx = (int)((c - minC) / step + 0.5f) + 1;
        if (idx < 0) idx = 0;
        if (idx > 12) idx = 12;
    }
    return (uint8_t)idx;
}


/* ===== NEW: Binary compact dump (from RAM log, not flash) =====
   - Header: 0xC1, 0x01, count(LE), bucket_count=13
   - FIRST timestamp only (OPC_TS + u32le) from RAM oldest
   - OPC_E on excursion toggles (byte-aligned)
   - Two 4-bit buckets per byte (high nibble then low)
   - Ends with OPC_END
*/
void dump_active_compact_bit(void) {
    uint32_t n = ramlog_count();

    /* header */
    putchar_raw(0xC1);
    putchar_raw(0x01);
    putchar_raw((uint8_t)(n & 0xFF));
    putchar_raw((uint8_t)(n >> 8));
    putchar_raw(13);  /* 13 buckets: 0..12 (22..28 °C by 0.5) */

    if (n == 0) { putchar_raw(OPC_END); return; }

    /* Emit ONLY the first timestamp from RAM oldest */
    uint32_t t0_ms; float t0_temp;
    ramlog_get(0, &t0_ms, &t0_temp);
    putchar_raw(OPC_TS);
    putchar_raw((uint8_t)(t0_ms));
    putchar_raw((uint8_t)(t0_ms >> 8));
    putchar_raw((uint8_t)(t0_ms >> 16));
    putchar_raw((uint8_t)(t0_ms >> 24));

    bool have_prev = false, prev_exc = false;
    uint8_t acc = 0;           /* accumulator for two 4-bit nibbles */
    bool high_nibble = true;   /* true = write high nibble next */

    for (uint32_t i = 0; i < n; i++) {
        uint32_t t_ms; float temp;
        ramlog_get(i, &t_ms, &temp);

        /* excursion toggle handling (byte-align before opcode) */
        bool exc = is_excursion(temp);
        if (!have_prev) {
            have_prev = true;
            prev_exc  = exc;
        } else if (exc != prev_exc) {
            if (!high_nibble) { putchar_raw(acc); acc = 0; high_nibble = true; }
            putchar_raw(OPC_E);
            prev_exc = exc;
        }

        /* map 22..28 in 0.5 °C steps (0..12) and pack two per byte */
        uint8_t b = bucket_0p5_22_28(temp) & 0x0F;
        if (high_nibble) {
            acc = (uint8_t)(b << 4);
            high_nibble = false;
        } else {
            acc |= b;
            putchar_raw(acc);
            acc = 0;
            high_nibble = true;
        }
    }

    /* flush trailing nibble if odd sample count */
    if (!high_nibble) putchar_raw(acc);

    putchar_raw(OPC_END);
}
