// storage_driver.c — ACTIVE logging + persistent BACKUP region (dual-region flash)

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
#define LOG_REGION_SIZE   (128 * 1024)    /* size per region: ACTIVE and BACKUP */
#endif

#define FLASH_PAGE_SIZE   256
#define FLASH_SECTOR_SIZE 4096

/* On-flash header signature */
#define LOG_MAGIC         0x424D5039u     /* 'BMP9' */
#define LOG_VERSION       2u

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

/* Flash layout (at the end of flash)
 *
 * [ ... firmware ... ][ BACKUP (128KB) ][ ACTIVE (128KB) ]
 * ACTIVE always at the very end. BACKUP sits just before it.
 */
#define LOG_FLASH_OFFSET     (PICO_FLASH_SIZE_BYTES - LOG_REGION_SIZE)          /* ACTIVE base */
#define BACKUP_FLASH_OFFSET  (LOG_FLASH_OFFSET - LOG_REGION_SIZE)               /* BACKUP base */

#define LOG_XIP_BASE         (XIP_BASE + LOG_FLASH_OFFSET)
#define BACKUP_XIP_BASE      (XIP_BASE + BACKUP_FLASH_OFFSET)

/* ===== Excursion thresholds (for compact text outputs) ===== */
#ifndef EXCUR_T_LOW_C
#define EXCUR_T_LOW_C   22.0f
#endif
#ifndef EXCUR_T_HIGH_C
#define EXCUR_T_HIGH_C  23.0f
#endif
static inline bool is_excursion(float c) { return (c < EXCUR_T_LOW_C) || (c > EXCUR_T_HIGH_C); }

/* ===== Opcodes for binary compact dump (kept for ACTIVE bit-dump) ===== */
#define OPC_TS   0xF0  /* timestamp marker (every 50 samples) */
#define OPC_E    0xE5  /* inline 'e' toggle when excursion state flips */
#define OPC_END  0xEE  /* end marker */

/* ===== 4-byte compact record (delta-time + temperature) ===== */
typedef struct __attribute__((packed)) {
    uint16_t dtime_10ms;     /* elapsed time since previous record, in units of 10 ms */
    int16_t  temp_c_centi;   /* temperature in centi-degC */
} log_rec4_t;

#define RECORD_SIZE       ((uint32_t)sizeof(log_rec4_t))
#define RECORDS_PER_PAGE  (FLASH_PAGE_SIZE / RECORD_SIZE)

/* Header (first page in each region) */
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

/* ===== XIP helpers ===== */
static inline const log_header_t *xip_hdr_active(void)   { return (const log_header_t *)(LOG_XIP_BASE); }
static inline const uint8_t      *xip_data_active(void)  { return (const uint8_t *)(LOG_XIP_BASE + FLASH_PAGE_SIZE); }
static inline const log_header_t *xip_hdr_backup(void)   { return (const log_header_t *)(BACKUP_XIP_BASE); }
static inline const uint8_t      *xip_data_backup(void)  { return (const uint8_t *)(BACKUP_XIP_BASE + FLASH_PAGE_SIZE); }

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
static void write_header_to_flash_region(uint32_t region_base_offset, const log_header_t *h) {
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    memcpy(page, h, sizeof *h);
    flash_program_block(region_base_offset, page, FLASH_PAGE_SIZE);
}
static inline void write_header_to_flash_active(const log_header_t *h) {
    write_header_to_flash_region(LOG_FLASH_OFFSET, h);
}
static inline void write_header_to_flash_backup(const log_header_t *h) {
    write_header_to_flash_region(BACKUP_FLASH_OFFSET, h);
}

/* ===== ACTIVE init / erase ===== */
static void storage_reset_state_after_erase(void) {
    memset(&g_hdr, 0, sizeof g_hdr);
    g_hdr.magic          = LOG_MAGIC;
    g_hdr.version        = LOG_VERSION;
    g_hdr.capacity_bytes = LOG_REGION_SIZE - FLASH_PAGE_SIZE; /* first page is header */
    g_hdr.count          = 0;
    g_hdr.base_time_ms   = 0;

    g_write_pos   = 0;
    g_page_fill   = 0;
    g_last_time_ms = 0;
    g_have_anchor  = false;

    memset(g_page_buf, 0xFF, sizeof g_page_buf);
    write_header_to_flash_active(&g_hdr);
}

void bmp388_storage_erase_all(void) {
    flash_erase_sectors(LOG_FLASH_OFFSET, LOG_REGION_SIZE);
    storage_reset_state_after_erase();
}

void bmp388_storage_init(bool erase_all) {
    /* If BACKUP would overlap firmware (very rare), we simply keep running without it. */

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

    if (g_hdr.count > 0) {
        /* reconstruct last_time_ms cheaply by walking committed data */
        uint32_t t = g_hdr.base_time_ms;
        uint32_t full_pages = g_hdr.count / RECORDS_PER_PAGE;
        uint32_t rem        = g_hdr.count % RECORDS_PER_PAGE;

        for (uint32_t p = 0; p < full_pages; ++p) {
            const log_rec4_t *pg = (const log_rec4_t *)(xip_data_active() + p * FLASH_PAGE_SIZE);
            for (uint32_t i=0; i<RECORDS_PER_PAGE; i++) t += (uint32_t)pg[i].dtime_10ms * 10u;
        }
        if (rem) {
            const log_rec4_t *pg = (const log_rec4_t *)(xip_data_active() + full_pages * FLASH_PAGE_SIZE);
            for (uint32_t i=0; i<rem; i++) t += (uint32_t)pg[i].dtime_10ms * 10u;
        }
        g_last_time_ms = t;
    }
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

/* ===== ACTIVE append/read/count ===== */
int bmp388_storage_append(uint32_t time_ms, float temperature_c) {
    if (!g_have_anchor) {
        g_hdr.base_time_ms = time_ms;
        g_last_time_ms     = time_ms;
        g_have_anchor      = true;
        write_header_to_flash_active(&g_hdr);
    }

    /* If ACTIVE is full, auto-rotate to BACKUP first, then restart ACTIVE */
    if (g_write_pos + RECORD_SIZE > g_hdr.capacity_bytes) {
        /* Copy current ACTIVE (including RAM tail) -> BACKUP */
        /* then erase ACTIVE and reset state */
        /* This matches the manual "backup" button behavior but is automatic on overflow. */
        /* (See backup_copy_active_to_backup below.) */
        uint32_t used_bytes = g_write_pos;
        uint32_t used_pages = (used_bytes + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

        /* erase BACKUP */
        flash_erase_sectors(BACKUP_FLASH_OFFSET, LOG_REGION_SIZE);

        /* copy committed pages */
        const uint8_t *active_data = xip_data_active();
        for (uint32_t p = 0; p < (used_pages ? (used_pages - 1) : 0); ++p) {
            const uint8_t *src_page = active_data + p * FLASH_PAGE_SIZE;
            uint32_t dst_off = BACKUP_FLASH_OFFSET + FLASH_PAGE_SIZE + p * FLASH_PAGE_SIZE;
            flash_program_block(dst_off, src_page, FLASH_PAGE_SIZE);
        }
        /* copy last/tail page (from RAM) */
        if (used_pages > 0) {
            uint32_t last_page_index = used_pages - 1;
            uint32_t dst_off = BACKUP_FLASH_OFFSET + FLASH_PAGE_SIZE + last_page_index * FLASH_PAGE_SIZE;

            if ((g_page_fill == 0) && (used_bytes != 0)) {
                const uint8_t *src_page = active_data + last_page_index * FLASH_PAGE_SIZE;
                flash_program_block(dst_off, src_page, FLASH_PAGE_SIZE);
            } else if (g_page_fill > 0) {
                uint8_t page[FLASH_PAGE_SIZE];
                memset(page, 0xFF, sizeof page);
                memcpy(page, g_page_buf, g_page_fill);
                flash_program_block(dst_off, page, FLASH_PAGE_SIZE);
            }
        }
        /* write header last */
        log_header_t bh = g_hdr;
        write_header_to_flash_backup(&bh);

        /* erase+reset ACTIVE */
        bmp388_storage_erase_all();
        return -1; /* tell caller we rotated */
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

    return flush_page_if_full();
}

uint32_t bmp388_storage_count(void) {
    return g_hdr.count;
}

int bmp388_storage_read(uint32_t index, uint32_t *out_ms, float *out_temp) {
    if (index >= g_hdr.count) return -1;

    uint32_t t = g_hdr.base_time_ms;
    uint32_t full_pages = g_hdr.count / RECORDS_PER_PAGE;
    uint32_t committed_records = full_pages * RECORDS_PER_PAGE;

    if (index < committed_records) {
        uint32_t page_index = index / RECORDS_PER_PAGE;
        uint32_t in_page    = index % RECORDS_PER_PAGE;

        for (uint32_t i=0; i<page_index; i++) {
            const log_rec4_t *pg = (const log_rec4_t *)(xip_data_active() + i * FLASH_PAGE_SIZE);
            for (uint32_t j=0; j<RECORDS_PER_PAGE; j++) t += (uint32_t)pg[j].dtime_10ms * 10u;
        }
        const log_rec4_t *pg = (const log_rec4_t *)(xip_data_active() + page_index * FLASH_PAGE_SIZE);
        for (uint32_t j=0; j<=in_page; j++)            t += (uint32_t)pg[j].dtime_10ms * 10u;

        if (out_ms)   *out_ms   = t;
        if (out_temp) *out_temp = (float)pg[in_page].temp_c_centi / 100.0f;
        return 0;
    }

    /* tail area is the RAM page buffer */
    uint32_t tail_idx = index - committed_records;
    const log_rec4_t *pg = (const log_rec4_t *)g_page_buf;

    for (uint32_t i=0; i<full_pages; i++) {
        const log_rec4_t *pgc = (const log_rec4_t *)(xip_data_active() + i * FLASH_PAGE_SIZE);
        for (uint32_t j=0; j<RECORDS_PER_PAGE; j++) t += (uint32_t)pgc[j].dtime_10ms * 10u;
    }
    for (uint32_t j=0; j<=tail_idx; j++) t += (uint32_t)pg[j].dtime_10ms * 10u;

    if (out_ms)   *out_ms   = t;
    if (out_temp) *out_temp = (float)pg[tail_idx].temp_c_centi / 100.0f;
    return 0;
}

/* ====== BACKUP helpers & API ====== */

static void backup_copy_active_to_backup(void) {
    uint32_t used_bytes = g_write_pos;
    uint32_t used_pages = (used_bytes + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

    flash_erase_sectors(BACKUP_FLASH_OFFSET, LOG_REGION_SIZE);

    /* copy committed pages */
    const uint8_t *active_data = xip_data_active();
    for (uint32_t p = 0; p < (used_pages ? (used_pages - 1) : 0); ++p) {
        const uint8_t *src_page = active_data + p * FLASH_PAGE_SIZE;
        uint32_t dst_off = BACKUP_FLASH_OFFSET + FLASH_PAGE_SIZE + p * FLASH_PAGE_SIZE;
        flash_program_block(dst_off, src_page, FLASH_PAGE_SIZE);
    }

    /* copy last page from RAM tail or from ACTIVE if aligned */
    if (used_pages > 0) {
        uint32_t last_page_index = used_pages - 1;
        uint32_t dst_off = BACKUP_FLASH_OFFSET + FLASH_PAGE_SIZE + last_page_index * FLASH_PAGE_SIZE;

        if ((g_page_fill == 0) && (used_bytes != 0)) {
            const uint8_t *src_page = active_data + last_page_index * FLASH_PAGE_SIZE;
            flash_program_block(dst_off, src_page, FLASH_PAGE_SIZE);
        } else if (g_page_fill > 0) {
            uint8_t page[FLASH_PAGE_SIZE];
            memset(page, 0xFF, sizeof page);
            memcpy(page, g_page_buf, g_page_fill);
            flash_program_block(dst_off, page, FLASH_PAGE_SIZE);
        }
    }

    /* write the BACKUP header last */
    log_header_t bh = g_hdr;  /* exact count, base_time, etc. */
    write_header_to_flash_backup(&bh);
}

void bmp388_storage_rotate_to_backup(void) {
    backup_copy_active_to_backup();
    /* keep ACTIVE as-is (optional), or clear for a fresh run.
       We’ll clear to ensure “freeze” semantics: BACKUP = snapshot, ACTIVE restarts */
   // bmp388_storage_erase_all();
}

void bmp388_backup_clear(void) {
    flash_erase_sectors(BACKUP_FLASH_OFFSET, LOG_REGION_SIZE);
    log_header_t h = {0};
    h.magic = LOG_MAGIC; h.version = LOG_VERSION;
    h.capacity_bytes = LOG_REGION_SIZE - FLASH_PAGE_SIZE;
    h.count = 0; h.base_time_ms = 0;
    write_header_to_flash_backup(&h);
}

uint32_t bmp388_backup_count(void) {
    const log_header_t *bh = xip_hdr_backup();
    if (bh->magic != LOG_MAGIC || bh->version != LOG_VERSION) return 0;
    return bh->count;
}

int bmp388_backup_read(uint32_t index, uint32_t *out_ms, float *out_temp) {
    const log_header_t *bh = xip_hdr_backup();
    if (bh->magic != LOG_MAGIC || bh->version != LOG_VERSION) return -1;
    if (index >= bh->count) return -2;

    uint32_t t = bh->base_time_ms;

    uint32_t full_pages = bh->count / RECORDS_PER_PAGE;
    uint32_t committed_records = full_pages * RECORDS_PER_PAGE;

    if (index < committed_records) {
        uint32_t page_index = index / RECORDS_PER_PAGE;
        uint32_t in_page    = index % RECORDS_PER_PAGE;

        for (uint32_t i=0; i<page_index; i++) {
            const log_rec4_t *pg = (const log_rec4_t *)(xip_data_backup() + i * FLASH_PAGE_SIZE);
            for (uint32_t j=0; j<RECORDS_PER_PAGE; j++) t += (uint32_t)pg[j].dtime_10ms * 10u;
        }
        const log_rec4_t *pg = (const log_rec4_t *)(xip_data_backup() + page_index * FLASH_PAGE_SIZE);
        for (uint32_t j=0; j<=in_page; j++)           t += (uint32_t)pg[j].dtime_10ms * 10u;

        if (out_ms)   *out_ms   = t;
        if (out_temp) *out_temp = (float)pg[in_page].temp_c_centi / 100.0f;
        return 0;
    }

    /* within last (partial) page */
    uint32_t tail_idx = index - committed_records;
    const log_rec4_t *pg_last = (const log_rec4_t *)(xip_data_backup() + full_pages * FLASH_PAGE_SIZE);

    for (uint32_t i=0; i<full_pages; i++) {
        const log_rec4_t *pg = (const log_rec4_t *)(xip_data_backup() + i * FLASH_PAGE_SIZE);
        for (uint32_t j=0; j<RECORDS_PER_PAGE; j++) t += (uint32_t)pg[j].dtime_10ms * 10u;
    }
    for (uint32_t j=0; j<=tail_idx; j++) t += (uint32_t)pg_last[j].dtime_10ms * 10u;

    if (out_ms)   *out_ms   = t;
    if (out_temp) *out_temp = (float)pg_last[tail_idx].temp_c_centi / 100.0f;
    return 0;
}

/* ===== Compact dumps used by bmp388.c ===== */

/* Human-readable compact dump (5 buckets) with timestamp every 50 samples.
   Emits 'e' token when excursion state toggles. (ACTIVE) */
static int bucket_5_text(float c) {
    if (c < 22.0f) return 0;
    if (c < 24.0f) return 1;
    if (c < 26.0f) return 2;
    if (c < 28.0f) return 3;
    return 4;
}

void dump_active_compact(void) {
    uint32_t n = bmp388_storage_count();
    if (n == 0) { printf("# no ACTIVE records\n"); return; }

    bool have_prev      = false;
    bool prev_exc       = false;
    bool first_in_block = true;

    for (uint32_t i = 0; i < n; i++) {
        uint32_t t_ms; float temp;
        if (bmp388_storage_read(i, &t_ms, &temp) != 0) continue;

        if ((i % 50) == 0) {
            printf("\n*timestamp* %lu\n", (unsigned long)t_ms);
            first_in_block = true;
        }

        bool exc = is_excursion(temp);
        int  b   = bucket_5_text(temp);

        if (!have_prev) {
            if (!first_in_block) printf(",");
            printf("%d", b);
            first_in_block = false;
            have_prev = true;
            prev_exc  = exc;
            continue;
        }

        if (exc != prev_exc) {
            if (!first_in_block) printf(",");
            printf("e");
            first_in_block = false;
            prev_exc = exc;
        }

        if (!first_in_block) printf(",");
        printf("%d", b);
        first_in_block = false;
    }
    printf("\n");
}

/* Identical compact dump, but reading from BACKUP (non-volatile snapshot). */
void dump_backup(void) {
    const log_header_t *bh = xip_hdr_backup();
    if (bh->magic != LOG_MAGIC || bh->version != LOG_VERSION || bh->count == 0) {
        printf("# no BACKUP records\n");
        return;
    }

    bool have_prev      = false;
    bool prev_exc       = false;
    bool first_in_block = true;

    for (uint32_t i = 0; i < bh->count; i++) {
        uint32_t t_ms; float temp;
        if (bmp388_backup_read(i, &t_ms, &temp) != 0) continue;

        if ((i % 50) == 0) {
            printf("\n*timestamp* %lu\n", (unsigned long)t_ms);
            first_in_block = true;
        }

        bool exc = is_excursion(temp);
        int  b   = bucket_5_text(temp);

        if (!have_prev) {
            if (!first_in_block) printf(",");
            printf("%d", b);
            first_in_block = false;
            have_prev = true;
            prev_exc  = exc;
            continue;
        }

        if (exc != prev_exc) {
            if (!first_in_block) printf(",");
            printf("e");
            first_in_block = false;
            prev_exc = exc;
        }

        if (!first_in_block) printf(",");
        printf("%d", b);
        first_in_block = false;
    }
    printf("\n");
}

/* Binary compact dump for ACTIVE (unchanged) */
static uint8_t temp_to_bucket5(float temp) {
    if (temp < 22.0f) return 0;
    if (temp < 24.0f) return 1;
    if (temp < 26.0f) return 2;
    if (temp < 28.0f) return 3;
    return 4;
}

void dump_active_compact_bit(void) {
    uint32_t n = bmp388_storage_count();

    /* header */
    putchar_raw(0xC1); putchar_raw(0x01);
    putchar_raw((uint8_t)(n & 0xFF));
    putchar_raw((uint8_t)(n >> 8));
    putchar_raw(0x05);

    uint8_t cur = 0; int bits = 0;
    bool have_prev = false, prev_exc = false;

    for (uint32_t i = 0; i < n; i++) {
        uint32_t t_ms; float temp;
        if (bmp388_storage_read(i, &t_ms, &temp) != 0) continue;

        if ((i % 50) == 0) {
            if (bits > 0) { putchar_raw(cur); cur = 0; bits = 0; }
            putchar_raw(OPC_TS);
            putchar_raw((uint8_t)(t_ms));
            putchar_raw((uint8_t)(t_ms >> 8));
            putchar_raw((uint8_t)(t_ms >> 16));
            putchar_raw((uint8_t)(t_ms >> 24));
            if (have_prev && prev_exc) putchar_raw(OPC_E);
        }

        bool exc = is_excursion(temp);
        if (!have_prev) { have_prev = true; prev_exc = exc; }
        else if (exc != prev_exc) { if (bits > 0) { putchar_raw(cur); cur = 0; bits = 0; } putchar_raw(OPC_E); prev_exc = exc; }

        uint8_t b = temp_to_bucket5(temp) & 0x07;
        cur |= (uint8_t)(b << bits);
        bits += 3;
        if (bits >= 8) {
            putchar_raw(cur);
            bits -= 8;
            cur = (bits > 0) ? (uint8_t)(b >> (3 - bits)) : 0;
        }
    }

    if (bits > 0) putchar_raw(cur);
    putchar_raw(OPC_END);
}
