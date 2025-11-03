// bmp388_storage.c
#include "bmp388_driver.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* ====== Logging layout/config ====== */
#ifndef LOG_REGION_SIZE
#define LOG_REGION_SIZE   (128 * 1024)    /* ACTIVE region size */
#endif

#define FLASH_SECTOR_SIZE 4096
#define FLASH_PAGE_SIZE   256
#define LOG_MAGIC         0x424D5039u     /* 'BMP9' - compact v2 */
#define LOG_VERSION       2u

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

/* Layout:
 * [ ... firmware ... ][ BACKUP LOG_REGION_SIZE ][ ACTIVE LOG_REGION_SIZE ]
 */
#define LOG_FLASH_OFFSET     (PICO_FLASH_SIZE_BYTES - LOG_REGION_SIZE)          /* ACTIVE base */
#define BACKUP_FLASH_OFFSET  (LOG_FLASH_OFFSET - LOG_REGION_SIZE)               /* BACKUP base */

#define LOG_XIP_BASE         (XIP_BASE + LOG_FLASH_OFFSET)
#define BACKUP_XIP_BASE      (XIP_BASE + BACKUP_FLASH_OFFSET)

/* 4-byte compact record: dtime(10ms) + temp(centi-degC) */
typedef struct __attribute__((packed)) {
    uint16_t dtime_10ms;
    int16_t  temp_c_centi;
} log_rec4_t;
#define RECORD_SIZE       ((uint32_t)sizeof(log_rec4_t))
#define RECORDS_PER_PAGE  (FLASH_PAGE_SIZE / RECORD_SIZE)

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t capacity_bytes;   /* bytes available for data (excl. header page) */
    uint32_t count;            /* #records (not necessarily page-flushed) */
    uint32_t base_time_ms;     /* absolute ms for record #0 */
    uint32_t reserved32[7];    /* pad to 64B; remainder of 256B page is 0xFF */
} log_header_t;

/* ===== Runtime state (ACTIVE) ===== */
static log_header_t g_hdr;
static uint8_t  g_page_buf[FLASH_PAGE_SIZE];
static uint32_t g_page_fill = 0;     /* bytes in RAM page (0..256) */
static uint32_t g_write_pos = 0;     /* bytes written to data area (excl. header page) */
static uint32_t g_last_time_ms = 0;
static bool     g_have_anchor = false;

/* XIP helpers */
static inline const log_header_t *xip_hdr_active(void)  { return (const log_header_t *)(LOG_XIP_BASE); }
static inline const uint8_t      *xip_data_active(void) { return (const uint8_t *)(LOG_XIP_BASE + FLASH_PAGE_SIZE); }
static inline const log_header_t *xip_hdr_backup(void)  { return (const log_header_t *)(BACKUP_XIP_BASE); }
static inline const uint8_t      *xip_data_backup(void) { return (const uint8_t *)(BACKUP_XIP_BASE + FLASH_PAGE_SIZE); }

void dump_active(void) {
    uint32_t n = bmp388_storage_count();
    printf("\n# ACTIVE index,time_ms,temp_C\n");
    for (uint32_t i=0;i<n;i++) {
        uint32_t t; float temp;
        if (bmp388_storage_read(i, &t, &temp) == 0)
            printf("%lu,%lu,%.2f\n", (unsigned long)i, (unsigned long)t, temp);
    }
    printf("# ACTIVE total %lu\n\n", (unsigned long)n);
}
void dump_backup(void) {
    uint32_t n = bmp388_backup_count();
    printf("\n# BACKUP index,time_ms,temp_C\n");
    for (uint32_t i=0;i<n;i++) {
        uint32_t t; float temp;
        if (bmp388_backup_read(i, &t, &temp) == 0)
            printf("%lu,%lu,%.2f\n", (unsigned long)i, (unsigned long)t, temp);
    }
    printf("# BACKUP total %lu\n\n", (unsigned long)n);
}

/* Flash helpers */
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
static void write_header_to_flash_backup(const log_header_t *h) {
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    memcpy(page, h, sizeof *h);
    flash_program_block(BACKUP_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
}

/* ===== ACTIVE region: reset/erase/init ===== */
static void storage_reset_state_after_erase(void) {
    memset(&g_hdr, 0, sizeof g_hdr);
    g_hdr.magic          = LOG_MAGIC;
    g_hdr.version        = LOG_VERSION;
    g_hdr.capacity_bytes = LOG_REGION_SIZE - FLASH_PAGE_SIZE; /* header occupies first page */
    g_hdr.count          = 0;
    g_hdr.base_time_ms   = 0;

    g_write_pos = 0;
    g_page_fill = 0;
    g_last_time_ms = 0;
    g_have_anchor = false;

    memset(g_page_buf, 0xFF, sizeof g_page_buf);
    write_header_to_flash_active(&g_hdr);
}

/* ==== Public storage API ==== */
void bmp388_storage_erase_all(void) {
    flash_erase_sectors(LOG_FLASH_OFFSET, LOG_REGION_SIZE);
    storage_reset_state_after_erase();
}

void bmp388_storage_init(bool erase_all) {
    if (BACKUP_FLASH_OFFSET < XIP_BASE) {
        /* If firmware grows too large, reduce LOG_REGION_SIZE. */
    }

    const log_header_t *h = xip_hdr_active();
    bool ok = (h->magic == LOG_MAGIC) && (h->version == LOG_VERSION);
    if (erase_all || !ok) { bmp388_storage_erase_all(); return; }

    g_hdr = *h;
    g_write_pos = g_hdr.count * RECORD_SIZE;
    g_page_fill = g_write_pos % FLASH_PAGE_SIZE;
    memset(g_page_buf, 0xFF, sizeof g_page_buf);

    g_have_anchor  = (g_hdr.count > 0) || (g_hdr.base_time_ms != 0);
    g_last_time_ms = g_hdr.base_time_ms;

    /* Rebuild last_time_ms by walking committed pages + RAM tail */
    if (g_hdr.count > 0) {
        uint32_t t = g_hdr.base_time_ms;
        uint32_t full_pages = g_hdr.count / RECORDS_PER_PAGE;
        uint32_t rem        = g_hdr.count % RECORDS_PER_PAGE;

        for (uint32_t p = 0; p < full_pages; ++p) {
            const log_rec4_t *pg = (const log_rec4_t *)(xip_data_active() + p * FLASH_PAGE_SIZE);
            for (uint32_t i=0;i<RECORDS_PER_PAGE;i++) t += (uint32_t)pg[i].dtime_10ms * 10u;
        }
        if (rem) {
            const log_rec4_t *pg = (const log_rec4_t *)(xip_data_active() + full_pages * FLASH_PAGE_SIZE);
            for (uint32_t i=0;i<rem;i++) t += (uint32_t)pg[i].dtime_10ms * 10u;
        }
        g_last_time_ms = t;
    }
}

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

/* ===== BACKUP helpers ===== */
static void backup_copy_active_to_backup(void) {
    uint32_t used_bytes = g_write_pos;                  // data bytes (excl. header page)
    uint32_t used_pages = (used_bytes + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

    flash_erase_sectors(BACKUP_FLASH_OFFSET, LOG_REGION_SIZE);

    /* Copy committed pages */
    const uint8_t *active_data = xip_data_active();
    for (uint32_t p = 0; p < (used_pages ? (used_pages - 1) : 0); ++p) {
        const uint8_t *src_page = active_data + p * FLASH_PAGE_SIZE;
        uint32_t dst_off = BACKUP_FLASH_OFFSET + FLASH_PAGE_SIZE + p * FLASH_PAGE_SIZE;
        flash_program_block(dst_off, src_page, FLASH_PAGE_SIZE);
    }

    /* Last page = page-aligned or RAM tail */
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

    /* Write BACKUP header last */
    log_header_t bh = g_hdr;
    write_header_to_flash_backup(&bh);
}

/* ==== Public BACKUP ops ==== */
void bmp388_storage_rotate_to_backup(void) {
    backup_copy_active_to_backup();
    bmp388_storage_erase_all();
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

        for (uint32_t i=0;i<page_index;i++) {
            const log_rec4_t *pg = (const log_rec4_t *)(xip_data_backup() + i * FLASH_PAGE_SIZE);
            for (uint32_t j=0;j<RECORDS_PER_PAGE;j++) t += (uint32_t)pg[j].dtime_10ms * 10u;
        }
        const log_rec4_t *pg = (const log_rec4_t *)(xip_data_backup() + page_index * FLASH_PAGE_SIZE);
        for (uint32_t j=0;j<=in_page;j++)           t += (uint32_t)pg[j].dtime_10ms * 10u;

        if (out_ms)   *out_ms   = t;
        if (out_temp) *out_temp = (float)pg[in_page].temp_c_centi / 100.0f;
        return 0;
    } else {
        uint32_t tail_idx = index - committed_records;
        const log_rec4_t *pg_last = (const log_rec4_t *)(xip_data_backup() + full_pages * FLASH_PAGE_SIZE);

        for (uint32_t i=0;i<full_pages;i++) {
            const log_rec4_t *pg = (const log_rec4_t *)(xip_data_backup() + i * FLASH_PAGE_SIZE);
            for (uint32_t j=0;j<RECORDS_PER_PAGE;j++) t += (uint32_t)pg[j].dtime_10ms * 10u;
        }
        for (uint32_t j=0;j<=tail_idx;j++)          t += (uint32_t)pg_last[j].dtime_10ms * 10u;

        if (out_ms)   *out_ms   = t;
        if (out_temp) *out_temp = (float)pg_last[tail_idx].temp_c_centi / 100.0f;
        return 0;
    }
}

/* ==== ACTIVE append/read ==== */
int bmp388_storage_append(uint32_t time_ms, float temperature_c) {
    if (!g_have_anchor) {
        g_hdr.base_time_ms = time_ms;
        g_last_time_ms     = time_ms;
        g_have_anchor      = true;
        write_header_to_flash_active(&g_hdr);
    }

    if (g_write_pos + RECORD_SIZE > g_hdr.capacity_bytes) {
        backup_copy_active_to_backup();
        bmp388_storage_erase_all();
        return -1; /* rotated because full */
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

        for (uint32_t i=0;i<page_index;i++) {
            const log_rec4_t *pg = (const log_rec4_t *)(xip_data_active() + i * FLASH_PAGE_SIZE);
            for (uint32_t j=0;j<RECORDS_PER_PAGE;j++) t += (uint32_t)pg[j].dtime_10ms * 10u;
        }
        const log_rec4_t *pg = (const log_rec4_t *)(xip_data_active() + page_index * FLASH_PAGE_SIZE);
        for (uint32_t j=0;j<=in_page;j++)            t += (uint32_t)pg[j].dtime_10ms * 10u;

        if (out_ms)   *out_ms   = t;
        if (out_temp) *out_temp = (float)pg[in_page].temp_c_centi / 100.0f;
        return 0;
    }

    uint32_t tail_idx = index - committed_records;
    const log_rec4_t *pg = (const log_rec4_t *)g_page_buf;

    for (uint32_t i=0;i<full_pages;i++) {
        const log_rec4_t *pgc = (const log_rec4_t *)(xip_data_active() + i * FLASH_PAGE_SIZE);
        for (uint32_t j=0;j<RECORDS_PER_PAGE;j++) t += (uint32_t)pgc[j].dtime_10ms * 10u;
    }
    for (uint32_t j=0;j<=tail_idx;j++) t += (uint32_t)pg[j].dtime_10ms * 10u;

    if (out_ms)   *out_ms   = t;
    if (out_temp) *out_temp = (float)pg[tail_idx].temp_c_centi / 100.0f;
    return 0;
}

/* ===== Compact dumps (unchanged API) ===== */

/* Text dump of ACTIVE in 5-bucket ranges with timestamp every 50 samples */
static int bucket_5(float c) {
    if (c < 22.0f) return 0;
    if (c < 24.0f) return 1;
    if (c < 26.0f) return 2;
    if (c < 28.0f) return 3;
    return 4;
}
void dump_active_compact(void) {
    uint32_t n = bmp388_storage_count();
    if (n == 0) { printf("# no ACTIVE records\n"); return; }

    for (uint32_t i = 0; i < n; i++) {
        uint32_t t_ms; float temp;
        if (bmp388_storage_read(i, &t_ms, &temp) != 0) continue;

        if ((i % 50) == 0) printf("\n*timestamp* %lu\n", (unsigned long)t_ms);
        int b = bucket_5(temp);
        printf("%d", b);

        bool is_last = (i == (n - 1));
        bool end_of_block = ((i % 50) == 49);
        if (!is_last && !end_of_block) printf(",");
    }
    printf("\n");
}

/* Binary, masked, timestamp-every-50 compact dump (3-bit buckets) */
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
    putchar_raw(0xC1);
    putchar_raw(0x01);
    putchar_raw((uint8_t)(n & 0xFF));
    putchar_raw((uint8_t)(n >> 8));
    putchar_raw(0x05);

    uint8_t cur = 0;
    int bits = 0;

    for (uint32_t i = 0; i < n; i++) {
        uint32_t t_ms; float temp;
        if (bmp388_storage_read(i, &t_ms, &temp) != 0) continue;

        if ((i % 50) == 0) {
            if (bits > 0) { putchar_raw(cur); cur = 0; bits = 0; }
            putchar_raw(0xF0);
            putchar_raw((uint8_t)(t_ms & 0xFF));
            putchar_raw((uint8_t)((t_ms >> 8) & 0xFF));
            putchar_raw((uint8_t)((t_ms >> 16) & 0xFF));
            putchar_raw((uint8_t)((t_ms >> 24) & 0xFF));
        }

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
    putchar_raw(0xEE);
}
