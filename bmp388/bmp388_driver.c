#include "bmp388_driver.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <string.h>

/* ===== BMP388 register subset (unchanged sensor driver) ===== */
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

static i2c_inst_t *g_i2c = i2c0;
static uint8_t g_addr = 0x77;
static bool g_ready = false;

/* Calibration for temp (unchanged) */
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
    return 0;
}
static float compensate_temperature(uint32_t t_raw)
{
    float p1 = ((float)t_raw / 16384.0f) - (g_tcal.par_t1 / 1024.0f);
    g_tcal.t_lin = p1 * g_tcal.par_t2 + (p1 * p1) * g_tcal.par_t3;
    return g_tcal.t_lin;
}
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
    i2c_reg_write_u8(BMP388_REG_ODR, 0x09);
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
    for (int i=0;i<200;i++) {
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
    return 0;
}

/* ============================================================
 *        Ultra-compact storage + Dual-Region BACKUP
 * ============================================================ */

#ifndef LOG_REGION_SIZE
#define LOG_REGION_SIZE   (128 * 1024)    /* ACTIVE region size */
#endif

#define FLASH_SECTOR_SIZE 4096
#define FLASH_PAGE_SIZE   256
#define LOG_MAGIC         0x424D5039u     /* 'BMP9' - compact v2 */
#define LOG_VERSION       2u

/* 4-byte compact record */
typedef struct __attribute__((packed)) {
    uint16_t dtime_10ms;     /* delta time in 10ms units (0..65535 -> 0..655.35 s) */
    int16_t  temp_c_centi;   /* temperature * 100 (signed) */
} log_rec4_t;
#define RECORD_SIZE       ((uint32_t)sizeof(log_rec4_t))
#define RECORDS_PER_PAGE  (FLASH_PAGE_SIZE / RECORD_SIZE)

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

/* Layout:
 * [ ... firmware ... ][ BACKUP 128KB ][ ACTIVE 128KB ]
 * ACTIVE sits at the very end. BACKUP is just before it.
 */
#define LOG_FLASH_OFFSET     (PICO_FLASH_SIZE_BYTES - LOG_REGION_SIZE)          /* ACTIVE region base */
#define BACKUP_FLASH_OFFSET  (LOG_FLASH_OFFSET - LOG_REGION_SIZE)               /* BACKUP region base */

#define LOG_XIP_BASE         (XIP_BASE + LOG_FLASH_OFFSET)
#define BACKUP_XIP_BASE      (XIP_BASE + BACKUP_FLASH_OFFSET)

/* Header is the first 256B page of each region */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t capacity_bytes;   /* bytes available for data (excl. header page) */
    uint32_t count;            /* #records (not necessarily page-flushed) */
    uint32_t base_time_ms;     /* absolute ms for record #0 */
    uint32_t reserved32[7];    /* pad to 64B; remainder of the 256B page is 0xFF */
} log_header_t;

/* ACTIVE runtime state */
static log_header_t g_hdr;
static uint8_t  g_page_buf[FLASH_PAGE_SIZE];
static uint32_t g_page_fill = 0;      /* bytes in RAM page (0..256) */
static uint32_t g_write_pos = 0;      /* bytes written to data area (excl. header page) */
static uint32_t g_last_time_ms = 0;
static bool     g_have_anchor = false;

/* XIP helpers */
static inline const log_header_t *xip_hdr_active(void)  { return (const log_header_t *)(LOG_XIP_BASE); }
static inline const uint8_t      *xip_data_active(void) { return (const uint8_t *)(LOG_XIP_BASE + FLASH_PAGE_SIZE); }
static inline const log_header_t *xip_hdr_backup(void)  { return (const log_header_t *)(BACKUP_XIP_BASE); }
static inline const uint8_t      *xip_data_backup(void) { return (const uint8_t *)(BACKUP_XIP_BASE + FLASH_PAGE_SIZE); }

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

/* ACTIVE region: reset/erase/init */
static void storage_reset_state_after_erase(void) {
    memset(&g_hdr, 0, sizeof g_hdr);
    g_hdr.magic          = LOG_MAGIC;
    g_hdr.version        = LOG_VERSION;
    g_hdr.capacity_bytes = LOG_REGION_SIZE - FLASH_PAGE_SIZE; /* first page = header */
    g_hdr.count          = 0;
    g_hdr.base_time_ms   = 0;

    g_write_pos = 0;
    g_page_fill = 0;
    g_last_time_ms = 0;
    g_have_anchor = false;

    memset(g_page_buf, 0xFF, sizeof g_page_buf);
    write_header_to_flash_active(&g_hdr);
}

void bmp388_storage_erase_all(void) {
    flash_erase_sectors(LOG_FLASH_OFFSET, LOG_REGION_SIZE);
    storage_reset_state_after_erase();
}

void bmp388_storage_init(bool erase_all) {
    /* Sanity: ensure backup region exists in flash map */
    if (BACKUP_FLASH_OFFSET < XIP_BASE) {
        /* If your firmware becomes huge or flash is small, reduce LOG_REGION_SIZE. */
        // We won't hard-fail; we just skip backup usage. (Rare in 2MB parts.)
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

    /* Rebuild last_time_ms cheaply by walking only committed pages + tail */
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

/* ====== BACKUP helpers ====== */

/* BACKUP (only the bytes actually used), then write header LAST. */
static void backup_copy_active_to_backup(void) {
    /* Compute how many data bytes are currently used (including RAM tail). */
    uint32_t used_bytes = g_write_pos;                  // bytes in data area (excl. header page)
    uint32_t used_pages = (used_bytes + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

    /* Erase the entire BACKUP region first. */
    flash_erase_sectors(BACKUP_FLASH_OFFSET, LOG_REGION_SIZE);

    /* 1) Copy all fully committed data pages already in ACTIVE flash. */
    const uint8_t *active_data = xip_data_active();
    for (uint32_t p = 0; p < (used_pages ? (used_pages - 1) : 0); ++p) {
        const uint8_t *src_page = active_data + p * FLASH_PAGE_SIZE;
        uint32_t dst_off = BACKUP_FLASH_OFFSET + FLASH_PAGE_SIZE + p * FLASH_PAGE_SIZE;
        flash_program_block(dst_off, src_page, FLASH_PAGE_SIZE);
    }

    /* 2) Handle the last page: either from flash (if page-aligned) or RAM tail. */
    if (used_pages > 0) {
        uint32_t last_page_index = used_pages - 1;
        uint32_t dst_off = BACKUP_FLASH_OFFSET + FLASH_PAGE_SIZE + last_page_index * FLASH_PAGE_SIZE;

        if ((g_page_fill == 0) && (used_bytes != 0)) {
            /* We’re page-aligned and nothing in RAM: copy the last page directly from ACTIVE. */
            const uint8_t *src_page = active_data + last_page_index * FLASH_PAGE_SIZE;
            flash_program_block(dst_off, src_page, FLASH_PAGE_SIZE);
        } else if (g_page_fill > 0) {
            /* We have a RAM tail. Build a page with tail bytes then 0xFF. */
            uint8_t page[FLASH_PAGE_SIZE];
            memset(page, 0xFF, sizeof page);
            memcpy(page, g_page_buf, g_page_fill);
            flash_program_block(dst_off, page, FLASH_PAGE_SIZE);
        }
    }

    /* 3) Write the BACKUP header LAST with accurate fields. */
    log_header_t bh = g_hdr;                 // start from current RAM header
    bh.count        = g_hdr.count;           // ensure exact count (includes RAM tail)
    /* capacity_bytes, base_time_ms, version, magic already set in g_hdr */
    write_header_to_flash_backup(&bh);
}


/* Public: manual rotate (ACTIVE -> BACKUP), then erase ACTIVE and reset. */
void bmp388_storage_rotate_to_backup(void) {
    backup_copy_active_to_backup();
    bmp388_storage_erase_all();
}

/* Public: clear BACKUP region */
void bmp388_backup_clear(void) {
    flash_erase_sectors(BACKUP_FLASH_OFFSET, LOG_REGION_SIZE);
    /* Write an empty, valid header (so we can tell it's empty backup) */
    log_header_t h = {0};
    h.magic = LOG_MAGIC; h.version = LOG_VERSION;
    h.capacity_bytes = LOG_REGION_SIZE - FLASH_PAGE_SIZE;
    h.count = 0; h.base_time_ms = 0;
    write_header_to_flash_backup(&h);
}

/* BACKUP readers */
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

    /* Walk full committed pages from XIP */
    uint32_t full_pages = bh->count / RECORDS_PER_PAGE;
    uint32_t committed_records = full_pages * RECORDS_PER_PAGE;

    /* If target is within committed part */
    if (index < committed_records) {
        uint32_t page_index = index / RECORDS_PER_PAGE;
        uint32_t in_page    = index % RECORDS_PER_PAGE;

        /* Sum deltas up to the target (O(n)); for bulk reads, iterate sequentially and cache t externally */
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
        /* There is no RAM tail for BACKUP (it’s fully materialized), so if index >= committed_records,
           it simply means last page is partially filled on backup; still in XIP. */
        uint32_t tail_idx = index - committed_records;
        const log_rec4_t *pg_last = (const log_rec4_t *)(xip_data_backup() + full_pages * FLASH_PAGE_SIZE);

        /* Sum all full pages */
        for (uint32_t i=0;i<full_pages;i++) {
            const log_rec4_t *pg = (const log_rec4_t *)(xip_data_backup() + i * FLASH_PAGE_SIZE);
            for (uint32_t j=0;j<RECORDS_PER_PAGE;j++) t += (uint32_t)pg[j].dtime_10ms * 10u;
        }
        /* Walk within last (partial) page */
        for (uint32_t j=0;j<=tail_idx;j++)          t += (uint32_t)pg_last[j].dtime_10ms * 10u;

        if (out_ms)   *out_ms   = t;
        if (out_temp) *out_temp = (float)pg_last[tail_idx].temp_c_centi / 100.0f;
        return 0;
    }
}

/* ===== ACTIVE appends (with auto-rotation on full) ===== */

int bmp388_storage_append(uint32_t time_ms, float temperature_c) {
    /* First record becomes anchor (absolute time) */
    if (!g_have_anchor) {
        g_hdr.base_time_ms = time_ms;
        g_last_time_ms     = time_ms;
        g_have_anchor      = true;
        write_header_to_flash_active(&g_hdr);
    }

    /* If ACTIVE is full, rotate to BACKUP first */
    if (g_write_pos + RECORD_SIZE > g_hdr.capacity_bytes) {
        // Copy ACTIVE -> BACKUP (including RAM tail), then erase ACTIVE
        backup_copy_active_to_backup();
        bmp388_storage_erase_all();
        return -1; /* signal "was full" to caller (optional to check) */
    }

    /* Build compact record */
    uint32_t delta_ms = (time_ms >= g_last_time_ms) ? (time_ms - g_last_time_ms) : 0;
    uint32_t delta_10 = delta_ms / 10u;  if (delta_10 > 0xFFFFu) delta_10 = 0xFFFFu;
    int32_t  t_centi  = (int32_t)(temperature_c * 100.0f);
    if (t_centi >  32767) t_centi =  32767;
    if (t_centi < -32768) t_centi = -32768;

    log_rec4_t rec;
    rec.dtime_10ms   = (uint16_t)delta_10;
    rec.temp_c_centi = (int16_t)t_centi;

    uint32_t in_page_off = g_page_fill;
    memcpy(&g_page_buf[in_page_off], &rec, sizeof rec);
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

    /* If target within committed part (XIP flash) */
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

    /* Otherwise, it’s in the RAM tail page */
    uint32_t tail_idx = index - committed_records;
    const log_rec4_t *pg = (const log_rec4_t *)g_page_buf;

    /* Sum committed part */
    for (uint32_t i=0;i<full_pages;i++) {
        const log_rec4_t *pgc = (const log_rec4_t *)(xip_data_active() + i * FLASH_PAGE_SIZE);
        for (uint32_t j=0;j<RECORDS_PER_PAGE;j++) t += (uint32_t)pgc[j].dtime_10ms * 10u;
    }
    /* Walk within RAM page */
    for (uint32_t j=0;j<=tail_idx;j++) t += (uint32_t)pg[j].dtime_10ms * 10u;

    if (out_ms)   *out_ms   = t;
    if (out_temp) *out_temp = (float)pg[tail_idx].temp_c_centi / 100.0f;
    return 0;
}
