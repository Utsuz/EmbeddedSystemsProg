// storage_driver.c — ACTIVE logging + shared compact-bit stream (UART/Flash)

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
#define LOG_REGION_SIZE      (256 * 1024)  /* ACTIVE region size (end of flash) */
#endif

#ifndef RAMLOG_CAP
#define RAMLOG_CAP           2048          /* mirror buffer for robust dumps */
#endif

/* ===== Record format (on flash) =====
 * Fixed 4-byte records: [dtime_10ms:u16][temp_c_centi:i16]
 */
typedef struct __attribute__((packed)) {
    uint16_t dtime_10ms;
    int16_t  temp_c_centi;
} log_rec4_t;

/* ===== ACTIVE region header ===== */
typedef struct __attribute__((packed)) {
    uint32_t magic;      /* 'LOG1' */
    uint32_t count;      /* number of valid records */
} log_header_t;

#define LOG_MAGIC 0x31474F4Cu /* 'LOG1' */

/* Flash/XIP layout */
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#define LOG_FLASH_OFFSET      (PICO_FLASH_SIZE_BYTES - LOG_REGION_SIZE)
#define LOG_XIP_BASE          (XIP_BASE + LOG_FLASH_OFFSET)

/* ===== XIP helpers ===== */
static inline const log_header_t *xip_hdr_active(void)   { return (const log_header_t *)(LOG_XIP_BASE); }
static inline const uint8_t      *xip_data_active(void)  { return (const uint8_t *)(LOG_XIP_BASE + FLASH_PAGE_SIZE); }

/* ===== Compact-bit blob region (for saving dump_active_compact_bit output) ===== */
#ifndef COMPACT_REGION_SIZE
#define COMPACT_REGION_SIZE   (32 * 1024)  /* adjust if you need more/less */
#endif
#define COMPACT_FLASH_OFFSET  (PICO_FLASH_SIZE_BYTES - LOG_REGION_SIZE - COMPACT_REGION_SIZE)
#define COMPACT_XIP_BASE      (XIP_BASE + COMPACT_FLASH_OFFSET)
#define CB_MAGIC  0x43424954u /* 'CBIT' */
typedef struct __attribute__((packed)) {
    uint32_t magic;   /* CB_MAGIC */
    uint32_t length;  /* number of valid bytes in 'data' */
    uint8_t  reserved[FLASH_PAGE_SIZE - 8]; /* pad to 1 page */
} compact_hdr_t;
static inline const compact_hdr_t *xip_hdr_compact(void) {
    return (const compact_hdr_t *)(COMPACT_XIP_BASE);
}
static inline const uint8_t *xip_data_compact(void) {
    return (const uint8_t *)(COMPACT_XIP_BASE + FLASH_PAGE_SIZE);
}

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

/* ===== ACTIVE in-RAM mirror for robust dumping ===== */
typedef struct {
    uint32_t t_ms[RAMLOG_CAP];
    float    temp[RAMLOG_CAP];
    uint32_t head;    /* oldest index */
    uint32_t count;   /* number of valid items (<= RAMLOG_CAP) */
} ramlog_t;

static struct {
    log_header_t hdr;         /* cached header */
    uint32_t     write_pos;   /* byte offset after header where next record goes */
    uint8_t      page_buf[FLASH_PAGE_SIZE];
    uint32_t     page_fill;
    uint32_t     last_time_ms;
} g;

static ramlog_t g_ram = { .head = 0, .count = 0 };

/* ===== Bucket & compact opcodes ===== */
#define OPC_TS   0xF0
#define OPC_E    0xE5
#define OPC_END  0xEE

/* ===== Bucket mapper: 0.5 °C buckets centered at 22.5..28.0.. =====
 * 0..12 covering [22.25, 28.25], clamp outside.
 */
static inline uint8_t bucket_0p5_22_28(float t) {
    float f = (t - 22.5f) * 2.0f + 1.0f;  /* 22.5->1, 23.0->2, 23.5->3 ... */
    int b = (int)(f + 0.0f);
    if (b < 0) b = 0;
    if (b > 12) b = 12;
    return (uint8_t)b;
}

/* Simple excursion detector; adjust to your policy */
static inline bool is_excursion(float t) { return (t < 23.0f || t > 24.5f); }

/* ===== ACTIVE header/cache ===== */
static void hdr_load_or_init(void) {
    const log_header_t *h = xip_hdr_active();
    if (h->magic != LOG_MAGIC) {
        log_header_t nh = { .magic = LOG_MAGIC, .count = 0 };
        flash_erase_sectors(LOG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
        flash_program_block(LOG_FLASH_OFFSET, &nh, FLASH_PAGE_SIZE);
        g.hdr = nh;
        g.write_pos = 0;
        g.page_fill = 0;
        g.last_time_ms = 0;
        memset(g.page_buf, 0xFF, sizeof g.page_buf);
        return;
    }
    g.hdr = *h;
    g.write_pos = g.hdr.count * sizeof(log_rec4_t);
    g.page_fill = 0;
    g.last_time_ms = 0;
    memset(g.page_buf, 0xFF, sizeof g.page_buf);
}

int bmp388_storage_init(void) {
    hdr_load_or_init();
    return 0;
}

uint32_t bmp388_storage_count(void) {
    return g.hdr.count;
}

/* Chronological RAM-log accessors */
static inline void ramlog_add(uint32_t t_ms, float temp) {
    uint32_t pos;
    if (g_ram.count < RAMLOG_CAP) {
        pos = (g_ram.head + g_ram.count) % RAMLOG_CAP;
        g_ram.count++;
    } else {
        pos = g_ram.head;
        g_ram.head = (g_ram.head + 1) % RAMLOG_CAP;
    }
    g_ram.t_ms[pos] = t_ms;
    g_ram.temp[pos] = temp;
}
static inline uint32_t ramlog_count(void) { return g_ram.count; }
static inline void ramlog_get(uint32_t i, uint32_t *t_ms, float *temp) {
    uint32_t pos = (g_ram.head + i) % RAMLOG_CAP;
    if (t_ms) *t_ms = g_ram.t_ms[pos];
    if (temp) *temp = g_ram.temp[pos];
}

/* Flush buffered page to flash ACTIVE data area */
static int flush_page_if_full(void) {
    if (g.page_fill == 0) return 0;
    uint32_t off = LOG_FLASH_OFFSET + FLASH_PAGE_SIZE + g.write_pos - g.page_fill;
    flash_program_block(off, g.page_buf, FLASH_PAGE_SIZE);
    g.page_fill = 0;
    memset(g.page_buf, 0xFF, sizeof g.page_buf);
    /* Update header count */
    flash_program_block(LOG_FLASH_OFFSET, &g.hdr, FLASH_PAGE_SIZE);
    return 0;
}

/* Append record to ACTIVE (and mirror to RAM) */
int bmp388_storage_append(uint32_t time_ms, float temperature_c) {
    /* Page-full handling */
    if ((g.write_pos + sizeof(log_rec4_t)) > (LOG_REGION_SIZE - FLASH_PAGE_SIZE)) {
        /* ACTIVE full: just stop appending (or rotate if you still use BACKUP) */
        return -1;
    }

    uint32_t delta_ms = (g.last_time_ms <= time_ms) ? (time_ms - g.last_time_ms) : 0;
    uint32_t delta_10 = delta_ms / 10u; if (delta_10 > 0xFFFFu) delta_10 = 0xFFFFu;

    int32_t t_centi = (int32_t)(temperature_c * 100.0f);
    if (t_centi >  32767) t_centi =  32767;
    if (t_centi < -32768) t_centi = -32768;

    log_rec4_t rec = { .dtime_10ms = (uint16_t)delta_10, .temp_c_centi = (int16_t)t_centi };

    memcpy(&g.page_buf[g.page_fill], &rec, sizeof rec);
    g.page_fill  += sizeof rec;
    g.write_pos  += sizeof rec;
    g.hdr.count  += 1;
    g.last_time_ms = g.last_time_ms + (uint32_t)rec.dtime_10ms * 10u;

    /* Mirror to RAM for dumps */
    ramlog_add(time_ms, temperature_c);

    /* Flush when the buffer reaches a page boundary */
    if (g.page_fill >= FLASH_PAGE_SIZE) return flush_page_if_full();
    return 0;
}

/* Optional: read from ACTIVE by replaying deltas (kept for compatibility) */
int bmp388_storage_read(uint32_t index, uint32_t *out_ms, float *out_temp) {
    if (index >= g.hdr.count) return -1;
    /* Simple linear walk (compact driver prefers RAM mirror anyway) */
    const uint8_t *p = xip_data_active();
    uint32_t t_ms = 0;
    for (uint32_t i = 0; i <= index; i++) {
        const log_rec4_t *r = (const log_rec4_t *)&p[i * sizeof(log_rec4_t)];
        t_ms += (uint32_t)r->dtime_10ms * 10u;
        if (i == index) {
            if (out_ms) *out_ms = t_ms;
            if (out_temp) *out_temp = ((float)r->temp_c_centi) / 100.0f;
        }
    }
    return 0;
}

void bmp388_storage_erase_all(void) {
    flash_erase_sectors(LOG_FLASH_OFFSET, LOG_REGION_SIZE);
    hdr_load_or_init();
}

/* ===== Shared compact-bit encoder (callback-based, single source of truth) ===== */
typedef void (*cbit_emit_fn)(uint8_t byte, void *ctx);
static inline void emit_byte(cbit_emit_fn f, void *ctx, uint8_t b, size_t *ctr) { f(b, ctx); (*ctr)++; }

static inline uint8_t bucket_0p5_22_28(float c);

size_t bmp388_compact_encode(cbit_emit_fn emit, void *ctx) {
    size_t out = 0;
    const uint32_t n = ramlog_count();

    /* Header: 0xC1, 0x01, count(LE16), bucket_count=13 */
    emit_byte(emit, ctx, 0xC1, &out);
    emit_byte(emit, ctx, 0x01, &out);
    emit_byte(emit, ctx, (uint8_t)(n & 0xFF), &out);
    emit_byte(emit, ctx, (uint8_t)(n >> 8),   &out);
    emit_byte(emit, ctx, 13,                  &out);

    if (n == 0) { emit_byte(emit, ctx, OPC_END, &out); return out; }

    /* First timestamp marker (oldest sample) */
    uint32_t t0_ms; float t0c;
    ramlog_get(0, &t0_ms, &t0c);
    emit_byte(emit, ctx, OPC_TS, &out);
    emit_byte(emit, ctx, (uint8_t)(t0_ms),       &out);
    emit_byte(emit, ctx, (uint8_t)(t0_ms >> 8),  &out);
    emit_byte(emit, ctx, (uint8_t)(t0_ms >> 16), &out);
    emit_byte(emit, ctx, (uint8_t)(t0_ms >> 24), &out);

    bool have_prev = false, prev_exc = false;
    uint8_t acc = 0;
    bool high_nibble = true;

    for (uint32_t i = 0; i < n; i++) {
        uint32_t t_ms; float temp;
        ramlog_get(i, &t_ms, &temp);

        bool exc = is_excursion(temp);
        if (!have_prev) { have_prev = true; prev_exc = exc; }
        else if (exc != prev_exc) {
            if (!high_nibble) { emit_byte(emit, ctx, acc, &out); acc=0; high_nibble=true; }
            emit_byte(emit, ctx, OPC_E, &out);
            prev_exc = exc;
        }

        uint8_t b = (uint8_t)(bucket_0p5_22_28(temp) & 0x0F);
        if (high_nibble) { acc = (uint8_t)(b << 4); high_nibble=false; }
        else { acc |= b; emit_byte(emit, ctx, acc, &out); acc=0; high_nibble=true; }
    }

    if (!high_nibble) emit_byte(emit, ctx, acc, &out);
    emit_byte(emit, ctx, OPC_END, &out);
    return out;
}

/* UART dumper via the same encoder */
static void uart_emit(uint8_t b, void *ctx) { (void)ctx; putchar_raw(b); }
void dump_active_compact_bit(void) {
    (void)bmp388_compact_encode(uart_emit, NULL);
}

/* ===== Save exact compact stream to flash and dump back raw ===== */
void bmp388_backup_compact_save(void) {
    static uint8_t staging[COMPACT_REGION_SIZE - FLASH_PAGE_SIZE];
    struct { uint8_t *p; size_t cap; size_t w; } bc = { staging, sizeof(staging), 0 };
    void buf_emit(uint8_t b, void *v) {
        struct { uint8_t *p; size_t cap; size_t w; } *ctx = v;
        if (ctx->w < ctx->cap) ctx->p[ctx->w++] = b;
    }

    (void)bmp388_compact_encode(buf_emit, &bc);

    /* Erase region and write header + data */
    flash_erase_sectors(COMPACT_FLASH_OFFSET, COMPACT_REGION_SIZE);

    compact_hdr_t hdr;
    memset(&hdr, 0xFF, sizeof hdr);
    hdr.magic  = CB_MAGIC;
    hdr.length = (uint32_t)bc.w;
    flash_program_block(COMPACT_FLASH_OFFSET, &hdr, FLASH_PAGE_SIZE);

    uint32_t written = 0;
    while (written < hdr.length) {
        uint8_t page[FLASH_PAGE_SIZE];
        size_t chunk = (hdr.length - written > FLASH_PAGE_SIZE) ? FLASH_PAGE_SIZE : (hdr.length - written);
        memset(page, 0xFF, sizeof page);
        memcpy(page, &staging[written], chunk);
        uint32_t dst = COMPACT_FLASH_OFFSET + FLASH_PAGE_SIZE + written;
        flash_program_block(dst, page, FLASH_PAGE_SIZE);
        written += (uint32_t)chunk;
    }
    printf("[COMPACT] Saved %u bytes to flash.\n", (unsigned)hdr.length);
}

void dump_backup_compact_raw(void) {
    const compact_hdr_t *h = xip_hdr_compact();
    if (h->magic != CB_MAGIC) { printf("[COMPACT] No blob found.\n"); return; }
    const uint8_t *p = xip_data_compact();
    for (uint32_t i=0;i<h->length;i++) putchar_raw(p[i]);
    printf("\n[COMPACT] Dumped %u bytes.\n", (unsigned)h->length);
}
