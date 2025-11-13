#include "flash_helpers.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

// Implementation copied from storage_driver.c
void flash_program_block(uint32_t flash_off, const void *data, size_t len) {
    uint32_t irq = save_and_disable_interrupts();
    flash_range_program(flash_off, (const uint8_t*)data, len);
    restore_interrupts(irq);
}

// Implementation copied from storage_driver.c
void flash_erase_sectors(uint32_t flash_off, size_t len) {
    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(flash_off, len);
    restore_interrupts(irq);
}
