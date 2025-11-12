// flash_helpers.h

#ifndef FLASH_HELPERS_H
#define FLASH_HELPERS_H

#include <stdint.h>
#include <stddef.h> 

void flash_program_block(uint32_t flash_off, const void *data, size_t len);
void flash_erase_sectors(uint32_t flash_off, size_t len);

#endif // FLASH_HELPERS_H
