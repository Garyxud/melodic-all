#ifndef BACKUP_SRAM_H
#define BACKUP_SRAM_H

#include <stdint.h>

void backup_sram_init();
void backup_sram_write(const void *src, size_t len);
void backup_sram_read(void *dst, size_t len);
void backup_sram_clear(size_t len);

#endif // BACKUP_SRAM_H
