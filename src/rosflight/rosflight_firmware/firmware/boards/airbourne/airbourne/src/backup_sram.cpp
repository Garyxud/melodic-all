#include <cstring> //For memcpy
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "backup_sram.h"

void backup_sram_init()
{
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
#pragma GCC diagnostic pop
    PWR_BackupRegulatorCmd(ENABLE);
}

void backup_sram_write(const void *src, size_t len)
{
    PWR_BackupAccessCmd(ENABLE);
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    std::memcpy(reinterpret_cast<void*>(BKPSRAM_BASE), src, len);
#pragma GCC diagnostic pop
    PWR_BackupAccessCmd(DISABLE);
}

void backup_sram_read(void *dst, size_t len)
{
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    std::memcpy(dst, reinterpret_cast<const void*>(BKPSRAM_BASE), len);
#pragma GCC diagnostic pop
}

void backup_sram_clear(size_t len)
{
    PWR_BackupAccessCmd(ENABLE);
#pragma GCC diagnostic push //Ignore old style cast from included library
#pragma GCC diagnostic ignored "-Wold-style-cast"
    std::memset(reinterpret_cast<void*>(BKPSRAM_BASE), 0, len);
#pragma GCC diagnostic pop
    PWR_BackupAccessCmd(DISABLE);
}
