/*
 * Copyright (c) 2017, James Jackson and Trey Henrichsen
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "string.h"
#include "system.h"

#include "eeprom.h"

void eeprom_init()
{
  return;//nothing needed
}
//Reads the data stored in flash sector 11
bool eeprom_read(void * dest, uint8_t len)
{
  memcpy(dest, FLASH_SECTOR11_START, len);
  return true;
}
//writes to the 11th sector, which is used for persistant storage
bool eeprom_write(const void* src, uint8_t len)
{
  return flash_write(FLASH_SECTOR11_START, src,len);
}
//Reads from the specified location. Note that there is no error checking
bool flash_read(void* flash_loc,void* dest, uint8_t len)
{
  memcpy(dest, flash_loc, len);
  return true;
}
//Erases the 11th sector, which is used for persistent storage
bool flash_erase()
{
  FLASH_Unlock();
  FLASH_EraseSector(FLASH_Sector_11,VoltageRange_3);
  return true;
}
//writes data to the given location. Note that there is no error checking
bool flash_write(void const* flash_loc, void const* data, uint8_t len)
{
  flash_erase();
  FLASH->CR|=FLASH_CR_PG;
  uint32_t const* dataPtr = static_cast<uint32_t const*>(data);
  uint32_t flash_address= reinterpret_cast<uint32_t>(flash_loc);
  for (uint32_t i=0;i<len;i+=4)
    FLASH_ProgramWord(flash_address+i,*(dataPtr++));
  return true;
}
