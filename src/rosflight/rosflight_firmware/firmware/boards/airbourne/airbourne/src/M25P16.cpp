/*
 * Copyright (c) 2017, James Jackson
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

#include "M25P16.h"
#include "revo_f4.h"

M25P16::M25P16() {}

void M25P16::init(SPI* _spi)
{
  // Set up the SPI peripheral
  spi_ = _spi;
  spi_->set_divisor(2);

  // Set up the clock select pin
  cs_.init(FLASH_CS_GPIO, FLASH_CS_PIN, GPIO::OUTPUT);
}

uint8_t M25P16::get_status()
{
  // Send the address byte
  spi_->enable(cs_);
  spi_->transfer_byte(READ_STATUS);

  // Push a garbage byte to clock out the status
  uint8_t status = spi_->transfer_byte(0xFF);
  spi_->disable(cs_);
  return status;
}

bool M25P16::read_config(uint8_t *data, uint32_t len)
{
  // Send the read data command, with address 0
  // Then clock out the right number of bytes
  spi_->enable(cs_);
  uint8_t addr[4] = {READ_DATA, 0, 0, 0};
  spi_->transfer(addr, 4, nullptr);
  while (spi_->is_busy()) {}
  spi_->transfer(nullptr, len, data);
  while (spi_->is_busy());
  spi_->disable(cs_);
  return true;
}

bool M25P16::write_config(const uint8_t *data, const uint32_t len)
{
  // Calculate the correct number of pages to store the config
  num_pages_for_config_ = len / 256;
  if (len % 256 != 0)
    num_pages_for_config_ ++; // We need an extra partial page

  // Enable the write
  spi_->transfer_byte(WRITE_ENABLE, &cs_);

  // Make sure we can erase (WEL bit is set)
  uint8_t status = get_status();
  if (!(status & STATUS_WEL_BIT))
    return false;

  // Erase Sector (There is really no way around this, we have to erase the entire sector
  uint8_t sector_addr[4] = {SECTOR_ERASE, 0, 0, 0};
  spi_->transfer(sector_addr, 4, NULL, &cs_);
  while (spi_->is_busy()) {}

  // Wait for Sector Erase to complete
  bool WIP = true;
  do
  {
    status = get_status();
    if ((status & STATUS_WIP_BIT) == 0x00)
      WIP = false;
  } while(WIP);

  // Program the data
  for (uint32_t i = 0; i < num_pages_for_config_; i++)
  {
    // Re-Enable the write (the WEL bit is reset after each program completion)
    spi_->transfer_byte(WRITE_ENABLE, &cs_);

    // Make sure that the WEL bit has been set, so we can write
    status = get_status();

    // Figure out how much of this page we are going to use
    uint16_t page_len = 256;
    if (i == num_pages_for_config_ - 1)
      page_len = len % 256; // If this is last page, then we write the partial

    // Send the PAGE_PROGRAM command with the right address
    spi_->enable(cs_);
    uint8_t addr[4] = {PAGE_PROGRAM, static_cast<uint8_t>(i >> 8), static_cast<uint8_t>(i & 0xFF), 0};
    spi_->transfer(addr, 4, NULL, NULL);
    while (spi_->is_busy()) {} // Wait for the address to clear

    // Transfer the data
    spi_->write(&data[256*i], page_len);
    while (spi_->is_busy()) {} // Wait for the page to write
    spi_->disable(cs_);

    // Wait for the page program to happen
    WIP = true;
    do
    {
      status = get_status();
      if ((status & STATUS_WIP_BIT) == 0x00)
        WIP = false;
    } while(WIP);
  }

  // Disable the write
  spi_->transfer_byte(WRITE_DISABLE, &cs_);
  return true;
}
