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

#include "revo_f4.h"

#include "spi.h"
#include "M25P16.h"
#include "led.h"
#include "vcp.h"
#include "printf.h"

#include <cstdlib>

VCP vcp;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    vcp.put_byte(c);
}

typedef struct
{
  uint8_t magic_BE = 0xBE;
  uint8_t big_array[2048];
  uint8_t magic_AC = 0xAC;
  uint8_t big_array2[2048];
  uint8_t magic_D3 = 0xD3;
  uint8_t crc;
} config_t;



int main()
{
  static config_t config_file;
  static uint8_t config_buffer[sizeof(config_t)];

  systemInit();

  vcp.init();
  init_printf(NULL, _putc);

  config_file.magic_AC = 0xAC;
  config_file.magic_BE = 0xBE;
  config_file.magic_D3 = 0xD3;

  for (int i = 0; i < 2048; i++)
  {
    config_file.big_array[i] = (uint8_t)std::rand();
    config_file.big_array2[i] = (uint8_t)std::rand();
  }

  LED warn;
  warn.init(LED1_GPIO, LED1_PIN);
  LED info;
  info.init(LED2_GPIO, LED2_PIN);

  SPI spi;
  spi.init(&spi_config[FLASH_SPI]);

  M25P16 flash;
  flash.init(&spi);

  // calculate crc
  uint8_t crc = 0;
  for (uint8_t* p = (uint8_t*)&config_file; p < (uint8_t*)&config_file + sizeof(config_file); p++)
  {
    crc ^= *p;
  }
  config_file.crc = crc;

  info.on();

  bool success = false;

  // write the config to flash
  flash.write_config((uint8_t*)&config_file, sizeof(config_t));

  // Read config from flash
  flash.read_config(config_buffer, sizeof(config_t));

  // See if it is valid
  config_t* config_ptr = (config_t*) config_buffer;

  // Calculate crc of new data
  crc = 0;
  for (uint8_t* p = (uint8_t*) config_ptr; p < (uint8_t*)config_ptr + sizeof(config_file); p++)
  {
    crc ^= *p;
  }

  if (config_ptr->magic_AC == 0xAC &&
      config_ptr->magic_BE == 0xBE &&
      config_ptr->magic_D3 == 0xD3 &&
      crc == 0)
  {
    warn.on();
    success = true;
  }
  else
  {
    warn.off();
    success = false;
  }

  while(1)
  {
    uint32_t size = sizeof(config_file);
    info.toggle();
    delay(1000);
    if (success)
      printf("successfully wrote, read and validated %d.%dKB worth of data\n", size/1000, size%1000);
    else
      printf("failed\n");
  }
}

