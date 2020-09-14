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
#include "mpu6000.h"
#include "led.h"
#include "vcp.h"
#include "printf.h"

VCP* uartPtr = NULL;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    uartPtr->put_byte(c);
}

int main() {

  systemInit();

  VCP vcp;
  vcp.init();
  uartPtr = &vcp;

  init_printf(NULL, _putc);

  LED warn;
  warn.init(LED1_GPIO, LED1_PIN);
  LED info;
  info.init(LED2_GPIO, LED2_PIN);

  SPI mpu_spi;
  mpu_spi.init(&spi_config[MPU6000_SPI]);

  MPU6000 imu;
  imu.init(&mpu_spi);
  float temp;
  float acc[3];
  float gyro[3];
  uint64_t time_us;
  while(1)
  {
    info.toggle();
    imu.read(acc, gyro, &temp, &time_us);
    if (acc[0] == 0xFFFF || acc[0] == 0x0000)
    {
      warn.on();
      printf("error\n");
    }
    else
    {
      warn.off();
      printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
             (int32_t) (acc[0]*1000.0),
             (int32_t) (acc[1]*1000.0),
             (int32_t) (acc[2]*1000.0),
             (int32_t) (gyro[0]*1000.0),
             (int32_t) (gyro[1]*1000.0),
             (int32_t) (gyro[2]*1000.0),
             (int32_t) (temp*1000.0),
             time_us);
    }
    delay(10);
  }
}
