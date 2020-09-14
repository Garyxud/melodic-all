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

#include "system.h"
#include "i2c.h"
#include "ms4525.h"
#include "led.h"
#include "vcp.h"
#include "printf.h"
#include "revo_f4.h"

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

  delay(500);

  info.on();
  I2C i2c1;
  i2c1.init(&i2c_config[EXTERNAL_I2C]);
  MS4525 airspeed;


  if (!airspeed.init(&i2c1))
  {
    warn.on();
    delay(100);
    warn.off();
  }

  float diff_press(0), temp(0);
  uint32_t last_print_ms = 0;
  while(1) {
    info.on();
    airspeed.update();
    if (airspeed.present())
    {
      airspeed.read(&diff_press, &temp);
      warn.off();
    }
    else
    {
      warn.on();      
    }
    
    if (millis() > last_print_ms + 50)
    {
      last_print_ms = millis();        
      printf("%d.%dPa, %d.%dC\n",
               (int32_t)(diff_press), (int32_t)(diff_press*1000)%1000,
               (int32_t)(temp), (int32_t)(temp*1000)%1000);
    }
  }
}
