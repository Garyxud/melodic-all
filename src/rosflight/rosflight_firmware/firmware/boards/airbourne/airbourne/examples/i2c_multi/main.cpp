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
#include "ms5611.h"
#include "hmc5883l.h"
#include "ms4525.h"
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
  
  delay(500);
  
  info.on();
  
  // Initialize the I2C peripherals1
  I2C i2c1;
  I2C i2c2;
  i2c1.init(&i2c_config[BARO_I2C]);
//  i2c2.init(&i2c_config[EXTERNAL_I2C]);
  
  // Initialize the sensors
  MS5611 baro;
  HMC5883L mag;
//  MS4525 airspeed;
  
  
  // Initialize the barometer
  float pressure(0.0), baro_temp(0.0);
  baro.init(&i2c1);
  
  // Initialize the Magnetometer
  float mag_data[3] = {0., 0., 0.};
  mag.init(&i2c1);
  
  // Initialize the airspeed Sensor
  float diff_press(0.0), as_temp(0.0);
//  airspeed.init(&i2c2);
  
  uint32_t count;
  uint32_t last_print = 0;
  while(1)
  {
    baro.update();
    mag.update();
//    airspeed.update();
    if (baro.present())
    {
      baro.read(&pressure, &baro_temp);
    }
    else
    {
      pressure = 0.0;
      baro_temp = 0.0;
    }
    
    double seconds = millis()/1000.0;
    
    if (mag.present())
    {
      mag.read(mag_data);
    }
    else
    {
      for (int i =0; i < 3; i++) mag_data[i] = 0.0;
    }
//    if (airspeed.present())
//    {
//      airspeed.read(&diff_press, &as_temp);
//    }
//    else
//    {
//      diff_press = 0.0;
//      as_temp = 0.0;
//    }
    
    if (pressure == 0.0 || baro_temp == 0.0 || mag_data[0] == 0.0)
      warn.on();
    else
      warn.off();
           
    if (millis() > last_print + 20)
    {
      if (count++ % 10 == 0)
        info.toggle();
      last_print = millis();
      printf("t: %.2f\t", seconds);
      printf("baro: %d Pa, %.2f K\t", (int32_t)pressure, (double)baro_temp);
      printf("mag: %d, %d, %d\t",(int32_t)(mag_data[0]),(int32_t)(mag_data[1]),(int32_t)(mag_data[2]));
  //    printf("as: %.2f Pa, %.2fC\t", (double)diff_press, (double)as_temp);
      printf ("err1: %d\t err2:%d\n", i2c1.num_errors(), i2c2.num_errors());    
    }
  }
}
    
