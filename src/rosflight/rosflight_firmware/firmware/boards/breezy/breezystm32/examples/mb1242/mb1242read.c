/*
   mb1242read.c : read values from MaxBotix MB1242 I^2C sonar

   Don't forget to supply external power to the board!

   Copyright (C) 2016 Simon D. Levy 

   This file is part of BreezySTM32.

   BreezySTM32 is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BreezySTM32 is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <breezystm32.h>

static bool sonar_present;

void setup(void)
{
    i2cInit(I2CDEV_2);
    delay(500);
    sonar_present = mb1242_init();
}

void loop(void)
{
  mb1242_async_update();
  delay(25);
  LED1_TOGGLE;
  if(mb1242_present() || sonar_present)
  {
        float distance = mb1242_async_read();
        printf("distance = %d.%dm\n", (uint32_t)distance, (uint32_t)(distance*1000)%1000);
  }
  else
        printf("no sonar\n");
}
