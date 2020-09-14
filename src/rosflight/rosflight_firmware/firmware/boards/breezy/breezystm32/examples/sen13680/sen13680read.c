/*
   sen13680read.c : read values from Sparkfun SEN13680 LIDAR-Lite v2 I^2C lidar

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

static bool lidar_present;

// initialize i2c
void setup(void)
{
  i2cInit(I2CDEV_2);
  i2cWrite(0, 0, 0);
}

// look for the lidar and then read measurements
void loop(void)
{
  if(lidar_present)
  {
    sen13680_update();
    float distance = sen13680_read();
    printf("distance = %d.%dm\n", (uint32_t)distance, (uint32_t)(distance*1000)%1000);
  }
  else
  {
    printf("No lidar present. Reinitializing.\n");
    lidar_present = sen13680_init();
  }
  delay(100);
  LED1_TOGGLE;
}
