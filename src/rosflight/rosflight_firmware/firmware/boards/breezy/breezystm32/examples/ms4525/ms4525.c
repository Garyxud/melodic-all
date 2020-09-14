/*
   ms4525.c : Airpseed Measurement Values

   Copyright (C) 2016 James Jackson

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

#include "math.h"

bool airspeed_present = false;
bool barometer_present = false;

void setup(void)
{
    delay(500);
    i2cInit(I2CDEV_2);

    airspeed_present = ms4525_init();

//    barometer_present = ms5611_init();
}



float pressure, altitude, temperature;
void loop(void)
{
//    ms5611_async_update();
    ms4525_async_update();

    delay(10);

//    if (ms5611_present())
    {
//      ms5611_async_read(&altitude, &pressure, &temperature);
//      ms4525_set_atm((uint32_t) pressure);
    }
    if (ms4525_present())
    {
        float velocity, diff_pressure, temp;
        ms4525_async_read(&diff_pressure, &temp, &velocity);
//        printf("calibrated = %d\tvel: %d.%d m/s\tdiff_press: %d.%dPa\ttemp:%d.%dK\n",
        printf("%d.%d,\t%d.%d,\t%d.%d,\t%d.%d\n",
//               ms4525_calibrated(),
               (int32_t)velocity, (int32_t)(fabs(velocity)*1000)%1000,
               (int32_t)diff_pressure, (int32_t)(fabs(diff_pressure)*1000)%1000,
               (int32_t)temp, (int32_t)(temp*1000)%1000,
               (int32_t)pressure, (int32_t)(pressure*1000)%1000);
    }
    else
    {
        printf("no airspeed\n");
    }
}

