/*
   accelgyro.c : report accelerometer and gyroscope values

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

#define BOARD_REV 2

float accel_scale; // converts to units of m/s^2
float gyro_scale; // converts to units of rad/s

volatile int16_t accel_data[3];
volatile int16_t gyro_data[3];
volatile int16_t temp_data;

volatile uint8_t accel_status = 0;
volatile uint8_t gyro_status = 0;
volatile uint8_t temp_status = 0;
volatile bool mpu_new_measurement = false;

uint32_t start_time = 0;

void setup(void)
{
    delay(500);
    i2cInit(I2CDEV_2);

    uint16_t acc1G;
    mpu6050_init(true, &acc1G, &gyro_scale, BOARD_REV);
    accel_scale = 9.80665f / acc1G;
}

void loop(void)
{
//  printf("test");
//  delay(100);
    if (mpu6050_new_data())
    {
        LED0_TOGGLE;
        volatile uint64_t time_us;
        mpu6050_async_read_all(accel_data, &temp_data, gyro_data, &time_us);
        printf("%d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t \n",
               (int32_t)(accel_data[0]*accel_scale*1000.0f),
                (int32_t)(accel_data[1]*accel_scale*1000.0f),
                (int32_t)(accel_data[2]*accel_scale*1000.0f),
                (int32_t)(gyro_data[0]*gyro_scale*1000.0f),
                (int32_t)(gyro_data[1]*gyro_scale*1000.0f),
                (int32_t)(gyro_data[2]*gyro_scale*1000.0f),
                (int32_t)temp_data,
                (int32_t)time_us);
//        printf("test");
        delay(50);
    }
}
