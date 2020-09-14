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
#include <stdint.h>

#define BOARD_REV 2

float accel_scale; // converts to units of m/s^2
float gyro_scale; // converts to units of rad/s

int16_t accel_data[3];
int16_t gyro_data[3];
volatile int16_t temp_data;
float mag_data[3];

volatile uint8_t accel_status = 0;
volatile uint8_t gyro_status = 0;
volatile uint8_t temp_status = 0;
volatile bool mpu_new_measurement = false;

uint32_t start_time = 0;

bool baro_present= false;
bool mag_present=false;
bool sonar_present=false;
bool airspeed_present=false;
void setup(void)
{
  delay(500);
  i2cInit(I2CDEV_2);

  // Init Baro
  i2cWrite(0, 0, 0);
  ms5611_init();

  // Init Mag
  hmc5883lInit();

  // Init Sonar
  mb1242_init();

  // Init Airspeed
  ms4525_init();

  //Init IMU (has to come last because of the ISR)
  uint16_t acc1G;
  mpu6050_init(true, &acc1G, &gyro_scale, BOARD_REV);
  accel_scale = 9.80665f / acc1G;
}

void loop(void)
{

  float baro = 0;
  float temp = 0;
  float diff_pres = 0;
  float diff_temp = 0;
  float sonar = 0;
    
  // Update Baro
  ms5611_async_update();
  if(ms5611_present())
  {
    ms5611_async_read(&baro, &temp);
  }

  // Update Mag
  hmc5883l_request_async_update();
  if(hmc5883l_present())
  {
    hmc5883l_async_read(mag_data);
  }

  // Update Sonar
  mb1242_async_update();
  if (mb1242_present())
  {
    sonar = mb1242_async_read();
  }
  //  if(sonar_present)
  //  {
  //    sonar = mb1242_poll();
  //  }

    // Update Airspeed
    ms4525_async_update();
    if(ms4525_present())
    {
      ms4525_async_read(&diff_pres, &diff_temp);
    }

  static uint64_t imu_timestamp_us = 0;
  if (mpu6050_new_data())
  {
    mpu6050_async_read_all(accel_data, &temp_data, gyro_data, &imu_timestamp_us);
  }

  static uint32_t last_print_ms = 0;
  // Throttle printing
  if(millis() > last_print_ms + 10)
  {
    last_print_ms += 10;
    
    printf("err: %d\t", i2cGetErrorCounter());
    printf("acc: %d.%d, %d.%d, %d.%d\t", (int32_t)accel_data[0], (uint32_t)(accel_data[0]*100)%100,
        (int32_t)accel_data[1], (uint32_t)(accel_data[1]*100)%100, (int32_t)accel_data[2], (uint32_t)(accel_data[2]*100)%100);
    printf("baro: %d Pa %d.%d K\t", (uint32_t) baro, (int32_t) temp, (uint32_t)(temp*100)%100);
    printf("mag: %d, %d, %d, \t", (int32_t) mag_data[0], (int32_t) mag_data[1], (int32_t) mag_data[2]);
    printf("as: %d.%d, %d.%d\t", (int32_t)diff_pres, (uint32_t)(diff_pres*100)%100, (int32_t) diff_temp, (uint32_t)(diff_temp*100)%100);
    printf("sonar: %d.%d", (int32_t)sonar, (uint32_t)(sonar*100)%100);
    
    printf("\n");
    
//    printf("%d\t %d\t %d\t %d\t %d\t %d\n",
////           (int32_t)(accel_data[2]*accel_scale*1000.0f),
////        (int32_t)(gyro_data[2]*gyro_scale*1000.0f),
//        (int32_t)imu_timestamp_us,
//        (int32_t)mag_data[2],
//        (int32_t)baro,
//        (int32_t)i2cGetErrorCounter());
  }
}
