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

#include "mpu6000.h"

uint8_t raw[15] = {MPU_RA_ACCEL_XOUT_H | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

MPU6000* IMU_Ptr;

void data_transfer_cb(void)
{
  IMU_Ptr->data_transfer_callback();
}

void MPU6000::write(uint8_t reg, uint8_t data)
{
  spi->enable(cs_);
  spi->transfer_byte(reg);
  spi->transfer_byte(data);
  spi->disable(cs_);
  delayMicroseconds(1);
}

void MPU6000::init(SPI* spi_drv)
{
  IMU_Ptr = this;
  spi = spi_drv;

  // Configure Chip Select Pin
  cs_.init(MPU6000_CS_GPIO, MPU6000_CS_PIN, GPIO::OUTPUT);
  spi->set_divisor(2); // 21 MHz SPI clock (within 20 +/- 10%)

  write(MPU_RA_PWR_MGMT_1, MPU_BIT_H_RESET);
  delay(150);

  write(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
  write(MPU_RA_USER_CTRL, MPU_BIT_I2C_IF_DIS);
  write(MPU_RA_PWR_MGMT_2, 0x00);
  write(MPU_RA_SMPLRT_DIV, 0x00);
  write(MPU_RA_CONFIG, MPU_BITS_DLPF_CFG_98HZ);
  write(MPU_RA_ACCEL_CONFIG, MPU_BITS_FS_4G);
  write(MPU_RA_GYRO_CONFIG, MPU_BITS_FS_2000DPS);
  write(MPU_RA_INT_ENABLE, 0x01);



  for (int i = 0; i < 100; i++)
  // Read the IMU
  raw[0] = MPU_RA_TEMP_OUT_H | 0x80;
  spi->transfer(raw, 3, raw, &cs_);
  delay(20);
//  while (spi->is_busy()) {}

  // Set up the EXTI pin
  exti_.init(GPIOC, GPIO_Pin_4, GPIO::INPUT);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);

  EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.EXTI_Line = EXTI_Line4;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&EXTI_InitStruct);

  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x03;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);


  // set the accel and gyro scale parameters
  accel_scale_ = (4.0 * 9.80665f) / (static_cast<float>(0x7FFF));
  gyro_scale_= (2000.0 * 3.14159f/180.0f) / (static_cast<float>(0x7FFF));
}

void MPU6000::data_transfer_callback()
{
  new_data_ = true;
  acc_[0] = static_cast<float>(static_cast<int16_t>((raw[1] << 8) | raw[2])) * accel_scale_;
  acc_[1] = static_cast<float>(static_cast<int16_t>((raw[3] << 8) | raw[4])) * accel_scale_;
  acc_[2] = static_cast<float>(static_cast<int16_t>((raw[5] << 8) | raw[6])) * accel_scale_;

  temp_  = static_cast<float>(static_cast<int16_t>((raw[7] << 8) | raw[8])) / 340.0f + 36.53f;

  gyro_[0]  = static_cast<float>(static_cast<int16_t>((raw[9]  << 8) | raw[10])) * gyro_scale_;
  gyro_[1]  = static_cast<float>(static_cast<int16_t>((raw[11] << 8) | raw[12])) * gyro_scale_;
  gyro_[2]  = static_cast<float>(static_cast<int16_t>((raw[13] << 8) | raw[14])) * gyro_scale_;
}

void MPU6000::read(float* accel_data, float* gyro_data, float* temp_data, uint64_t* time_us)
{
  accel_data[0] = acc_[0];
  accel_data[1] = acc_[1];
  accel_data[2] = acc_[2];
  gyro_data[0] = gyro_[0];
  gyro_data[1] = gyro_[1];
  gyro_data[2] = gyro_[2];
  *temp_data = temp_;
  *time_us = imu_timestamp_;
}

void MPU6000::exti_cb()
{
  imu_timestamp_ = micros();
  raw[0] = MPU_RA_ACCEL_XOUT_H | 0x80;
  spi->transfer(raw, 15, raw, &cs_, &data_transfer_cb);
}

bool MPU6000::new_data() 
  {
    if (new_data_)
    {
      new_data_ = false;
      return true;
    }
    else
      return false;
  }

extern "C"
{

void EXTI4_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line4);
  IMU_Ptr->exti_cb();
}

}
