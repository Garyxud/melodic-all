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

#ifndef SYSTEM_H
#define SYSTEM_H

#define ARM_MATH_CM4

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <stdarg.h>

#pragma GCC system_header

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_adc.h"
#include "misc.h"

typedef struct {
	SPI_TypeDef* dev;
	GPIO_TypeDef* GPIO;
	uint8_t SCK_PinSource;
	uint16_t SCK_Pin;
	uint8_t MOSI_PinSource;
	uint16_t MOSI_Pin;
	uint8_t MISO_PinSource;
	uint16_t MISO_Pin;
	uint8_t GPIO_AF;
	IRQn_Type DMA_IRQn;
	DMA_Stream_TypeDef* Tx_DMA_Stream;
	DMA_Stream_TypeDef* Rx_DMA_Stream;
	uint32_t DMA_Channel;
	uint32_t Tx_DMA_TCIF;
	uint32_t Rx_DMA_TCIF;
} spi_hardware_struct_t;

typedef struct {
	I2C_TypeDef* dev;
	uint32_t I2C_ClockSpeed;
	IRQn_Type I2C_EV_IRQn;
	IRQn_Type I2C_ER_IRQn;
	GPIO_TypeDef* GPIO;
	uint8_t GPIO_AF;
	uint8_t SCL_PinSource;
	uint16_t SCL_Pin;
	uint8_t SDA_PinSource;
	uint16_t SDA_Pin;
	DMA_Stream_TypeDef* DMA_Stream;
	uint32_t DMA_Channel;
	IRQn_Type DMA_IRQn;
	uint32_t DMA_TCIF;
} i2c_hardware_struct_t;

typedef struct {
	GPIO_TypeDef* GPIO;
	uint16_t GPIO_Pin;
	uint8_t GPIO_PinSource;
	TIM_TypeDef* TIM;
	uint8_t TIM_Channel;
	uint8_t GIPO_AF_TIM;
	IRQn_Type TIM_IRQn;
	uint16_t TIM_IT_CC;
} pwm_hardware_struct_t;

typedef struct {
	USART_TypeDef* dev;
	GPIO_TypeDef* GPIO;
	uint16_t Rx_Pin;
	uint16_t Tx_Pin;
	uint8_t Rx_PinSource;
	uint8_t Tx_PinSource;
	uint8_t GPIO_AF;
	IRQn_Type USART_IRQn;
	IRQn_Type Rx_DMA_IRQn;
	IRQn_Type Tx_DMA_IRQn;
	DMA_Stream_TypeDef* Rx_DMA_Stream;
	DMA_Stream_TypeDef* Tx_DMA_Stream;
	uint32_t DMA_Channel;
	uint32_t DMA_Rx_IT_Bit;
	uint32_t DMA_Tx_IT_Bit;
} uart_hardware_struct_t;

struct ADCHardwareStruct
{
  ADC_TypeDef *adc;
  DMA_Stream_TypeDef *DMA_Stream;
  uint32_t DMA_channel;
};

struct BatteryMonitorHardwareStruct
{
  GPIO_TypeDef *voltage_gpio;
  uint16_t voltage_pin;
  uint8_t voltage_adc_channel;
  GPIO_TypeDef *current_gpio;
  uint16_t current_pin;
  uint8_t current_adc_channel;
  const struct ADCHardwareStruct *adc;
};

#ifdef __cplusplus
extern "C" {
#endif

void systemInit(void);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

volatile uint64_t micros(void);
volatile uint32_t millis(void);

// bootloader/IAP
void systemReset(void);
void systemResetToBootloader(void);

#ifdef __cplusplus
}
#endif

#endif //SYSTEM_H
