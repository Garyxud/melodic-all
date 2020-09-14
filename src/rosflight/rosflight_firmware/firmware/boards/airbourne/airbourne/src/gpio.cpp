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

#include "gpio.h"

void GPIO::init(GPIO_TypeDef* BasePort, uint16_t pin, gpio_mode_t mode)
{
  pin_ = pin;
  port_ = BasePort;
  set_mode(mode);
}

void GPIO::write(gpio_write_t state)
{
  if(mode_ == OUTPUT)
  {
    if(state == LOW)
      GPIO_ResetBits(port_, pin_);
    else
      GPIO_SetBits(port_, pin_);
  }
}

void GPIO::toggle()
{
  if(mode_ == OUTPUT)
  {
    if(GPIO_ReadOutputDataBit(port_, pin_))
      GPIO_ResetBits(port_, pin_);
    else
      GPIO_SetBits(port_, pin_);
  }
}

bool GPIO::read()
{
  // If it's an input pin, use the read input data
  if(mode_ == INPUT)
  {
    if(port_->IDR & pin_)
      return HIGH;
    else
      return LOW;
  }
  else
  {
    if(port_->ODR & pin_)
      return HIGH;
    else
      return LOW;
  }
}

void GPIO::set_mode(gpio_mode_t mode)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = pin_;

  switch(mode)
  {
  case OUTPUT:
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    break;
  case PERIPH_OUT:
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    break;
  case PERIPH_IN:
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    break;
  case PERIPH_IN_OUT:
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    break;
  case ANALOG:
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    break;
  case INPUT:
  case EXTERNAL_INTERRUPT:
  default:
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    break;
  }

  // Who cares about power usage?  Go as fast as possible.
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

  // Initialize the GPIO
  GPIO_Init(port_, &GPIO_InitStruct);
  mode_ = mode;
  write(LOW);
}
