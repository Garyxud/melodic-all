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

#include "pwm.h"

PWM_OUT::PWM_OUT(){}

void PWM_OUT::init(const pwm_hardware_struct_t* pwm_init, uint16_t frequency, uint32_t max_us, uint32_t min_us)
{
  GPIO_InitTypeDef gpio_init_struct;
  TIM_TimeBaseInitTypeDef tim_init_struct;
  TIM_OCInitTypeDef tim_oc_init_struct;

  port_ = pwm_init->GPIO;
  pin_  = pwm_init->GPIO_Pin;

  GPIO_PinAFConfig(port_, pwm_init->GPIO_PinSource, pwm_init->GIPO_AF_TIM);

  gpio_init_struct.GPIO_Pin 	= pin_;
  gpio_init_struct.GPIO_Mode 	= GPIO_Mode_AF;
  gpio_init_struct.GPIO_Speed	= GPIO_Speed_50MHz;
  gpio_init_struct.GPIO_OType = GPIO_OType_PP;
  gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_DOWN;
  GPIO_Init(port_, &gpio_init_struct);

  TIM_TypeDef* TIMPtr = pwm_init->TIM;

  //calculate timer values
  //This is dependent on how fast the SystemCoreClock is. (ie will change between stm32fX models)
  const uint16_t prescaler_default = 42;
  uint32_t freq_prescale = prescaler_default * 2;
  uint32_t tim_prescaler = prescaler_default;

  if (TIMPtr == TIM1 || TIMPtr == TIM8 || TIMPtr == TIM9 || TIMPtr == TIM10 || TIMPtr == TIM11)
  {
    //For F4's (possibly others) TIM9-11 have a max timer clk double that of all the other TIMs
    //compensate for this by doubling its prescaler
    tim_prescaler = tim_prescaler * 2;
  }
  uint32_t timer_freq_hz = SystemCoreClock / freq_prescale;

  cycles_per_us_ = timer_freq_hz / 1000000;//E^6
  max_cyc_ = max_us * cycles_per_us_;
  min_cyc_ = min_us * cycles_per_us_;

  //init timer
  TIM_TimeBaseStructInit(&tim_init_struct);
  tim_init_struct.TIM_Period 		  = (2000000 / frequency) - 1; // 0 indexed
  tim_init_struct.TIM_Prescaler 	  = tim_prescaler - 1;
  tim_init_struct.TIM_ClockDivision = TIM_CKD_DIV1; //0x0000
  tim_init_struct.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(pwm_init->TIM, &tim_init_struct);

  //init output compare
  TIM_OCStructInit(&tim_oc_init_struct);
  tim_oc_init_struct.TIM_OCMode 		= TIM_OCMode_PWM2;
  tim_oc_init_struct.TIM_OutputState 	= TIM_OutputState_Enable;
  tim_oc_init_struct.TIM_OutputNState = TIM_OutputNState_Disable;
  tim_oc_init_struct.TIM_Pulse 		= min_cyc_ - 1;
  tim_oc_init_struct.TIM_OCPolarity 	= TIM_OCPolarity_Low;
  tim_oc_init_struct.TIM_OCIdleState 	= TIM_OCIdleState_Set;

  switch (pwm_init->TIM_Channel)
  {
  case TIM_Channel_1:
  default:
    TIM_OC1Init(TIMPtr, &tim_oc_init_struct);
    TIM_OC1PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
    CCR_ = &TIMPtr->CCR1;
    break;
  case TIM_Channel_2:
    TIM_OC2Init(TIMPtr, &tim_oc_init_struct);
    TIM_OC2PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
    CCR_ = &TIMPtr->CCR2;
    break;
  case TIM_Channel_3:
    TIM_OC3Init(TIMPtr, &tim_oc_init_struct);
    TIM_OC3PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
    CCR_ = &TIMPtr->CCR3;
    break;
  case TIM_Channel_4:
    TIM_OC4Init(TIMPtr, &tim_oc_init_struct);
    TIM_OC4PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
    CCR_ = &TIMPtr->CCR4;
    break;
  }

  // Set Main Output Enable Bit
  if (TIMPtr == TIM1 || TIMPtr == TIM8)
  {
    TIM_CtrlPWMOutputs(TIMPtr, ENABLE);
  }

  TIM_ARRPreloadConfig(TIMPtr, ENABLE);
  TIM_Cmd(TIMPtr, ENABLE);
}

void PWM_OUT::enable() {
  GPIO_InitTypeDef gpio_init_struct;
  gpio_init_struct.GPIO_Mode = GPIO_Mode_AF;
  gpio_init_struct.GPIO_Pin = pin_;

  GPIO_Init(port_, &gpio_init_struct);
}

void PWM_OUT::disable() {
  //This could conflict with the GPIO_PinAFConfig above
  GPIO_InitTypeDef gpio_init_struct;
  gpio_init_struct.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init_struct.GPIO_Pin = pin_;

  GPIO_Init(port_, &gpio_init_struct);
  GPIO_ResetBits(port_, pin_);
}

void PWM_OUT::write(float value) {
  *CCR_ = min_cyc_ + static_cast<uint16_t>((max_cyc_ - min_cyc_) * value);
}

void PWM_OUT::writeUs(uint16_t value) {
  *CCR_ = value * cycles_per_us_;
}
