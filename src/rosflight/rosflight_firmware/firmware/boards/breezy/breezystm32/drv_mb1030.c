#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "stm32f10x_conf.h"
#include "core_cm3.h"

#include "drv_gpio.h"
#include "drv_timer.h"
#include "drv_pwm.h"

typedef struct{
  TIM_TypeDef *tim;
  GPIO_TypeDef *gpio;
  uint32_t pin;
  uint8_t channel;
  uint8_t irq;
} sonar_t;

static sonar_t sonar_hardware[6] = {
  { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn },          // RC2
  { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn },          // RC3
  { TIM2, GPIOA, Pin_3, TIM_Channel_4, TIM2_IRQn },          // RC4
  { TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn },          // RC5
  { TIM3, GPIOA, Pin_7, TIM_Channel_2, TIM3_IRQn },          // RC6
  { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn },          // RC7
};

void mb1030_init()
{
  for(int i = 0; i < 6; i++)
  {
    pwmGPIOConfig(sonar_hardware[i].gpio, sonar_hardware[i].pin, Mode_IPD);
    pwmICConfig(sonar_hardware[i].tim, sonar_hardware[i].channel, TIM_ICPolarity_Rising);
}
