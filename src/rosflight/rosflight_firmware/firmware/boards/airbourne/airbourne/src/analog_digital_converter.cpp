/*
 * Copyright (c) 2020, Richard Henrichsen
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


#include "analog_digital_converter.h"

void AnalogDigitalConverter::init(const ADCHardwareStruct *adc_def)
{
  this->current_channels = 0;
  this->adc_def_ = adc_def;

  ADC_TypeDef *adc = adc_def_->adc;

  for (size_t index =0; index< CHANNEL_COUNT; index++)
    this->buffer[index]=AnalogDigitalConverter::NO_READING;

  ADC_CommonInitTypeDef adc_common_init_struct;
  adc_common_init_struct.ADC_Mode = ADC_Mode_Independent;
  adc_common_init_struct.ADC_Prescaler = ADC_Prescaler_Div2;
  adc_common_init_struct.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
  adc_common_init_struct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&adc_common_init_struct);

  ADC_InitTypeDef adc_init_struct;
  ADC_StructInit(&adc_init_struct);
  adc_init_struct.ADC_Resolution = ADC_Resolution_12b;
  adc_init_struct.ADC_ScanConvMode = ENABLE;
  adc_init_struct.ADC_ContinuousConvMode = ENABLE;
  adc_init_struct.ADC_DataAlign = ADC_DataAlign_Right;
  adc_init_struct.ADC_NbrOfConversion = 1; //This can't be less than 1
  ADC_Init(adc,&adc_init_struct);

  this->init_dma();
  ADC_ContinuousModeCmd(adc, ENABLE);

  this->is_initialized_ = true;
}

void AnalogDigitalConverter::init_dma()
{
  DMA_Cmd(this->adc_def_->DMA_Stream, DISABLE);
  DMA_DeInit(this->adc_def_->DMA_Stream);
  DMA_InitTypeDef dma_init_struct;
  DMA_StructInit(&dma_init_struct);

  dma_init_struct.DMA_Channel = this->adc_def_->DMA_channel;
  dma_init_struct.DMA_PeripheralBaseAddr = reinterpret_cast<uint32_t>(&(this->adc_def_->adc->DR));
  dma_init_struct.DMA_Memory0BaseAddr = reinterpret_cast<uint32_t>((this->buffer));
  dma_init_struct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  dma_init_struct.DMA_BufferSize = this->get_current_channel_count();
  dma_init_struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma_init_struct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  // The ADC data register is 32 bits wide, even though the data is only 12 bits wide
  dma_init_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  dma_init_struct.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  dma_init_struct.DMA_Mode = DMA_Mode_Circular;
  dma_init_struct.DMA_Priority = DMA_Priority_Medium;
  dma_init_struct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  dma_init_struct.DMA_FIFOMode = DMA_FIFOStatus_1QuarterFull;
  dma_init_struct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  dma_init_struct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  DMA_Init(this->adc_def_->DMA_Stream, &dma_init_struct);

  ADC_DMACmd(this->adc_def_->adc, ENABLE);
  ADC_DMARequestAfterLastTransferCmd(this->adc_def_->adc, ENABLE);
}

uint8_t AnalogDigitalConverter::add_channel(uint8_t channel)
{
  uint8_t index = this->get_current_channel_count() + 1;
  this->current_channels++;
  ADC_RegularChannelConfig(this->adc_def_->adc, channel, index, ADC_SampleTime_480Cycles);

  //Increment the number of channels
  this->adc_def_->adc->SQR1 &=(~SQR1_CHANNEL_COUNT_MASK);
  this->adc_def_->adc->SQR1 |=(((index-1)<<SQR1_CHANNEL_COUNT_OFFSET)&SQR1_CHANNEL_COUNT_MASK);

  this->init_dma(); // reconfigure the DMA with the new memory size
  this->start_dma();
  ADC_Cmd(this->adc_def_->adc, ENABLE);
  ADC_SoftwareStartConv(this->adc_def_->adc);
  return index;
}
void AnalogDigitalConverter::start_dma()
{
  DMA_Cmd(this->adc_def_->DMA_Stream, ENABLE);
}
bool AnalogDigitalConverter::is_initialized() const
{
  return this->is_initialized_;
}
uint16_t AnalogDigitalConverter::read(uint8_t index) const
{
  return (this->buffer[index-1]& 0xFFFF);
}


uint8_t AnalogDigitalConverter::get_current_channel_count() const
{
  return this->current_channels;
}
