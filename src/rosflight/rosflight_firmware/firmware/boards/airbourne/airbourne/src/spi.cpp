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

#include "spi.h"

SPI* SPI1ptr;
SPI* SPI2ptr;
SPI* SPI3ptr;

static uint8_t dummy_buffer[256];

void SPI::init(const spi_hardware_struct_t *c)
{
  SPI_InitTypeDef  spi_init_struct;

  c_ = c;

  if (c_->dev == SPI1)
    SPI1ptr = this;
  else if (c_->dev == SPI2)
    SPI2ptr = this;
  else if (c_->dev == SPI3)
    SPI3ptr = this;


  // Set the AF configuration for the other pins
  GPIO_PinAFConfig(c_->GPIO, c_->SCK_PinSource, c_->GPIO_AF);
  GPIO_PinAFConfig(c_->GPIO, c_->MOSI_PinSource, c_->GPIO_AF);
  GPIO_PinAFConfig(c_->GPIO, c_->MISO_PinSource, c_->GPIO_AF);

  // Initialize other pins
  sck_.init(c_->GPIO, c_->SCK_Pin, GPIO::PERIPH_OUT);
  miso_.init(c_->GPIO, c_->MOSI_Pin, GPIO::PERIPH_OUT);
  mosi_.init(c_->GPIO, c_->MISO_Pin, GPIO::PERIPH_OUT);

  // Set up the SPI peripheral
  SPI_I2S_DeInit(c_->dev);
  spi_init_struct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi_init_struct.SPI_Mode = SPI_Mode_Master;
  spi_init_struct.SPI_DataSize = SPI_DataSize_8b;
  spi_init_struct.SPI_CPOL = SPI_CPOL_High;
  spi_init_struct.SPI_CPHA = SPI_CPHA_2Edge;
  spi_init_struct.SPI_NSS = SPI_NSS_Soft;
  spi_init_struct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  // 42/64 = 0.65625 MHz SPI Clock
  spi_init_struct.SPI_FirstBit = SPI_FirstBit_MSB;
  spi_init_struct.SPI_CRCPolynomial = 7;
  SPI_Init(c_->dev, &spi_init_struct);
  SPI_CalculateCRC(c_->dev, DISABLE);
  SPI_Cmd(c_->dev, ENABLE);

  // Wait for any transfers to clear (this should be really short if at all)
  while (SPI_I2S_GetFlagStatus(c_->dev, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_ReceiveData(c_->dev); //dummy read if needed

  // Configure the DMA
  DMA_InitStructure_.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure_.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure_.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure_.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure_.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure_.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure_.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure_.DMA_Channel = c_->DMA_Channel;
  DMA_InitStructure_.DMA_PeripheralBaseAddr = reinterpret_cast<uint32_t>(&(c_->dev->DR));
  DMA_InitStructure_.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure_.DMA_Priority = DMA_Priority_High;

  // Configure the Appropriate Interrupt Routine
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = c_->DMA_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_Init(&NVIC_InitStruct);

  transfer_cb_ = NULL;
}

void SPI::set_divisor(uint16_t new_divisor) {
  SPI_Cmd(c_->dev, DISABLE);

  const uint16_t clearBRP = 0xFFC7;

  uint16_t temp = c_->dev->CR1;

  temp &= clearBRP;
  switch(new_divisor) {
  case 2:
    temp |= SPI_BaudRatePrescaler_2;
    break;
  case 4:
    temp |= SPI_BaudRatePrescaler_4;
    break;
  case 8:
    temp |= SPI_BaudRatePrescaler_8;
    break;
  case 16:
    temp |= SPI_BaudRatePrescaler_16;
    break;
  case 32:
  default:
    temp |= SPI_BaudRatePrescaler_32;
    break;
  case 64:
    temp |= SPI_BaudRatePrescaler_64;
    break;
  case 128:
    temp |= SPI_BaudRatePrescaler_128;
    break;
  case 256:
    temp |= SPI_BaudRatePrescaler_256;
    break;
  }
  c_->dev->CR1 = temp;
  SPI_Cmd(c_->dev, ENABLE);
}

void SPI::enable(GPIO& cs) {
  cs.write(GPIO::LOW);
}

void SPI::disable(GPIO& cs) {
  cs.write(GPIO::HIGH);
}

uint8_t SPI::transfer_byte(uint8_t data, GPIO *cs)
{
  uint16_t spiTimeout;

  spiTimeout = 0x1000;

  if (cs != NULL)
    enable(*cs);

  while (SPI_I2S_GetFlagStatus(c_->dev, SPI_I2S_FLAG_TXE) == RESET)
  {
    if ((--spiTimeout) == 0)
      return false;
  }

  SPI_I2S_SendData(c_->dev, data);

  spiTimeout = 0x1000;

  while (SPI_I2S_GetFlagStatus(c_->dev, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if ((--spiTimeout) == 0)
      return false;
  }

  if (cs)
    disable(*cs);

  return static_cast<uint8_t>(SPI_I2S_ReceiveData(c_->dev));
}

bool SPI::write(const uint8_t *out_data, uint32_t num_bytes, GPIO* cs)
{
	busy_ = true;
  
	// Save Job parameters
	in_buffer_ptr_ = dummy_buffer;
	out_buffer_ptr_ = (out_data == NULL) ? dummy_buffer : out_data;
	cs_ = cs;
	transfer_cb_ = NULL;  
	num_bytes_ = num_bytes;
  
	perform_transfer();
	return true;
	
}

bool SPI::transfer(uint8_t *out_data, uint32_t num_bytes, uint8_t* in_data, GPIO* cs, void (*cb)(void))
{
  busy_ = true;

  // Save Job parameters
  in_buffer_ptr_ = (in_data == NULL) ? dummy_buffer : in_data;
  out_buffer_ptr_ = (out_data == NULL) ? dummy_buffer : out_data;
  cs_ = cs;
  transfer_cb_ = cb;  
  num_bytes_ = num_bytes;

  perform_transfer();
  return true;
}

void SPI::perform_transfer()
{
	// Configure the DMA
	DMA_DeInit(c_->Tx_DMA_Stream); //SPI1_TX_DMA_STREAM
	DMA_DeInit(c_->Rx_DMA_Stream); //SPI1_RX_DMA_STREAM
  
	DMA_InitStructure_.DMA_BufferSize = num_bytes_;
  
	// Configure Tx DMA
	DMA_InitStructure_.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure_.DMA_Memory0BaseAddr = reinterpret_cast<uint32_t>(out_buffer_ptr_);
	DMA_Init(c_->Tx_DMA_Stream, &DMA_InitStructure_);
  
	// Configure Rx DMA
	DMA_InitStructure_.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure_.DMA_Memory0BaseAddr = reinterpret_cast<uint32_t>(in_buffer_ptr_);
	DMA_Init(c_->Rx_DMA_Stream, &DMA_InitStructure_);
  
	//  Configure the Interrupt
	DMA_ITConfig(c_->Tx_DMA_Stream, DMA_IT_TC, ENABLE);
  
	if (cs_ != NULL)
	  enable(*cs_);
  
	// Turn on the DMA streams
	DMA_Cmd(c_->Tx_DMA_Stream, ENABLE);
	DMA_Cmd(c_->Rx_DMA_Stream, ENABLE);
  
	// Enable the SPI Rx/Tx DMA request
	SPI_I2S_DMACmd(c_->dev, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(c_->dev, SPI_I2S_DMAReq_Tx, ENABLE);
}



void SPI::transfer_complete_cb()
{
//  uint8_t rxne, txe;
//  do
//  {
//    rxne = SPI_I2S_GetFlagStatus(c_->dev, SPI_I2S_FLAG_RXNE);
//    txe =  SPI_I2S_GetFlagStatus(c_->dev, SPI_I2S_FLAG_TXE);
//  }while (rxne == RESET || txe == RESET);

  disable(*cs_);
  DMA_ClearFlag(c_->Tx_DMA_Stream, c_->Tx_DMA_TCIF);
  DMA_ClearFlag(c_->Rx_DMA_Stream, c_->Rx_DMA_TCIF);

  DMA_Cmd(c_->Tx_DMA_Stream, DISABLE);
  DMA_Cmd(c_->Rx_DMA_Stream, DISABLE);

  SPI_I2S_DMACmd(c_->dev, SPI_I2S_DMAReq_Rx, DISABLE);
  SPI_I2S_DMACmd(c_->dev, SPI_I2S_DMAReq_Tx, DISABLE);

//  SPI_Cmd(c_->dev, DISABLE);

  if (cs_ != NULL)
  {
    disable(*cs_);
  }

  busy_ = false;
  if (transfer_cb_ != NULL)
    transfer_cb_();
}

extern "C"
{

void DMA2_Stream3_IRQHandler()
{
  if (DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3))
  {
    DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3);
    SPI1ptr->transfer_complete_cb();
  }
}

void DMA1_Stream4_IRQHandler()
{
  if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))
  {
    DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
    SPI2ptr->transfer_complete_cb();
  }
}

void DMA1_Stream5_IRQHandler()
{
  if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
  {
    DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
    SPI3ptr->transfer_complete_cb();
  }
}

}
