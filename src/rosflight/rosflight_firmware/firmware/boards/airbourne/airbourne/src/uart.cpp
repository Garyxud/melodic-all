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

#include "uart.h"

UART* UART1Ptr = NULL;
UART* UART3Ptr = NULL;

UART::UART()
{}


void UART::init(const uart_hardware_struct_t* conf, uint32_t baudrate, uart_mode_t mode)
{
  receive_CB_ = nullptr;
  c_ = conf;//Save the configuration

  //initialize pins
  rx_pin_.init(c_->GPIO, c_->Rx_Pin, GPIO::PERIPH_IN_OUT);
  tx_pin_.init(c_->GPIO, c_->Tx_Pin, GPIO::PERIPH_OUT);
  GPIO_PinAFConfig(c_->GPIO, c_->Rx_PinSource, c_->GPIO_AF);
  GPIO_PinAFConfig(c_->GPIO, c_->Tx_PinSource, c_->GPIO_AF);


  //Save the pointer, for callbacks
  if (c_->dev == USART1)
  {
    UART1Ptr = this;
  }
  if(c_->dev == USART3)
  {
    UART3Ptr = this;
  }

  init_UART(baudrate, mode);
  init_DMA();
  init_NVIC();
}

void UART::init_UART(uint32_t baudrate, uart_mode_t mode)
{
  // Configure the device
  USART_Cmd(c_->dev, DISABLE);//Disable the usart device for configuration

  USART_InitTypeDef USART_InitStruct;
  USART_InitStruct.USART_BaudRate = baudrate;

  switch (mode)
  {
  case MODE_8N1:
  default:
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    break;
  case MODE_8E2:
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_Parity = USART_Parity_Even;
    USART_InitStruct.USART_StopBits = USART_StopBits_2;
    break;
  }

  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//Set to both receive and send
  USART_Init(c_->dev, &USART_InitStruct);
  //USART_OverSampling8Cmd(c_->dev, ENABLE);//Please don't break anything

  // Throw interrupts on byte receive
  USART_ITConfig(c_->dev, USART_IT_RXNE, ENABLE);//enable interupts on receive
  USART_Cmd(c_->dev, ENABLE);//reenable the usart
}

void UART::init_DMA()
{
  DMA_InitTypeDef DMA_InitStructure;

  // Common DMA Configuration
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralBaseAddr = reinterpret_cast<uint32_t>(&(c_->dev->DR));
  DMA_InitStructure.DMA_Channel = c_->DMA_Channel;

  // Configure the Tx DMA
  DMA_DeInit(c_->Tx_DMA_Stream);
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = TX_BUFFER_SIZE;
  DMA_InitStructure.DMA_Memory0BaseAddr = reinterpret_cast<uint32_t>(tx_buffer_);
  DMA_Init(c_->Tx_DMA_Stream, &DMA_InitStructure);

  // Configure the Rx DMA
  DMA_DeInit(c_->Rx_DMA_Stream);
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = RX_BUFFER_SIZE;
  DMA_InitStructure.DMA_Memory0BaseAddr = reinterpret_cast<uint32_t>(rx_buffer_);
  DMA_Init(c_->Rx_DMA_Stream, &DMA_InitStructure);

  // Turn on the Rx DMA Stream
  DMA_Cmd(c_->Rx_DMA_Stream, ENABLE);

  //  Hook up the DMA to the uart
  USART_DMACmd(c_->dev, USART_DMAReq_Tx, ENABLE);
  USART_DMACmd(c_->dev, USART_DMAReq_Rx, ENABLE);

  // Turn on the transfer complete interrupt source from the DMA
  DMA_ITConfig(c_->Tx_DMA_Stream, DMA_IT_TC, ENABLE);
  DMA_ITConfig(c_->Rx_DMA_Stream, DMA_IT_TC, ENABLE);

  // Initialize the Circular Buffers
  // set the buffer pointers to where the DMA is starting (starts at 256 and counts down)
  rx_buffer_tail_ = DMA_GetCurrDataCounter(c_->Rx_DMA_Stream);
  rx_buffer_head_ = rx_buffer_tail_;
  tx_buffer_head_ = 0;
  tx_buffer_tail_ = 0;

  memset(rx_buffer_, 0, RX_BUFFER_SIZE);
  memset(tx_buffer_, 0, TX_BUFFER_SIZE);
}


void UART::init_NVIC()
{
  // Configure the Interrupt
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = c_->USART_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = c_->Tx_DMA_IRQn;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = c_->Rx_DMA_IRQn;
  NVIC_Init(&NVIC_InitStruct);
}


void UART::write(const uint8_t *ch, uint8_t len)
{
  // Put Data on the tx_buffer
  for (int i = 0; i < len ; i++)
  {
    tx_buffer_[tx_buffer_head_] = ch[i];
    tx_buffer_head_ = (tx_buffer_head_ + 1) % TX_BUFFER_SIZE;
  }
  if (DMA_GetCmdStatus(c_->Tx_DMA_Stream) == DISABLE)
  {
    startDMA();
  }
//  this->flush();//testing
}


void UART::startDMA()
{
  // Set the start of the transmission to the oldest data
  c_->Tx_DMA_Stream->M0AR = reinterpret_cast<uint32_t>(&tx_buffer_[tx_buffer_tail_]);
  if(tx_buffer_head_ > tx_buffer_tail_)
  {
    // Set the length of the transmission to the data on the buffer
    // if contiguous, this is easy
    DMA_SetCurrDataCounter(c_->Tx_DMA_Stream, tx_buffer_head_ - tx_buffer_tail_);
    tx_buffer_tail_ = tx_buffer_head_;
  }
  else
  {
    // We will have to send the data in two groups, first the tail,
    // then the head we will do later
    DMA_SetCurrDataCounter(c_->Tx_DMA_Stream, TX_BUFFER_SIZE - tx_buffer_tail_);
    tx_buffer_tail_ = 0;
  }
  // Start the Transmission
  DMA_Cmd(c_->Tx_DMA_Stream, ENABLE);
}

uint8_t UART::read_byte()
{
  uint8_t byte = 0;
  // pull the next byte off the array
  // (the head counts down, because CNTR counts down)
  if(rx_buffer_head_ != rx_buffer_tail_)
  {
    // read a new byte and decrement the tail
    byte = rx_buffer_[RX_BUFFER_SIZE - rx_buffer_tail_];
    if(--rx_buffer_tail_ == 0)
    {
      // wrap to the top if at the bottom
      rx_buffer_tail_ = RX_BUFFER_SIZE;
    }
  }
  return byte;
}


void UART::put_byte(uint8_t ch)
{
  write(&ch, 1);
}

uint32_t UART::rx_bytes_waiting()
{
  // Remember, the DMA CNDTR counts down
  rx_buffer_head_ = DMA_GetCurrDataCounter(c_->Rx_DMA_Stream);
  if (rx_buffer_head_ < rx_buffer_tail_)
  {
    // Easy, becasue it's contiguous
    return rx_buffer_tail_ - rx_buffer_head_;
  }
  else if (rx_buffer_head_ > rx_buffer_tail_)
  {
    // Add the parts on either end of the buffer
    // I'm pretty sure this is wrong
    return rx_buffer_tail_ + RX_BUFFER_SIZE - rx_buffer_head_;
  }
  else
  {
    return 0;
  }
}

uint32_t UART::tx_bytes_free()
{
  // Remember, the DMA CNDTR counts down
  tx_buffer_head_ = DMA_GetCurrDataCounter(c_->Rx_DMA_Stream);
  if (tx_buffer_head_ >= tx_buffer_tail_)
  {
    return TX_BUFFER_SIZE - (tx_buffer_head_ - tx_buffer_tail_);
  }
  else
    return tx_buffer_tail_ - rx_buffer_head_;
}

bool UART::set_mode(uint32_t baud, uart_mode_t mode)
{
  init_UART(baud, mode);
  return true;
}

bool UART::tx_buffer_empty()
{
  return tx_buffer_head_ == tx_buffer_tail_;
}

bool UART::flush()
{
  uint32_t timeout = 100000;
  while (!tx_buffer_empty() && --timeout);
  if (timeout)
    return true;
  else
    return false;
}

void UART::DMA_Rx_IRQ_callback()
{
  // DMA took care of putting the data on the buffer
  // Just call the callback until we have no more data
  // Update the head position from the DMA
  rx_buffer_head_ =  DMA_GetCurrDataCounter(c_->Rx_DMA_Stream);
  if(receive_CB_ != nullptr)
  {
    while(rx_buffer_head_ != rx_buffer_tail_)
    {
      // read a new byte and decrement the tail
      uint8_t byte = rx_buffer_[RX_BUFFER_SIZE - rx_buffer_tail_];
      receive_CB_(byte);
      if(--rx_buffer_tail_ == 0)
      {
        // wrap to the top if at the bottom
        rx_buffer_tail_ = RX_BUFFER_SIZE;
      }
    }
  }
}

void UART::DMA_Tx_IRQ_callback()
{
  // If there is more data to be sent
  if(tx_buffer_head_ != tx_buffer_tail_)
  {
    startDMA();
  }
}

void UART::register_rx_callback(void (*cb)(uint8_t data) )
{
  receive_CB_ = cb;
}

void UART::unregister_rx_callback()
{
  receive_CB_ = nullptr;
}

extern "C"
{

void USART1_IRQHandler (void)
{
  UART1Ptr->DMA_Rx_IRQ_callback();
}
void USART3_IRQHandler (void)
{
  UART3Ptr->DMA_Rx_IRQ_callback();
}

void DMA2_Stream5_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))
  {
    DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
    UART1Ptr->DMA_Rx_IRQ_callback();
  }
}

void DMA2_Stream7_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7))
  {
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
    DMA_Cmd(DMA2_Stream7, DISABLE);
    UART1Ptr->DMA_Tx_IRQ_callback();
  }
}

void DMA1_Stream1_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
  {
    DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
    UART3Ptr->DMA_Rx_IRQ_callback();
  }
}

void DMA1_Stream3_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3))
  {
    DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
    DMA_Cmd(DMA1_Stream3, DISABLE);
    UART3Ptr->DMA_Tx_IRQ_callback();
  }
}

}
