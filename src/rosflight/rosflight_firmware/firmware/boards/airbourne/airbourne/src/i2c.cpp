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

#include "i2c.h"

#define while_check(cond, result) \
{\
  int32_t timeout_var = 200; \
  while ((cond) && timeout_var) \
  timeout_var--; \
  if (!timeout_var) \
{ \
  handle_hardware_failure();\
  result = RESULT_ERROR; \
  }\
  }

#define log_line event_history_.add_event(__LINE__)

//global i2c ptrs used by the event interrupts
I2C* I2C1_Ptr;
I2C* I2C2_Ptr;
I2C* I2C3_Ptr;

void I2C::init(const i2c_hardware_struct_t *c)
{
  c_ = c;
  
  if (c->dev == I2C1)
    I2C1_Ptr = this;
  
  if (c->dev == I2C2)
    I2C2_Ptr = this;
  
  if (c->dev == I2C3)
    I2C3_Ptr = this;
  
  GPIO_PinAFConfig(c->GPIO, c->SCL_PinSource, c->GPIO_AF);
  GPIO_PinAFConfig(c->GPIO, c->SDA_PinSource, c->GPIO_AF);
  scl_.init(c->GPIO, c->SCL_Pin, GPIO::PERIPH_IN_OUT);
  sda_.init(c->GPIO, c->SDA_Pin, GPIO::PERIPH_IN_OUT);
  
  //initialize the i2c itself
  I2C_DeInit(c->dev);
  
  I2C_InitTypeDef I2C_InitStructure;
  I2C_StructInit(&I2C_InitStructure);
  I2C_InitStructure.I2C_ClockSpeed = c->I2C_ClockSpeed;
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0; //The first device address
  I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(c->dev, &I2C_InitStructure);
  
  // Interrupts
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannel = c->I2C_EV_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  
  // I2C Event Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = c->I2C_ER_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  
  // DMA Event Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = c->DMA_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  
  DMA_Cmd(c->DMA_Stream, DISABLE);
  DMA_DeInit(c->DMA_Stream);
  DMA_InitStructure_.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_InitStructure_.DMA_FIFOThreshold = DMA_FIFOThreshold_Full ;
  DMA_InitStructure_.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure_.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure_.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure_.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure_.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure_.DMA_Channel = c->DMA_Channel;
  
  DMA_InitStructure_.DMA_PeripheralBaseAddr = reinterpret_cast<uint32_t>(&(c->dev->DR));
  DMA_InitStructure_.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure_.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure_.DMA_DIR = DMA_DIR_PeripheralToMemory;
  
  last_event_us_ = micros();
  I2C_Cmd(c->dev, ENABLE);
  
  unstick(); //unsti1ck will properly initialize pins
  log_line;
}

void I2C::unstick()
{
  I2C_Cmd(c_->dev, DISABLE);
  
  I2C_ClearFlag(c_->dev, I2C_FLAG_BUSY);
  
  // Turn off the interrupts
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
  
  //reset errors
  I2C_ClearFlag(c_->dev, I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);
  
  scl_.set_mode(GPIO::OUTPUT);
  sda_.set_mode(GPIO::OUTPUT);
  
  scl_.write(GPIO::HIGH);
  sda_.write(GPIO::HIGH);
  
  delayMicroseconds(100);
  
  // clock out some bits
  for (int i = 0; i < 16; ++i)
  {
    delayMicroseconds(1);
    scl_.toggle();
  }
  delayMicroseconds(1);
  
  // send a start condition
  sda_.write(GPIO::LOW);
  delayMicroseconds(1);
  scl_.write(GPIO::LOW);
  delayMicroseconds(1);
  
  // then a stop
  scl_.write(GPIO::HIGH);
  delayMicroseconds(1);
  sda_.write(GPIO::HIGH);
  delayMicroseconds(1);
  
  // turn things back on
  scl_.set_mode(GPIO::PERIPH_IN_OUT);
  sda_.set_mode(GPIO::PERIPH_IN_OUT);
  I2C_Cmd(c_->dev, ENABLE);
  
  current_status_ = IDLE;  
  write(0, 0, 0);
  
  last_event_us_ = micros();
  log_line;
  
}


int8_t I2C::read(uint8_t addr, uint8_t reg, uint8_t num_bytes, uint8_t* data, void(*callback)(uint8_t), bool blocking)
{
  if (check_busy())
    return RESULT_BUSY;
  log_line;
  
  // configure the job
  current_status_ = READING;
  addr_ = addr << 1;
  cb_ = callback;
  reg_ = reg;
  subaddress_sent_ = (reg_ == 0xFF);
  len_ = num_bytes;
  done_ = false;
  return_code_ = RESULT_SUCCESS;
  
  DMA_DeInit(c_->DMA_Stream);
  DMA_InitStructure_.DMA_BufferSize = static_cast<uint16_t>(len_);
  DMA_InitStructure_.DMA_Memory0BaseAddr = reinterpret_cast<uint32_t>(data);
  DMA_Init(c_->DMA_Stream, &DMA_InitStructure_);
  
  I2C_Cmd(c_->dev, ENABLE);
  
  while_check (I2C_GetFlagStatus(c_->dev, I2C_FLAG_BUSY), return_code_);
  
  // If we don't need to send the subaddress, then go ahead and spool up the DMA NACK
  if (subaddress_sent_)
  {
    I2C_AcknowledgeConfig(c_->dev, ENABLE);
    I2C_DMALastTransferCmd(c_->dev, ENABLE);
  }
  I2C_GenerateSTART(c_->dev, ENABLE);
  
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
  
  last_event_us_ = micros();
  if (blocking)
  {
    log_line;
    while(check_busy());
  }
  log_line;
  last_event_us_ = micros();
  
  return return_code_;
}


void I2C::transfer_complete_cb()
{
  current_status_ = IDLE;
  if (cb_)
    cb_(return_code_);
  log_line;
}


// blocking, single register read (for configuring devices)
int8_t I2C::read(uint8_t addr, uint8_t reg, uint8_t *data)
{
  if (check_busy())
    return RESULT_BUSY;
  log_line;
  return_code_ = RESULT_SUCCESS;
  
  // Turn off interrupts while blocking
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
  
  while_check (I2C_GetFlagStatus(c_->dev, I2C_FLAG_BUSY), return_code_);
  
  I2C_Cmd(c_->dev, ENABLE);
  if (reg != 0xFF)
  {
    log_line;
    I2C_GenerateSTART(c_->dev, ENABLE);
    while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_MODE_SELECT), return_code_);
    I2C_Send7bitAddress(c_->dev, addr << 1, I2C_Direction_Transmitter);
    uint32_t timeout = 500;
    while (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --timeout != 0);
    if (timeout != 0)
    {
      I2C_GenerateSTOP(c_->dev, ENABLE);
      I2C_Cmd(c_->dev, DISABLE);
    }
    else
    {
      return_code_ = RESULT_ERROR;
      log_line;
      return return_code_;
    }
    I2C_Cmd(c_->dev, ENABLE);
    I2C_SendData(c_->dev, reg);
    while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_BYTE_TRANSMITTED), return_code_);
  }
  
  // Read the byte
  I2C_AcknowledgeConfig(c_->dev, DISABLE);
  I2C_GenerateSTART(c_->dev, ENABLE);
  while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_MODE_SELECT), return_code_);
  I2C_Cmd(c_->dev, ENABLE);
  I2C_Send7bitAddress(c_->dev, addr << 1, I2C_Direction_Receiver);
  uint32_t timeout = 500;
  while (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_BYTE_RECEIVED) && --timeout != 0);
  if (timeout != 0)
  {
    *data = I2C_ReceiveData(c_->dev);
  }
  else
  {
    return_code_ = RESULT_ERROR;
  }
  I2C_GenerateSTOP(c_->dev, ENABLE);
  I2C_Cmd(c_->dev, DISABLE);
  log_line;
  
  return return_code_;
}

// asynchronous write, for commanding adc conversions
int8_t I2C::write(uint8_t addr, uint8_t reg, uint8_t data, void(*callback)(uint8_t), bool blocking)
{
  if (check_busy())
    return RESULT_BUSY;
  
  log_line;
  current_status_ = WRITING;
  addr_ = addr << 1;
  cb_ = callback;
  reg_ = reg;
  subaddress_sent_ = (reg_ == 0xFF);
  len_ = 1;
  done_ = false;
  data_ = data;
  return_code_ = RESULT_SUCCESS;
  
  I2C_Cmd(c_->dev, ENABLE);
  
  
  while_check (I2C_GetFlagStatus(c_->dev, I2C_FLAG_BUSY), return_code_);
  
  I2C_GenerateSTART(c_->dev, ENABLE);
  
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
  
  last_event_us_ = micros();
  if (blocking)
  {
    log_line;
    while (check_busy());
  }
  log_line;
  return return_code_;
}

// blocking, single register write (for configuring devices)
int8_t I2C::write(uint8_t addr, uint8_t reg, uint8_t data)
{
  if (check_busy())
    return RESULT_BUSY;
  
  log_line;
  return_code_ = RESULT_SUCCESS;
  while_check (I2C_GetFlagStatus(c_->dev, I2C_FLAG_BUSY), return_code_);
  
  // Turn off interrupts for blocking write
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
  I2C_Cmd(c_->dev, ENABLE);
  
  // start the transfer
  I2C_GenerateSTART(c_->dev, ENABLE);
  while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_MODE_SELECT), return_code_);
  I2C_Send7bitAddress(c_->dev, addr << 1, I2C_Direction_Transmitter);
  uint32_t timeout = 500;
  while (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && !(I2C_GetLastEvent(c_->dev) & AF) && timeout--);
  
  // No acknowledgement or timeout
  if (I2C_GetLastEvent(c_->dev) & AF || timeout == 0) 
  {
    log_line;
    I2C_GenerateSTOP(c_->dev, ENABLE);
    I2C_Cmd(c_->dev, DISABLE);
    return RESULT_ERROR;
  }
  
  // Send the register
  if (reg != 0xFF)
  {
    log_line;
    I2C_SendData(c_->dev, reg);
    while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_BYTE_TRANSMITTED), return_code_);
  }
  
  // Write the byte with a NACK
  I2C_AcknowledgeConfig(c_->dev, DISABLE);
  I2C_SendData(c_->dev, data);
  while_check (!I2C_CheckEvent(c_->dev, I2C_EVENT_MASTER_BYTE_TRANSMITTED), return_code_);
  I2C_GenerateSTOP(c_->dev, ENABLE  );
  I2C_Cmd(c_->dev, DISABLE);
  log_line;
  last_event_us_ = micros();
  return return_code_;
  
}

// if for some reason, a step in an I2C read or write fails, call this
void I2C::handle_hardware_failure() {
  error_count_++;
  return_code_ = RESULT_ERROR;
  log_line;
//  unstick(); //unstick and reinitialize the hardware
}


// This is the I2C_IT_ERR handler
void I2C::handle_error()
{
  log_line;
  I2C_Cmd(c_->dev, DISABLE);
  return_code_ = RESULT_ERROR;
  while_check (I2C_GetFlagStatus(c_->dev, I2C_FLAG_BUSY), return_code_);
  
  // Turn off the interrupts
  I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
  
  //reset errors
  I2C_ClearFlag(c_->dev, I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);
  current_status_ = IDLE;
  log_line;
  transfer_complete_cb();
}

// This is the I2C_IT_EV handler
void I2C::handle_event()
{
  uint32_t last_event = I2C_GetLastEvent(c_->dev);
  interrupt_history_.add_event(c_->dev->SR2 << 16 | c_->dev->SR1);
  // We just sent a byte
  if ((last_event & I2C_EVENT_MASTER_BYTE_TRANSMITTED) == I2C_EVENT_MASTER_BYTE_TRANSMITTED)
  {
    last_event_us_ = micros();
    // If we are reading, then we just sent a subaddress and need to send
    // a repeated start, and enable the DMA NACK
    if (current_status_ == READING)
    {
      log_line;
      I2C_AcknowledgeConfig(c_->dev, ENABLE);
      I2C_DMALastTransferCmd(c_->dev, ENABLE);
      I2C_GenerateSTART(c_->dev, ENABLE);
    }
    // We are in write mode and are done, need to clean up
    else
    {
      log_line;
      I2C_GenerateSTOP(c_->dev, ENABLE);
      I2C_ITConfig(c_->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
      transfer_complete_cb();
    }
  }
  
  // We just sent the address in write mode
  else if ((last_event & I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
  {
    last_event_us_ = micros();
    // We need to send the subaddress
    if (!subaddress_sent_)
    {
      log_line;
      I2C_SendData(c_->dev, reg_);
      subaddress_sent_ = true;
      if (current_status_ == WRITING)
      {
        log_line;
        I2C_SendData(c_->dev, data_);
        done_ = true;
      }
    }
    // We need to send our data (no subaddress)
    else
    {
      log_line;
      I2C_SendData(c_->dev, data_);
      done_ = true;
    }
  }
  
  // We are in receiving mode, preparing to receive the big DMA dump
  else if ((last_event & I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
  {
    last_event_us_ = micros();
    log_line;
    //    I2C_ITConfig(c_->dev, I2C_IT_EVT, DISABLE);
    DMA_SetCurrDataCounter(c_->DMA_Stream, len_);
    I2C_DMACmd(c_->dev, ENABLE);
    DMA_ITConfig(c_->DMA_Stream, DMA_IT_TC, ENABLE);
    DMA_Cmd(c_->DMA_Stream, ENABLE);
  }
  
  // Start just sent
  else if ((last_event & I2C_EVENT_MASTER_MODE_SELECT) == I2C_EVENT_MASTER_MODE_SELECT)
  {
    last_event_us_ = micros();
    // we either don't need to send, or already sent the subaddress
    if (subaddress_sent_ && current_status_ == READING)
    {
      log_line;
      // Set up a receive
      I2C_Send7bitAddress(c_->dev, addr_, I2C_Direction_Receiver);
    }
    // We need to either send the subaddress or our datas
    else
    {
      log_line;
      // Set up a write
      I2C_Send7bitAddress(c_->dev, addr_, I2C_Direction_Transmitter);
    }
  }
  
  // Sometimes we just get this over and over and over again
  else if (last_event == SB)
  {
    // SB is cleared by clearing and resetting PE
    I2C_Cmd(c_->dev, DISABLE);
    I2C_Cmd(c_->dev, ENABLE);
    error_count_++;
  }
}


// This checks to make sure that the I2C device isn't currently handling an i2c job, and if
// it is, that it hasn't stalled out
bool I2C::check_busy()
{
  if (current_status_ == IDLE)
    return false;
  else
  {
    // If we haven't seen anything in a long while, then restart the device
    if (micros() > last_event_us_ + 2000)
    {
      error_count_++;
      // Send a stop condition
      I2C_GenerateSTOP(c_->dev, ENABLE);
      return_code_ = RESULT_SUCCESS;
      while_check (c_->dev->SR2 & BUSY, return_code_)
          
      // Force reset of the bus
      // This is really slow, but it seems to be the only
      // way to regain a connection if bad things happen
      if (return_code_ == RESULT_ERROR)
      {
        I2C_Cmd(c_->dev, DISABLE);
        scl_.set_mode(GPIO::OUTPUT);
        sda_.set_mode(GPIO::OUTPUT);
        
        // Write Pins low
        scl_.write(GPIO::LOW);
        sda_.write(GPIO::LOW);
        delayMicroseconds(1);
        
        // Send a stop
        scl_.write(GPIO::HIGH);
        delayMicroseconds(1);
        sda_.write(GPIO::HIGH);
        delayMicroseconds(1);
        
        // turn things back on
        scl_.set_mode(GPIO::PERIPH_IN_OUT);
        sda_.set_mode(GPIO::PERIPH_IN_OUT);
        I2C_Cmd(c_->dev, ENABLE);
        
        current_status_ = IDLE;  
      }
      log_line;
      return false;    
    }
    else
    {
      return true;
    }
  }
}

extern "C"
{

// C-based IRQ functions (defined in the startup script)
void DMA1_Stream2_IRQHandler(void)
{
  
  if (DMA_GetFlagStatus(DMA1_Stream2, DMA_FLAG_TCIF2))
  {
    /* Clear transmission complete flag */
    DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
    
    I2C_DMACmd(I2C2, DISABLE);
    /* Send I2C1 STOP Condition */
    I2C_GenerateSTOP(I2C2, ENABLE);
    /* Disable DMA channel*/
    DMA_Cmd(DMA1_Stream2, DISABLE);
    
    /* Turn off I2C interrupts since we are done with the transfer */
    I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    
    I2C2_Ptr->transfer_complete_cb(); // TODO make this configurable
  }
}

void DMA1_Stream0_IRQHandler(void)
{
  if (DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0))
  {
    /* Clear transmission complete flag */
    DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
    
    I2C_DMACmd(I2C1, DISABLE);
    /* Send I2C1 STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);
    /* Disable DMA channel*/
    DMA_Cmd(DMA1_Stream0, DISABLE);
    /* Turn off I2C interrupts, because we are done with the transfer */
    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    
    I2C1_Ptr->transfer_complete_cb(); // TODO make this configurable
  }
}

void I2C1_ER_IRQHandler(void) {
  I2C1_Ptr->handle_error();
}

void I2C1_EV_IRQHandler(void) {
  I2C1_Ptr->handle_event();
}

void I2C2_ER_IRQHandler(void) {
  I2C2_Ptr->handle_error();
}

void I2C2_EV_IRQHandler(void) {
  I2C2_Ptr->handle_event();
}

void I2C3_ER_IRQHandler(void) {
  I2C3_Ptr->handle_error();
}

void I2C3_EV_IRQHandler(void) {
  I2C3_Ptr->handle_event();
}

}
