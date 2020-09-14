/*********************************************************************
* FSRobo-R Package BSDL
* ---------
* Copyright (C) 2019 FUJISOFT. All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "fsrobo_r_driver/simple_message/io_state.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::shared_types;

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace io_state
{

IOState::IOState(void)
{
  this->init();
}

IOState::~IOState(void)
{

}

void IOState::init()
{
  this->setDigital(0, 0);
  this->setDigital(1, 0);
  this->setAnalog(0, 0);
}

void IOState::copyFrom(IOState &src)
{
  this->setDigital(0, src.getDigital(0));
  this->setDigital(1, src.getDigital(1));
  this->setAnalog(0, src.getAnalog(0));
}

bool IOState::operator==(IOState &rhs)
{
  return this->digital_[0] == rhs.digital_[0] && this->digital_[1] == rhs.digital_[1]
      && this->analog_[0] == rhs.analog_[0];
}

bool IOState::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing I/O state load");

  if (buffer->load(this->digital_[0]) && buffer->load(this->digital_[1]) && buffer->load(this->analog_[0]))
  {

    LOG_COMM("I/O state successfully loaded");
    rtn = true;
  }
  else
  {
    LOG_COMM("I/O state not loaded");
    rtn = false;
  }

  return rtn;
}

bool IOState::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing I/O state unload");
  if (buffer->unload(this->analog_[0]) && buffer->unload(this->digital_[1]) && buffer->unload(this->digital_[0]))
  {
    rtn = true;
    LOG_COMM("I/O state successfully unloaded");
  }

  else
  {
    LOG_ERROR("Failed to unload I/O state");
    rtn = false;
  }

  return rtn;
}

}
}
}
