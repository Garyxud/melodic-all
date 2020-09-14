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

#include "fsrobo_r_driver/simple_message/wrench.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"

using namespace industrial::shared_types;

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace wrench
{

Wrench::Wrench(void)
{
  this->init();
}

Wrench::~Wrench(void)
{

}

void Wrench::init()
{
  this->setForce(0, 0);
  this->setForce(1, 0);
  this->setForce(2, 0);
  this->setTorque(0, 0);
  this->setTorque(1, 0);
  this->setTorque(2, 0);
}

void Wrench::copyFrom(Wrench &src)
{
  this->setForce(0, src.getForce(0));
  this->setForce(1, src.getForce(1));
  this->setForce(2, src.getForce(2));
  this->setTorque(0, src.getTorque(0));
  this->setTorque(1, src.getTorque(1));
  this->setTorque(2, src.getTorque(2));
}

bool Wrench::operator==(Wrench &rhs)
{
  return this->force_[0] == rhs.force_[0] && this->force_[1] == rhs.force_[1] && this->force_[2] == rhs.force_[2]
      && this->torque_[0] == rhs.torque_[0] && this->torque_[1] == rhs.torque_[1] && this->torque_[2] == rhs.torque_[2];
}

bool Wrench::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing wrench state load");

  if (buffer->load(this->force_[0]) && buffer->load(this->force_[1]) && buffer->load(this->force_[2])
    && buffer->load(this->torque_[0]) && buffer->load(this->torque_[1]) && buffer->load(this->torque_[2]))
  {

    LOG_COMM("wrench state successfully loaded");
    rtn = true;
  }
  else
  {
    LOG_COMM("wrench state not loaded");
    rtn = false;
  }

  return rtn;
}

bool Wrench::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;

  LOG_COMM("Executing wrench state unload");
  if (buffer->unload(this->torque_[2]) && buffer->unload(this->torque_[1]) && buffer->unload(this->torque_[0])
    && buffer->unload(this->force_[2]) && buffer->unload(this->force_[1]) && buffer->unload(this->force_[0]))
  {
    rtn = true;
    LOG_COMM("wrench state successfully unloaded");
  }

  else
  {
    LOG_ERROR("Failed to unload wrench state");
    rtn = false;
  }

  return rtn;
}

} // namespace wrench
} // namespace simple_message
} // namespace fsrobo_r_driver
