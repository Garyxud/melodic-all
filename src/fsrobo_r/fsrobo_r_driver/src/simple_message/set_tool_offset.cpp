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

#include "fsrobo_r_driver/simple_message/set_tool_offset.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"

using industrial::shared_types::shared_real;

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace set_tool_offset
{

SetToolOffset::SetToolOffset(void)
{
  this->init();
}

SetToolOffset::~SetToolOffset(void)
{
}

void SetToolOffset::init()
{
  this->init(0, 0, 0, 0, 0, 0);
}

void SetToolOffset::init(shared_real x, shared_real y, shared_real z, shared_real rz, shared_real ry, shared_real rx)
{
  this->setX(x);
  this->setY(y);
  this->setZ(z);
  this->setRz(rz);
  this->setRy(ry);
  this->setRx(rx);
}

void SetToolOffset::copyFrom(SetToolOffset &src)
{
  this->setX(src.getX());
  this->setY(src.getY());
  this->setZ(src.getZ());
  this->setRz(src.getRz());
  this->setRy(src.getRy());
  this->setRx(src.getRx());
}

bool SetToolOffset::operator==(SetToolOffset &rhs)
{
  bool rslt = this->x_ == rhs.x_ &&
              this->y_ == rhs.y_ &&
              this->z_ == rhs.z_ &&
              this->rz_ == rhs.rz_ &&
              this->ry_ == rhs.ry_ &&
              this->rx_ == rhs.rx_;

  return rslt;
}

bool SetToolOffset::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SetToolOffset command load");

  if (!buffer->load(this->x_))
  {
    LOG_ERROR("Failed to load SetToolOffset x");
    return false;
  }

  if (!buffer->load(this->y_))
  {
    LOG_ERROR("Failed to load SetToolOffset y");
    return false;
  }

  if (!buffer->load(this->z_))
  {
    LOG_ERROR("Failed to load SetToolOffset z");
    return false;
  }

  if (!buffer->load(this->rz_))
  {
    LOG_ERROR("Failed to load SetToolOffset rz");
    return false;
  }

  if (!buffer->load(this->ry_))
  {
    LOG_ERROR("Failed to load SetToolOffset ry");
    return false;
  }

  if (!buffer->load(this->rx_))
  {
    LOG_ERROR("Failed to load SetToolOffset rx");
    return false;
  }

  LOG_COMM("SetToolOffset data successfully loaded");
  return true;
}

bool SetToolOffset::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SetToolOffset command unload");

  if (!buffer->unload(this->x_)) {
    LOG_ERROR("Failed to unload SetToolOffset x");
    return false;
  }

  if (!buffer->unload(this->y_)) {
    LOG_ERROR("Failed to unload SetToolOffset y");
    return false;
  }

  if (!buffer->unload(this->z_)) {
    LOG_ERROR("Failed to unload SetToolOffset z");
    return false;
  }

  if (!buffer->unload(this->rz_)) {
    LOG_ERROR("Failed to unload SetToolOffset rz");
    return false;
  }

  if (!buffer->unload(this->ry_)) {
    LOG_ERROR("Failed to unload SetToolOffset ry");
    return false;
  }

  if (!buffer->unload(this->rx_)) {
    LOG_ERROR("Failed to unload SetToolOffset rx");
    return false;
  }

  LOG_COMM("SetToolOffset data successfully unloaded");
  return true;
}

}
}
}
