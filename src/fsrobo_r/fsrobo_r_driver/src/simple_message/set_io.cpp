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

#include "fsrobo_r_driver/simple_message/set_io.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#include <vector>

using industrial::shared_types::shared_int;
using std::vector;

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace io_control
{

SetIO::SetIO(void)
{
  this->init();
}

SetIO::~SetIO(void)
{
}

void SetIO::init()
{
  vector<shared_int> data(32, 0);

  // TODO: is '0' a good initial value?
  this->init(0, 0, data);
}

void SetIO::init(shared_int fun, shared_int address, vector<shared_int> &data)
{
  this->setFun(fun);
  this->setAddress(address);

  std::size_t size = data.size();
  this->setDataSize(size);

  int i = 0;
  for (auto &x : this->data_)
  {
    if (i < size) {
      x = data[i];
    }
    i++;
  }
}

void SetIO::copyFrom(SetIO &src)
{
  this->setFun(src.getFun());
  this->setAddress(src.getAddress());
  this->setDataSize(src.getDataSize());

  shared_int *data = src.getData();
  for (int i = 0; i < 32; i++)
  {
    this->data_[i] = data[i];
  }
}

bool SetIO::operator==(SetIO &rhs)
{
  bool rslt = this->fun_ == rhs.fun_ &&
              this->address_ == rhs.address_ &&
              this->data_ == rhs.data_;

  return rslt;
}

bool SetIO::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SetIO command load");

  if (!buffer->load(this->fun_))
  {
    LOG_ERROR("Failed to load SetIO fun");
    return false;
  }

  if (!buffer->load(this->address_))
  {
    LOG_ERROR("Failed to load SetIO address");
    return false;
  }

  if (!buffer->load(this->data_size_))
  {
    LOG_ERROR("Failed to load SetIO data size");
    return false;
  }

  for (auto x : this->data_)
  {
    if (!buffer->load(x))
    {
      LOG_ERROR("Failed to load SetIO data");
      return false;
    }
  }

  LOG_COMM("SetIO data successfully loaded");
  return true;
}

bool SetIO::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SetIO command unload");

  for (auto &x : this->data_)
  {
    if (!buffer->unload(x))
    {
      LOG_ERROR("Failed to unload SetIO data");
      return false;
    }
  }

  if (!buffer->unload(this->data_size_))
  {
    LOG_ERROR("Failed to unload SetIO data size");
    return false;
  }

  if (!buffer->unload(this->address_))
  {
    LOG_ERROR("Failed to unload SetIO address");
    return false;
  }

  if (!buffer->unload(this->fun_))
  {
    LOG_ERROR("Failed to unload SetIO fun");
    return false;
  }

  LOG_COMM("SetIO data successfully unloaded");
  return true;
}

}
}
}
