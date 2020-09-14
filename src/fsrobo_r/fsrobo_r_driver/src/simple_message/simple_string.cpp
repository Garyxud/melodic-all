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

#include <string>
#include "fsrobo_r_driver/simple_message/simple_string.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"

using industrial::shared_types::shared_int;

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace simple_string
{

SimpleString::SimpleString(void)
{
  this->init();
}

SimpleString::~SimpleString(void)
{
}

void SimpleString::init()
{
  this->init("");
}

void SimpleString::init(string str)
{
  this->setString(str);
}

void SimpleString::copyFrom(SimpleString &src)
{
  this->setString(src.getString());
}

bool SimpleString::operator==(SimpleString &rhs)
{
  bool rslt = this->string_ == rhs.string_;

  return rslt;
}

bool SimpleString::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SimpleString load");

  shared_int size = this->getStringSize();

  char *str = new char[size + 1];
  str[size] = '\0';
  this->string_.copy(str, size);

  if (!buffer->load(str, size)) {
    LOG_ERROR("Failed to load SimpleString data");
    delete[] str;
    return false;
  }
  delete[] str;

  if (!buffer->load(size))
  {
    LOG_ERROR("Failed to load SimpleString size");
    return false;
  }

  LOG_COMM("SimpleString data successfully loaded");
  return true;
}

bool SimpleString::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing SimpleString unload");

  shared_int size;

  //if (!buffer->unload(this->result_code_))
  if (!buffer->unload(size))
  {
    LOG_ERROR("Failed to unload SimpleString size");
    return false;
  }

  char *str = new char[size + 1];
  str[size] = '\0';
  if (!buffer->unload(str, size)) {
    LOG_ERROR("Failed to unload SimpleString size");
    delete[] str;
    return false;
  }
  this->string_ = str;

  delete[] str;

  LOG_COMM("SimpleString data successfully unloaded");

  return true;
}

}
}
}
