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

#include "fsrobo_r_driver/simple_message/execute_program.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#include <vector>
#include <iostream>

using industrial::shared_types::shared_int;
using std::vector;

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace execute_program
{

ExecuteProgram::ExecuteProgram(void)
{ this->init(); } ExecuteProgram::~ExecuteProgram(void) { } void ExecuteProgram::init() { this->name_.init();
  this->param_.init();
}

void ExecuteProgram::init(SimpleString &name, SimpleString &param)
{
  this->setName(name);
  this->setParam(param);
}

void ExecuteProgram::copyFrom(ExecuteProgram &src)
{
  src.getName(this->name_);
  src.getParam(this->param_);
}

bool ExecuteProgram::operator==(ExecuteProgram &rhs)
{
  bool rslt = this->name_ == rhs.name_ && this->param_ == rhs.param_;

  return rslt;
}

bool ExecuteProgram::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing ExecuteProgram command load");

  if (!this->name_.load(buffer))
  {
    LOG_ERROR("Failed to load ExecuteProgram name");
    return false;
  }

  if (!this->param_.load(buffer))
  {
    LOG_ERROR("Failed to load ExecuteProgram param");
    return false;
  }


  LOG_COMM("ExecuteProgram data successfully loaded");
  return true;
}

bool ExecuteProgram::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing ExecuteProgram command unload");

  if (!this->param_.unload(buffer))
  {
    LOG_ERROR("Failed to unload ExecuteProgram param");
    return false;
  }

  if (!this->name_.unload(buffer))
  {
    LOG_ERROR("Failed to unload ExecuteProgram name");
    return false;
  }

  LOG_COMM("ExecuteProgram data successfully unloaded");
  return true;
}

}
}
}
