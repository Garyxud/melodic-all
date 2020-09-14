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

#include "fsrobo_r_driver/simple_message/posture.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#include <vector>

using industrial::shared_types::shared_int;

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace posture
{

Posture::Posture(void)
{
  this->init();
}

Posture::~Posture(void)
{
}

void Posture::init()
{
  this->init(0);
}

void Posture::init(shared_int posture)
{
  this->setPosture(posture);
}

void Posture::copyFrom(Posture &src)
{
  this->setPosture(src.getPosture());
}

bool Posture::operator==(Posture &rhs)
{
  bool rslt = this->posture_ == rhs.posture_;

  return rslt;
}

bool Posture::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing Posture command load");

  if (!buffer->load(this->posture_))
  {
    LOG_ERROR("Failed to load Posture posture");
    return false;
  }

  LOG_COMM("Posture data successfully loaded");
  return true;
}

bool Posture::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing Posture command unload");

  if (!buffer->unload(this->posture_))
  {
    LOG_ERROR("Failed to unload Posture posture");
    return false;
  }

  LOG_COMM("SetPosture data successfully unloaded");
  return true;
}

}
}
}
