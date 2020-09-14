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

#ifndef FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_TOOL_OFFSET_MESSAGE_H
#define FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_TOOL_OFFSET_MESSAGE_H

#include "simple_message/typed_message.h"
#include "simple_message/shared_types.h"
#include "fsrobo_r_driver/simple_message/fsrobo_r_simple_message.h"
#include "fsrobo_r_driver/simple_message/set_tool_offset.h"

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace set_tool_offset_message
{
/**
 * \brief Class encapsulated FSRobo-R set tool offset message generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type.
 *
 * This message simply wraps the following data type:
 *   fsrobo_r_driver::simple_message::set_tool_offset::SetToolOffset
 * The data portion of this typed message matches SetToolOffset exactly.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class SetToolOffsetMessage : public industrial::typed_message::TypedMessage
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  SetToolOffsetMessage(void);

  /**
   * \brief Destructor
   *
   */
  ~SetToolOffsetMessage(void);

  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a read set tool offset structure
   *
   * \param cmd read set tool offset data structure
   *
   */
  void init(fsrobo_r_driver::simple_message::set_tool_offset::SetToolOffset & cmd);

  /**
   * \brief Initializes a new message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return this->cmd_.byteLength();
  }

  fsrobo_r_driver::simple_message::set_tool_offset::SetToolOffset cmd_;

private:
};
}
}
}

#endif // FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_TOOL_OFFSET_MESSAGE_H
