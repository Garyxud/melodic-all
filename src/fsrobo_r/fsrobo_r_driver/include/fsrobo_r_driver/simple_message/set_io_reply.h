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

#ifndef FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_IO_REPLY_H
#define FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_IO_REPLY_H

#include <string>
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace io_control_reply
{

/**
 * \brief Enumeration of set IO reply result codes.
 */
namespace SetIOReplyResults
{
enum SetIOReplyResult
{
  FAILURE    = 0,
  SUCCESS    = 1
};
}
typedef SetIOReplyResults::SetIOReplyResult SetIOReplyResult;

/**
 * \brief Class encapsulated set io reply data.  These messages are sent
 * by the FSRobo-R controller in response to SetIO messages.
 *
 * The byte representation of a set io reply is as follows
 * (in order lowest index to highest). The standard sizes are given,
 * but can change based on type sizes:
 *
 *   member:             type                                      size
 *   result_code         (industrial::shared_types::shared_int)    4  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class SetIOReply : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  SetIOReply(void);

  /**
   * \brief Destructor
   *
   */
  ~SetIOReply(void);

  /**
   * \brief Initializes a empty motion control reply
   *
   */
  void init();

  /**
   * \brief Initializes a complete read single io reply
   *
   */
  void init(SetIOReplyResult result_code);

  /**
   * \brief Sets the result code
   *
   * \param result code
   */
  void setResultCode(industrial::shared_types::shared_int result_code)
  {
    this->result_code_ = result_code;
  }

  /**
   * \brief Returns the result code
   *
   * \return result_code number
   */
  industrial::shared_types::shared_int getResultCode() const
  {
    return this->result_code_;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(SetIOReply &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(SetIOReply &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 1 * sizeof(industrial::shared_types::shared_int);
  }

private:
  /**
   * \brief The result code
   */
  industrial::shared_types::shared_int result_code_;
};
}
}
}

#endif // FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_IO_REPLY_H