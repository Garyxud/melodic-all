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

#ifndef FSROBO_R_DRIVER_SIMPLE_MESSAGE_EXECUTE_PROGRAM_H
#define FSROBO_R_DRIVER_SIMPLE_MESSAGE_EXECUTE_PROGRAM_H

#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#include "fsrobo_r_driver/simple_message/simple_string.h"
#include <vector>

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace execute_program
{
  using fsrobo_r_driver::simple_message::simple_string::SimpleString;

/**
 * \brief Class encapsulated write single io data. FSRobo-R specific interface
 * to set IO element on the controller.
 *
 * The byte representation of a write single IO command is as follows
 * (in order lowest index to highest). The standard sizes are given,
 * but can change based on type sizes:
 *
 *   member:             type                                      size
 *   address             (industrial::shared_types::shared_int)    4  bytes
 *   value               (industrial::shared_types::shared_int)    4  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class ExecuteProgram : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  ExecuteProgram(void);

  /**
   * \brief Destructor
   *
   */
  ~ExecuteProgram(void);

  /**
   * \brief Initializes a empty execute program command
   *
   */
  void init();

  /**
   * \brief Initializes a complete execute program command
   *
   */
  void init(SimpleString &name, SimpleString &param);

  /**
   * \brief Sets name data
   *
   * \param name
   */
  void setName(SimpleString &name)
  {
    this->name_.copyFrom(name);
  }

  /**
   * \brief Returns a copy of the name data
   *
   * \param dest name
   */
  void getName(SimpleString &dest)
  {
    dest.copyFrom(this->name_);
  }

  /**
   * \brief Sets param data
   *
   * \param param
   */
  void setParam(SimpleString &param)
  {
    this->param_.copyFrom(param);
  }

  /**
   * \brief Returns a copy of the name data
   *
   * \param dest name
   */
  void getParam(SimpleString &dest)
  {
    dest.copyFrom(this->param_);
  }
  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(ExecuteProgram &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(ExecuteProgram &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return name_.byteLength() + param_.byteLength();
  }

private:
  /**
   * \brief Value of program name.
   */
 SimpleString name_;

  /**
   * \brief Value of program param.
   */
 SimpleString param_;
};
}
}
}

#endif // FSROBO_R_DRIVER_SIMPLE_MESSAGE_EXECUTE_PROGRAM_H