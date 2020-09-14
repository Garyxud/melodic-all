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

#ifndef FSROBO_R_DRIVER_SIMPLE_MESSAGE_SIMPLE_STRING_H
#define FSROBO_R_DRIVER_SIMPLE_MESSAGE_SIMPLE_STRING_H

#include <string>
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace simple_string
{
  using std::string;

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

class SimpleString : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  SimpleString(void);

  /**
   * \brief Destructor
   *
   */
  ~SimpleString(void);

  /**
   * \brief Initializes a empty motion control reply
   *
   */
  void init();

  /**
   * \brief Initializes a complete read single io reply
   *
   */
  void init(string str);

  /**
   * \brief Sets the string data
   *
   * \param string data
   */
  void setString(string str)
  {
    this->string_ = str;
    this->string_size_ = str.size();
  }

  /**
   * \brief Returns the string size
   *
   * \return string size
   */
  industrial::shared_types::shared_int getStringSize() const
  {
    return this->string_size_;
  }

  /**
   * \brief Returns the string data
   *
   * \return string data
   */
  string getString() const
  {
    return this->string_;
  }


  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(SimpleString &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(SimpleString &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 1 * sizeof(industrial::shared_types::shared_int) + string_.size();
  }

private:
  /**
   * \brief string size
   */
  industrial::shared_types::shared_int string_size_;
  /**
   * \brief string data
   */
  string string_;
};
}
}
}

#endif // FSROBO_R_DRIVER_SIMPLE_MESSAGE_SIMPLE_STRING_H