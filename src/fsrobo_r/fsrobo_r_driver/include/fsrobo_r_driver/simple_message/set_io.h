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

#ifndef FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_IO_H
#define FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_IO_H

#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#include <vector>

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace io_control
{

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
class SetIO : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  SetIO(void);
  /**
   * \brief Destructor
   *
   */
  ~SetIO(void);

  /**
   * \brief Initializes a empty write single io command
   *
   */
  void init();

  /**
   * \brief Initializes a complete write single io command
   *
   */
  void init(industrial::shared_types::shared_int fun,
    industrial::shared_types::shared_int address, std::vector<industrial::shared_types::shared_int> &data);

  /**
   * \brief Sets address
   *
   * \param address Controller address of the targeted IO element.
   */
  void setAddress(industrial::shared_types::shared_int address)
  {
    this->address_ = address;
  }

  /**
   * \brief Sets value
   *
   * \param value Controller value of the targeted IO element.
   */
  void setFun(industrial::shared_types::shared_int fun)
  {
    this->fun_ = fun;
  }

  /**
   * \brief Sets value
   *
   * \param value Controller value of the targeted IO element.
   */
  void setDataSize(industrial::shared_types::shared_int data_size)
  {
    this->data_size_ = data_size;
  }

  /**
   * \brief Returns the address of the IO element
   *
   * \return address
   */
  industrial::shared_types::shared_int getAddress()
  {
    return this->address_;
  }

  /**
   * \brief Returns the value of the IO element
   *
   * \return value
   */
  industrial::shared_types::shared_int getFun()
  {
    return this->fun_;
  }

  /**
   * \brief Returns the value of the IO element
   *
   * \return value
   */
  industrial::shared_types::shared_int getDataSize()
  {
    return this->data_size_;
  }

  /**
   * \brief Returns the value of the IO element
   *
   * \return value
   */
  industrial::shared_types::shared_int *getData()
  {
    return this->data_;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(SetIO &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(SetIO &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 35 * sizeof(industrial::shared_types::shared_int);
  }

private:
  /**
   * \brief Value of IO element.
   */
  industrial::shared_types::shared_int fun_;

  /**
   * \brief Address of IO element.
   */
  industrial::shared_types::shared_int address_;

  industrial::shared_types::shared_int data_size_;
  industrial::shared_types::shared_int data_[32];

};
}
}
}

#endif // FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_SINGLE_IO_H