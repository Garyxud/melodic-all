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

#ifndef FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_TOOL_OFFSET_H
#define FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_TOOL_OFFSET_H

#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#include <vector>

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace set_tool_offset
{

/**
 * \brief Class encapsulated tool offset data. FSRobo-R specific interface
 * to set tool offset on the controller.
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
class SetToolOffset : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  SetToolOffset(void);

  /**
   * \brief Destructor
   *
   */
  ~SetToolOffset(void);

  /**
   * \brief Initializes a empty set tool offset command
   *
   */
  void init();

  /**
   * \brief Initializes a complete set tool offset command
   *
   */
  void init(industrial::shared_types::shared_real x,
    industrial::shared_types::shared_real y,
    industrial::shared_types::shared_real z,
    industrial::shared_types::shared_real rz,
    industrial::shared_types::shared_real ry,
    industrial::shared_types::shared_real rx);

  /**
   * \brief Sets offset
   *
   * \param x
   */
  void setX(industrial::shared_types::shared_real x)
  {
    this->x_ = x;
  }

  /**
   * \brief Sets offset
   *
   * \param y
   */
  void setY(industrial::shared_types::shared_real y)
  {
    this->y_ = y;
  }

  /**
   * \brief Sets offset
   *
   * \param z
   */
  void setZ(industrial::shared_types::shared_real z)
  {
    this->z_ = z;
  }

  /**
   * \brief Sets offset
   *
   * \param rz
   */
  void setRz(industrial::shared_types::shared_real rz)
  {
    this->rz_ = rz;
  }

  /**
   * \brief Sets offset
   *
   * \param ry
   */
  void setRy(industrial::shared_types::shared_real ry)
  {
    this->ry_ = ry;
  }

  /**
   * \brief Sets offset
   *
   * \param rx
   */
  void setRx(industrial::shared_types::shared_real rx)
  {
    this->rx_ = rx;
  }

  /**
   * \brief Returns offset
   *
   * \return offset
   */
  industrial::shared_types::shared_real getX()
  {
    return this->x_;
  }

  /**
   * \brief Returns offset
   *
   * \return offset
   */
  industrial::shared_types::shared_real getY()
  {
    return this->y_;
  }

  /**
   * \brief Returns offset
   *
   * \return offset
   */
  industrial::shared_types::shared_real getZ()
  {
    return this->z_;
  }

  /**
   * \brief Returns offset
   *
   * \return offset
   */
  industrial::shared_types::shared_real getRz()
  {
    return this->rz_;
  }

  /**
   * \brief Returns offset
   *
   * \return offset
   */
  industrial::shared_types::shared_real getRx()
  {
    return this->rx_;
  }

  /**
   * \brief Returns offset
   *
   * \return offset
   */
  industrial::shared_types::shared_real getRy()
  {
    return this->ry_;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(SetToolOffset &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(SetToolOffset &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 6 * sizeof(industrial::shared_types::shared_int);
  }

private:
  /**
   * \brief Values of tool origin.
   */
  industrial::shared_types::shared_real x_;
  industrial::shared_types::shared_real y_;
  industrial::shared_types::shared_real z_;
  industrial::shared_types::shared_real rz_;
  industrial::shared_types::shared_real ry_;
  industrial::shared_types::shared_real rx_;
};
}
}
}

#endif // FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_TOOL_OFFSET_H