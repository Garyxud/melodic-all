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

#ifndef FSROBO_R_DRIVER_SIMPLE_MESSAGE_SYS_STAT_H
#define FSROBO_R_DRIVER_SIMPLE_MESSAGE_SYS_STAT_H

#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#include <vector>

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace sys_stat
{

/**
 * \brief Enumeration of SysStat type codes.
 */
namespace sys_stat_type
{
enum SysStatType
{
  EMS_STATE = 1,
  MOTION_ID = 2,
  INC_MODE = 3,
  ABS_LOST = 4,
  MOTION_REQUEST = 5,
  MOTION_STATE = 6,
  SPI_ERROR = 7,
  SERVO_CONTROL = 8,
  PULSE = 0x8000
};
}
typedef sys_stat_type::SysStatType SysStatType;


/**
 * \brief Class encapsulated write single io data. FSRobo-R specific interface
 * to set IO element on the controller.
 *
 * The byte representation of a write single IO command is as follows
 * (in order lowest index to highest). The standard sizes are given,
 * but can change based on type sizes:
 *
 *   member:             type                                      size
 *   stat_type             (industrial::shared_types::shared_int)    4  bytes
 *   result               (industrial::shared_types::shared_int)    4  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class SysStat : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  SysStat(void);
  /**
   * \brief Destructor
   *
   */
  ~SysStat(void);

  /**
   * \brief Initializes a empty write single io command
   *
   */
  void init();

  /**
   * \brief Initializes a complete SysStat command 
   *
   */
  void init(industrial::shared_types::shared_int stat_type, industrial::shared_types::shared_int result);

  /**
   * \brief Sets stat type
   *
   * \param stat_type Status type value of SysStat element.
   */
  void setStatType(industrial::shared_types::shared_int stat_type)
  {
    this->stat_type_ = stat_type;
  }

  /**
   * \brief Sets result
   *
   * \param result result value of SysStat element.
   */
  void setResult(industrial::shared_types::shared_int result)
  {
    this->result_ = result;
  }

  /**
   * \brief Returns the value of the SysStat element
   *
   * \return stat type value
   */
  industrial::shared_types::shared_int getStatType()
  {
    return this->stat_type_;
  }

  /**
   * \brief Returns the value of the SysStat element
   *
   * \return result value
   */
  industrial::shared_types::shared_int getResult()
  {
    return this->result_;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(SysStat &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(SysStat &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 2 * sizeof(industrial::shared_types::shared_int);
  }

private:
  /**
   * \brief Value of SysStat element.
   */
  industrial::shared_types::shared_int stat_type_;

  /**
   * \brief Address of SysStat element.
   */
  industrial::shared_types::shared_int result_;

};
}
}
}

#endif // FSROBO_R_DRIVER_SIMPLE_MESSAGE_SET_SINGLE_IO_H