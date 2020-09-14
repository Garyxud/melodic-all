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

#ifndef FSROBO_R_DRIVER_SIMPLE_MESSAGE_IO_STATE_H
#define FSROBO_R_DRIVER_SIMPLE_MESSAGE_IO_STATE_H

#ifndef FLATHEADERS
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#else
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#endif

namespace fsrobo_r_driver
{
namespace simple_message
{
namespace io_state
{

/**
 * \brief Class encapsulated robot status data.  The robot status data is
 * meant to mirror the industrial_msgs/RobotStatus message.
 *
 *
 * The byte representation of a robot status is as follows (in order lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   drives_powered      (industrial::shared_types::shared_int)    4  bytes
 *   e_stopped           (industrial::shared_types::shared_int)    4  bytes
 *   error_code          (industrial::shared_types::shared_int)    4  bytes
 *   in_error            (industrial::shared_types::shared_int)    4  bytes
 *   in_motion           (industrial::shared_types::shared_int)    4  bytes
 *   mode                (industrial::shared_types::shared_int)    4  bytes
 *   motion_possible     (industrial::shared_types::shared_int)    4  bytes
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class IOState : public industrial::simple_serialize::SimpleSerialize
{
public:
/**
 * \brief Default constructor
 *
 * This method creates empty data.
 *
 */
IOState(void);

/**
 * \brief Destructor
 *
 */
~IOState(void);

/**
 * \brief Initializes an empty robot status
 *
 */
void init();

/**
 * \brief Initializes a full robot status message
 *
 */
//void init(TriState drivesPowered, TriState eStopped, industrial::shared_types::shared_int errorCode, TriState inError,
//          TriState inMotion, RobotMode mode, TriState motionPossible);

industrial::shared_types::shared_int getDigital(int idx)
{
  return digital_[idx];
}

industrial::shared_types::shared_int getAnalog(int idx)
{
  return analog_[idx];
}

void setDigital(int idx, industrial::shared_types::shared_int val)
{
  this->digital_[idx] = val;
}

void setAnalog(int idx, industrial::shared_types::shared_int val)
{
  this->analog_[idx] = val;
}

/**
 * \brief Copies the passed in value
 *
 * \param src (value to copy)
 */
void copyFrom(IOState &src);

/**
 * \brief == operator implementation
 *
 * \return true if equal
 */
bool operator==(IOState &rhs);

// Overrides - SimpleSerialize
bool load(industrial::byte_array::ByteArray *buffer);
bool unload(industrial::byte_array::ByteArray *buffer);
unsigned int byteLength()
{
  return 3 * sizeof(industrial::shared_types::shared_int);
}

private:
/**
 * \brief Digital
 */
industrial::shared_types::shared_int digital_[2];

/**
 * \brief Analog
 */
industrial::shared_types::shared_int analog_[1];

};

}
}
}

#endif // FSROBO_R_DRIVER_SIMPLE_MESSAGE_IO_STATE_H
