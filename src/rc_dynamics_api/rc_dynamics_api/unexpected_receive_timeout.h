/*
 * This file is part of the rc_dynamics_api package.
 *
 * Copyright (c) 2017 Roboception GmbH
 * All rights reserved
 *
 * Author: Christian Emmerich
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RC_DYNAMICS_API_UNEXPECTED_RECEIVE_TIMEOUT_H
#define RC_DYNAMICS_API_UNEXPECTED_RECEIVE_TIMEOUT_H

#include <stdexcept>

namespace rc
{
namespace dynamics
{
/**
 * Exception handling cases where actually everything should be fine and
 * rc_visard's dynamic state estimates should be received, but it is not.
 *
 * Gives some possible explanations why this timeout might have been received.
 */
class UnexpectedReceiveTimeout : public std::runtime_error
{
public:
  /**
   * @brief Constructor.
   * @param timeout_millis time out in milli seconds
   */
  UnexpectedReceiveTimeout(unsigned int timeout_millis);

  /**
   * @brief Returns the corresponding timeout in milli seconds
   * @return timeout that was received unexpectedly (in ms)
   */
  inline unsigned int getTimeout() const noexcept
  {
    return timeout_;
  }

protected:
  unsigned int timeout_;
};
}
}

#endif  // RC_DYNAMICS_API_UNEXPECTED_RECEIVE_TIMEOUT_H
