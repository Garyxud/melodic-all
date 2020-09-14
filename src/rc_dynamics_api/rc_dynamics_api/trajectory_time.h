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

#ifndef RC_DYNAMICS_API_TRAJECTORY_TIME_H
#define RC_DYNAMICS_API_TRAJECTORY_TIME_H

namespace rc
{
/**
 * Represents a time stamp to query the trajectory of rcvisard's slam module.
 *
 * This class serves as convenience object to be used in conjunction with
 * RemoteInterface::getSlamTrajectory(...)
 *
 * A TrajectoryTime can be defined either as an absolute time stamp (see
 * Absolute()), or as a relative reference to either the start time
 * of the trajectory (see RelativeToStartOfTrajectory(...))  or its end (see
 * RelativeToEndOfTrajectory(...)).
 *
 * Internally, relative times are represented with signed values: positive values
 * define an offset of the trajectory's start, negative values an offset to the
 * trajectory's end (see getSe(), getNsec().
 */
class TrajectoryTime
{
public:
  /**
   * Creates an absolute time stamp of the given values.
   * @param sec   Unix time stamp (seconds since Jan 01 1970 UTC)
   * @param nsec  nanoseconds added to sec
   * @return
   */
  static TrajectoryTime Absolute(unsigned long sec, unsigned long nsec);

  /**
   * Creates a time stamp from the given values as an offset to the start point
   * of the trajectory.
   * @param sec   seconds since the start of trajectory
   * @param nsec  nanoseconds added to sec
   * @return
   */
  static TrajectoryTime RelativeToStart(unsigned long sec = 0, unsigned long nsec = 0);

  /**
   * Creates a time stamp from the given values as an offset from the end point
   * of the trajectory.
   * @param sec   seconds to the end of trajectory
   * @param nsec  nanoseconds added to sec
   * @return
   */
  static TrajectoryTime RelativeToEnd(unsigned long sec = 0, unsigned long nsec = 0);

  /**
   * Full constructor for specifiying a time either as relative offset or as
   * absolute timestamp
   * @param sec seconds of absolute timestamp, or of relative offset to either trajectory start (positive values) or
   * trajectory end (negative values)
   * @param nsec nanoseconds of absolute timestamp, or of relative offset to either trajectory start (positive values)
   * or trajectory end (negative values)
   * @param relative if true, sec and nsec values are treated as relative offset; otherwise they are treated as absolute
   * timestamp
   */
  TrajectoryTime(long sec, long nsec, bool relative);

  inline bool isRelative() const
  {
    return relative_;
  }

  inline long getSec() const
  {
    return sec_;
  }

  inline long getNsec() const
  {
    return nsec_;
  }

protected:
  bool relative_;
  long sec_, nsec_;
};
}

#endif  // RC_DYNAMICS_API_TRAJECTORY_TIME_H
