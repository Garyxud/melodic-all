// Copyright (c) 2020 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef PSEN_SCAN_DEFAULT_PARAMETERS_H
#define PSEN_SCAN_DEFAULT_PARAMETERS_H

#include <string>

#include <psen_scan/psen_scan_internal_angle.h>

namespace psen_scan
{
static const std::string DEFAULT_FRAME_ID = "scanner"; /**< ROS Frame ID */
static const uint16_t DEFAULT_SKIP = 0; /**< How many incoming frames should be skipped (reduces publish rate) */
static const PSENscanInternalAngle DEFAULT_ANGLE_START(0);  /**< Start angle of measurement */
static const PSENscanInternalAngle DEFAULT_ANGLE_END(2750); /**< End angle of measurement */
static const Degree DEFAULT_X_AXIS_ROTATION(137.5);         /**< Rotation of x-axis around the center. */
static const Degree MAX_X_AXIS_ROTATION(360.0);             /**< Maximum value for x-axis-rotation parameter */
static const Degree MIN_X_AXIS_ROTATION(-360.0);            /**< Minimum value for x-axis-rotation parameter */
static const std::string DEFAULT_PUBLISH_TOPIC = "scan";    /**< Topic to publish LaserScan data on */
}  // namespace psen_scan
#endif  // PSEN_SCAN_DEFAULT_PARAMETERS_H