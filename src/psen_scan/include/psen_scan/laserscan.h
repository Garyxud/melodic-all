// Copyright (c) 2019 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_LASERSCAN_H
#define PSEN_SCAN_LASERSCAN_H

#include <vector>
#include <cstdint>
#include <psen_scan/psen_scan_internal_angle.h>

namespace psen_scan
{
/**
 * @brief Class to hold the data for one laserscan without depencies to ROS.
 *
 */
typedef struct LaserScan
{
public:
  LaserScan(const PSENscanInternalAngle& resolution,
            const PSENscanInternalAngle& min_scan_angle,
            const PSENscanInternalAngle& max_scan_angle);

  std::vector<uint16_t> measures_;             /**< Measurement data of the laserscan in Millimeters. */
  PSENscanInternalAngle resolution_;           /**< Distance of angle between the measurements in tenths of degree. */
  PSENscanInternalAngle const min_scan_angle_; /**< Lowest  Angle the Scanner is scanning in tenths of degree.*/
  PSENscanInternalAngle const max_scan_angle_; /**< Highest Angle the Scanner is scanning in tenths of degree.*/
} LaserScan;
}  // namespace psen_scan

#endif  // PSEN_SCAN_LASERSCAN_H