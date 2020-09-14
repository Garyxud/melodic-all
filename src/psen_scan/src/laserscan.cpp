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

#include <psen_scan/laserscan.h>

namespace psen_scan
{
/**
 * @brief Construct a new Laser Scan object
 *
 * @param resolution Distance of angle between the measurements in tenths of degree.
 * @param min_scan_angle Lowest  Angle the Scanner is scanning in tenths of degree.
 * @param max_scan_angle Highest Angle the Scanner is scanning in tenths of degree.
 */
LaserScan::LaserScan(const PSENscanInternalAngle& resolution,
                     const PSENscanInternalAngle& min_scan_angle,
                     const PSENscanInternalAngle& max_scan_angle)
  : resolution_(resolution), min_scan_angle_(min_scan_angle), max_scan_angle_(max_scan_angle)
{
  measures_.clear();
}

}  // namespace psen_scan