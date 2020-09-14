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

#ifndef PSEN_SCAN_INTERNAL_ANGLE
#define PSEN_SCAN_INTERNAL_ANGLE

#include <ostream>

namespace psen_scan
{
class PSENscanInternalAngle;

/**
 * @brief Class to model angles in degrees from user's perspective
 *
 */
class Degree
{
private:
  double angle_;

public:
  Degree(const Degree& angle);
  explicit Degree(const double& angle);
  explicit Degree(const PSENscanInternalAngle& angle);
  explicit operator double() const noexcept;
  bool operator<(const Degree& rhs) const;
  bool operator>(const Degree& rhs) const;
  bool operator==(const Degree& rhs) const;
  Degree& operator*=(const double& rhs);
  Degree operator*(const double& rhs);
  Degree& operator-=(const Degree& rhs);
  Degree operator-(const Degree& rhs);
  Degree operator-();
  Degree& operator=(const Degree& rhs) = default;
  friend std::ostream& operator<<(std::ostream& os, const Degree& deg);
};

/**
 * @brief Class to model angles in PSENscan internal format (tenth of degrees)
 *
 */
class PSENscanInternalAngle
{
private:
  int angle_;

public:
  PSENscanInternalAngle(const PSENscanInternalAngle& angle);
  explicit PSENscanInternalAngle(const int& angle);
  explicit PSENscanInternalAngle(const Degree& angle);
  explicit operator int() const noexcept;
  bool operator<(const PSENscanInternalAngle& rhs) const;
  bool operator>(const PSENscanInternalAngle& rhs) const;
  bool operator<=(const PSENscanInternalAngle& rhs) const;
  bool operator>=(const PSENscanInternalAngle& rhs) const;
  bool operator==(const PSENscanInternalAngle& rhs) const;
  bool operator!=(const PSENscanInternalAngle& rhs) const;
  PSENscanInternalAngle operator-=(const PSENscanInternalAngle& rhs);
  PSENscanInternalAngle operator-(const PSENscanInternalAngle& rhs) const;
  PSENscanInternalAngle operator-();
  PSENscanInternalAngle operator+=(const PSENscanInternalAngle& rhs);
  PSENscanInternalAngle operator+(const PSENscanInternalAngle& rhs) const;
  PSENscanInternalAngle& operator=(const PSENscanInternalAngle& rhs) = default;
  friend std::ostream& operator<<(std::ostream& os, const PSENscanInternalAngle& deg);
};
}  // namespace psen_scan

#endif  // PSEN_SCAN_INTERNAL_ANGLE