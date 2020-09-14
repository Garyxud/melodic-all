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

#include <psen_scan/psen_scan_internal_angle.h>
#include <cmath>

namespace psen_scan
{
/**
 * @brief Construct a new Degree:: Degree object from a double precision floating point number.
 *
 * @param angle Angle to convert to Degree object.
 */
Degree::Degree(const double& angle) : angle_(angle)
{
}

/**
 * @brief Construct a new Degree:: Degree object from another Degree object.
 *
 * @param angle Degree object to copy the angle from.
 */
Degree::Degree(const Degree& angle) : angle_(static_cast<double>(angle))
{
}

/**
 * @brief Construct a new Degree:: Degree object from a PSENscanInternalAngle object.
 *
 * @param angle PSENscanInternalAngle object to get the angle from and convert it to Degrees.
 */
Degree::Degree(const PSENscanInternalAngle& angle) : angle_(static_cast<int>(angle) / 10.)
{
}

/**
 * @brief Get the saved angle as double (still needs to be explicitly cast to double).
 *
 * @return double Angle (still needs to be explicitly cast to double).
 */
Degree::operator double() const noexcept
{
  return angle_;
}

/**
 * @brief Compare a Degree object with another Degree object
 *
 * @param rhs Degree object to be compared with
 * @return bool
 */
bool Degree::operator<(const Degree& rhs) const
{
  return angle_ < rhs.angle_;
}

/**
 * @brief Compare a Degree object with another Degree object
 *
 * @param rhs Degree object to be compared with
 * @return bool
 */
bool Degree::operator>(const Degree& rhs) const
{
  return angle_ > rhs.angle_;
}

/**
 * @brief Compare a Degree object with another Degree object
 *
 * @param rhs Degree object to be compared with
 * @return bool
 */
bool Degree::operator==(const Degree& rhs) const
{
  return angle_ == rhs.angle_;
}

/**
 * @brief Define multiplying a Degree object by a double precision floating point number.
 *
 * @param rhs double precision floating point number to scale the angle by.
 * @return Degree
 */
Degree& Degree::operator*=(const double& rhs)
{
  angle_ *= rhs;
  return *this;
}

/**
 * @brief Multiply a Degree object by a double precision floating point number.
 *
 * @param rhs double precision floating point number to scale the angle by.
 * @return Degree
 */
Degree Degree::operator*(const double& rhs)
{
  Degree ret(*this);
  ret *= rhs;
  return ret;
}

/**
 * @brief Define subtracting a Degree object by another Degree.
 *
 * @param rhs Degree to subtract the angle by.
 * @return Degree
 */
Degree& Degree::operator-=(const Degree& rhs)
{
  angle_ -= rhs.angle_;
  return *this;
}

/**
 * @brief Subtract a Degree object by another Degree.
 *
 * @param rhs Degree to subtract the angle by.
 * @return Degree
 */
Degree Degree::operator-(const Degree& rhs)
{
  Degree ret(*this);
  ret -= rhs;
  return ret;
}

/**
 * @brief Unary Minus Operator switches the sign of the angle.
 *
 * @return Degree
 */
Degree Degree::operator-()
{
  Degree ret(*this);
  ret.angle_ = -ret.angle_;
  return ret;
}

/**
 * @brief Define how Degree should behave with stream operator
 *
 * @param os Output Stream
 * @param deg Degree
 * @return std::ostream& Reference to Output Stream
 */
std::ostream& operator<<(std::ostream& os, const Degree& deg)
{
  return os << deg.angle_;
}

/**
 * @brief Construct a new PSENscanInternalAngle::PSENscanInternalAngle object from another PSENscanInternalAngle object.
 *
 * @param angle PSENscanInternalAngle object to copy the angle from.
 */
PSENscanInternalAngle::PSENscanInternalAngle(const PSENscanInternalAngle& angle) : angle_(static_cast<int>(angle))
{
}

/**
 * @brief Construct a new PSENscanInternalAngle::PSENscanInternalAngle object from an Integer.
 *
 * @param angle
 */
PSENscanInternalAngle::PSENscanInternalAngle(const int& angle) : angle_(angle)
{
}

/**
 * @brief Construct a new PSENscanInternalAngle::PSENscanInternalAngle object from a Degree object.
 *
 * @param angle Degree object to get the angle from and convert it to PSENscanInternalAngle.
 */
PSENscanInternalAngle::PSENscanInternalAngle(const Degree& angle)
  : angle_(static_cast<int>(round(static_cast<double>(angle) * 10.)))
{
  if ((angle_ / 10.) != round(static_cast<double>(angle) * 10.) / 10.)
  {
    throw std::overflow_error("Degree cannot be converted to PSENscanInternalAngle.");
  }
}

/**
 * @brief Get the PSENscanInternalAngle as an integer.
 *
 * @return int Integer representation of PSENscanInternalAngle.
 */
PSENscanInternalAngle::operator int() const noexcept
{
  return angle_;
}

/**
 * @brief Compare a PSENscanInternalAngle object with another PSENscanInternalAngle object
 *
 * @param rhs PSENscanInternalAngle object to be compared with
 * @return bool
 */
bool PSENscanInternalAngle::operator<(const PSENscanInternalAngle& rhs) const
{
  return angle_ < rhs.angle_;
}

/**
 * @brief Compare a PSENscanInternalAngle object with another PSENscanInternalAngle object
 *
 * @param rhs PSENscanInternalAngle object to be compared with
 * @return bool
 */
bool PSENscanInternalAngle::operator>(const PSENscanInternalAngle& rhs) const
{
  return angle_ > rhs.angle_;
}

/**
 * @brief Compare a PSENscanInternalAngle object with another PSENscanInternalAngle object
 *
 * @param rhs PSENscanInternalAngle object to be compared with
 * @return bool
 */
bool PSENscanInternalAngle::operator<=(const PSENscanInternalAngle& rhs) const
{
  return angle_ <= rhs.angle_;
}

/**
 * @brief Compare a PSENscanInternalAngle object with another PSENscanInternalAngle object
 *
 * @param rhs PSENscanInternalAngle object to be compared with
 * @return bool
 */
bool PSENscanInternalAngle::operator>=(const PSENscanInternalAngle& rhs) const
{
  return angle_ >= rhs.angle_;
}

/**
 * @brief Compare a PSENscanInternalAngle object with another PSENscanInternalAngle object
 *
 * @param rhs PSENscanInternalAngle object to be compared with
 * @return bool
 */
bool PSENscanInternalAngle::operator==(const PSENscanInternalAngle& rhs) const
{
  return angle_ == rhs.angle_;
}

/**
 * @brief Compare a PSENscanInternalAngle object with another PSENscanInternalAngle object
 *
 * @param rhs PSENscanInternalAngle object to be compared with
 * @return bool
 */
bool PSENscanInternalAngle::operator!=(const PSENscanInternalAngle& rhs) const
{
  return angle_ != rhs.angle_;
}

/**
 * @brief Define subtracting a PSENscanInternalAngle object by another PSENscanInternalAngle.
 *
 * @param rhs PSENscanInternalAngle to subtract the angle by.
 * @return PSENscanInternalAngle
 */
PSENscanInternalAngle PSENscanInternalAngle::operator-=(const PSENscanInternalAngle& rhs)
{
  angle_ -= rhs.angle_;
  return *this;
}

/**
 * @brief Subtract a PSENscanInternalAngle object by another PSENscanInternalAngle.
 *
 * @param rhs PSENscanInternalAngle to subtract the angle by.
 * @return PSENscanInternalAngle
 */
PSENscanInternalAngle PSENscanInternalAngle::operator-(const PSENscanInternalAngle& rhs) const
{
  PSENscanInternalAngle ret(*this);
  ret -= rhs;
  return ret;
}

/**
 * @brief Unary Minus Operator to switch the sign of the Degree
 *
 * @return PSENscanInternalAngle
 */
PSENscanInternalAngle PSENscanInternalAngle::operator-()
{
  PSENscanInternalAngle ret(*this);
  ret.angle_ = -ret.angle_;
  return ret;
}

/**
 * @brief Define adding a PSENscanInternalAngle object to another PSENscanInternalAngle.
 *
 * @param rhs PSENscanInternalAngle to add the angle to.
 * @return PSENscanInternalAngle
 */
PSENscanInternalAngle PSENscanInternalAngle::operator+=(const PSENscanInternalAngle& rhs)
{
  angle_ += rhs.angle_;
  return *this;
}

/**
 * @brief Add a PSENscanInternalAngle object to another PSENscanInternalAngle.
 *
 * @param rhs PSENscanInternalAngle to add the angle to.
 * @return PSENscanInternalAngle
 */
PSENscanInternalAngle PSENscanInternalAngle::operator+(const PSENscanInternalAngle& rhs) const
{
  PSENscanInternalAngle ret(*this);
  ret += rhs;
  return ret;
}

/**
 * @brief Define how PSENscanInternalAngle should behave with stream operator
 *
 * @param os Output Stream
 * @param deg PSENscanInternalAngle
 * @return std::ostream& Reference to Output Stream
 */
std::ostream& operator<<(std::ostream& os, const PSENscanInternalAngle& deg)
{
  return os << deg.angle_;
}
}  // namespace psen_scan