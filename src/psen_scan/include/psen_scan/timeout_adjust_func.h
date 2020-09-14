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

#ifndef TIMEOUT_ADJUST_FUNC_H
#define TIMEOUT_ADJUST_FUNC_H

#include <chrono>

namespace psen_scan_utils
{
constexpr auto DEFAULT_TIMEOUT_INCREASE{ std::chrono::seconds(10) };
constexpr auto DEFAULT_MAX_TIMEOUT{ std::chrono::seconds(60) };

/**
 * @brief Increases the given timeout by the specified timeout increase. If the new timeout exceeds the max timeout,
 * the returned timeout is equal to the specified maximum timeout.
 */
static std::chrono::steady_clock::duration
adjustTimeout(const std::chrono::steady_clock::duration& timeout,
              const std::chrono::steady_clock::duration timeout_increase = DEFAULT_TIMEOUT_INCREASE,
              const std::chrono::steady_clock::duration max_timeout = DEFAULT_MAX_TIMEOUT)
{
  const std::chrono::steady_clock::duration new_timeout{ timeout + timeout_increase };
  const bool timeout_overflow{ new_timeout > max_timeout };
  return timeout_overflow ? max_timeout : new_timeout;
}

}  // namespace psen_scan_utils

#endif  // TIMEOUT_ADJUST_FUNC_H
