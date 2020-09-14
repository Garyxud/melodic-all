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

#include <chrono>

#include <gtest/gtest.h>

#include <psen_scan/timeout_adjust_func.h>

using namespace psen_scan_utils;

namespace psen_scan_test
{
TEST(TimeoutAdjustFuncTests, testTimeoutIncrease)
{
  constexpr auto max_timeout{ std::chrono::seconds(77) };
  constexpr auto timeout_increase{ std::chrono::seconds(7) };
  constexpr auto my_timeout{ max_timeout - 2 * timeout_increase };
  ASSERT_EQ(my_timeout + timeout_increase, adjustTimeout(my_timeout, timeout_increase, max_timeout))
      << "Incorrect timeout increase";
}

TEST(TimeoutAdjustFuncTests, testMaxTimeoutExceed)
{
  constexpr auto my_timeout{ std::chrono::seconds(75) };
  constexpr auto timeout_increase{ std::chrono::seconds(7) };
  constexpr auto max_allowed_timeout{ std::chrono::seconds(77) };

  ASSERT_EQ(max_allowed_timeout, adjustTimeout(my_timeout, timeout_increase, max_allowed_timeout))
      << "Incorrect max timeout returned";
}

TEST(TimeoutAdjustFuncTests, testDefaultTimeoutIncrease)
{
  constexpr auto my_timeout{ std::chrono::seconds(20) };
  ASSERT_EQ(my_timeout + DEFAULT_TIMEOUT_INCREASE, adjustTimeout(my_timeout)) << "Default timeout increase incorrect";
}

TEST(TimeoutAdjustFuncTests, testDefaultMaxTimeout)
{
  constexpr auto my_timeout{ DEFAULT_MAX_TIMEOUT - std::chrono::seconds(1) };
  constexpr auto timeout_increase{ std::chrono::seconds(2) };

  ASSERT_EQ(DEFAULT_MAX_TIMEOUT, adjustTimeout(my_timeout, timeout_increase)) << "Max default timeout incorrect";
}

TEST(TimeoutAdjustFuncTests, testMaxTimeoutExceedingTimeoutArgument)
{
  constexpr auto my_timeout{ DEFAULT_MAX_TIMEOUT + std::chrono::seconds(7) };
  ASSERT_EQ(DEFAULT_MAX_TIMEOUT, adjustTimeout(my_timeout)) << "Max timeout incorrect";
}

}  // namespace psen_scan_test
