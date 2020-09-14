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

#include <gtest/gtest.h>
#include <limits>
#include <random>

#include <psen_scan/psen_scan_internal_angle.h>

using namespace psen_scan;

TEST(DegreeTest, copy_constructor)
{
  Degree deg1(2.);
  Degree deg2 = deg1;

  EXPECT_EQ(static_cast<double>(deg1), 2.);
  EXPECT_EQ(static_cast<double>(deg2), 2.);
  EXPECT_EQ(deg1, deg2);
}

TEST(DegreeTest, double_constructor)
{
  Degree deg(0.);
  EXPECT_EQ(static_cast<double>(deg), 0.);

  deg = Degree(std::numeric_limits<double>::max());
  EXPECT_EQ(static_cast<double>(deg), std::numeric_limits<double>::max());

  deg = Degree(std::numeric_limits<double>::lowest());
  EXPECT_EQ(static_cast<double>(deg), std::numeric_limits<double>::lowest());
}

TEST(DegreeTest, psen_scan_internal_angle_constructor)
{
  Degree deg(PSENscanInternalAngle(0));
  EXPECT_EQ(static_cast<double>(deg), 0.);

  deg = Degree(PSENscanInternalAngle(-1));
  EXPECT_EQ(static_cast<double>(deg), -0.1);

  deg = Degree(PSENscanInternalAngle(2));
  EXPECT_EQ(static_cast<double>(deg), 0.2);

  deg = Degree(PSENscanInternalAngle(1776));
  EXPECT_EQ(static_cast<double>(deg), 177.6);

  deg = Degree(PSENscanInternalAngle(std::numeric_limits<int>::max()));
  EXPECT_EQ(static_cast<double>(deg), std::numeric_limits<int>::max() / 10.);

  deg = Degree(PSENscanInternalAngle(std::numeric_limits<int>::lowest()));
  EXPECT_EQ(static_cast<double>(deg), std::numeric_limits<int>::lowest() / 10.);
}

TEST(DegreeTest, operator_smaller_than)
{
  Degree deg_neg_l(-100.);
  Degree deg_neg_s(-1.);
  Degree deg_zero(0.);
  Degree deg_pos_s(1.);
  Degree deg_pos_l(100.);

  EXPECT_EQ(deg_neg_l < deg_neg_l, false);
  EXPECT_EQ(deg_neg_l < deg_neg_s, true);
  EXPECT_EQ(deg_neg_l < deg_zero, true);
  EXPECT_EQ(deg_neg_l < deg_pos_s, true);
  EXPECT_EQ(deg_neg_l < deg_pos_l, true);

  EXPECT_EQ(deg_neg_s < deg_neg_l, false);
  EXPECT_EQ(deg_neg_s < deg_neg_s, false);
  EXPECT_EQ(deg_neg_s < deg_zero, true);
  EXPECT_EQ(deg_neg_s < deg_pos_s, true);
  EXPECT_EQ(deg_neg_s < deg_pos_l, true);

  EXPECT_EQ(deg_zero < deg_neg_l, false);
  EXPECT_EQ(deg_zero < deg_neg_s, false);
  EXPECT_EQ(deg_zero < deg_zero, false);
  EXPECT_EQ(deg_zero < deg_pos_s, true);
  EXPECT_EQ(deg_zero < deg_pos_l, true);

  EXPECT_EQ(deg_pos_s < deg_neg_l, false);
  EXPECT_EQ(deg_pos_s < deg_neg_s, false);
  EXPECT_EQ(deg_pos_s < deg_zero, false);
  EXPECT_EQ(deg_pos_s < deg_pos_s, false);
  EXPECT_EQ(deg_pos_s < deg_pos_l, true);

  EXPECT_EQ(deg_pos_l < deg_neg_l, false);
  EXPECT_EQ(deg_pos_l < deg_neg_s, false);
  EXPECT_EQ(deg_pos_l < deg_zero, false);
  EXPECT_EQ(deg_pos_l < deg_pos_s, false);
  EXPECT_EQ(deg_pos_l < deg_pos_l, false);
}

TEST(DegreeTest, operator_greater_than)
{
  Degree deg_neg_l(-100.);
  Degree deg_neg_s(-1.);
  Degree deg_zero(0.);
  Degree deg_pos_s(1.);
  Degree deg_pos_l(100.);

  EXPECT_EQ(deg_neg_l > deg_neg_l, false);
  EXPECT_EQ(deg_neg_l > deg_neg_s, false);
  EXPECT_EQ(deg_neg_l > deg_zero, false);
  EXPECT_EQ(deg_neg_l > deg_pos_s, false);
  EXPECT_EQ(deg_neg_l > deg_pos_l, false);

  EXPECT_EQ(deg_neg_s > deg_neg_l, true);
  EXPECT_EQ(deg_neg_s > deg_neg_s, false);
  EXPECT_EQ(deg_neg_s > deg_zero, false);
  EXPECT_EQ(deg_neg_s > deg_pos_s, false);
  EXPECT_EQ(deg_neg_s > deg_pos_l, false);

  EXPECT_EQ(deg_zero > deg_neg_l, true);
  EXPECT_EQ(deg_zero > deg_neg_s, true);
  EXPECT_EQ(deg_zero > deg_zero, false);
  EXPECT_EQ(deg_zero > deg_pos_s, false);
  EXPECT_EQ(deg_zero > deg_pos_l, false);

  EXPECT_EQ(deg_pos_s > deg_neg_l, true);
  EXPECT_EQ(deg_pos_s > deg_neg_s, true);
  EXPECT_EQ(deg_pos_s > deg_zero, true);
  EXPECT_EQ(deg_pos_s > deg_pos_s, false);
  EXPECT_EQ(deg_pos_s > deg_pos_l, false);

  EXPECT_EQ(deg_pos_l > deg_neg_l, true);
  EXPECT_EQ(deg_pos_l > deg_neg_s, true);
  EXPECT_EQ(deg_pos_l > deg_zero, true);
  EXPECT_EQ(deg_pos_l > deg_pos_s, true);
  EXPECT_EQ(deg_pos_l > deg_pos_l, false);
}

TEST(DegreeTest, operator_is_equal)
{
  Degree deg_neg(-100.);
  Degree deg_zero(0.);
  Degree deg_pos(100.);

  EXPECT_EQ(deg_neg == deg_neg, true);
  EXPECT_EQ(deg_neg == deg_zero, false);
  EXPECT_EQ(deg_neg == deg_pos, false);

  EXPECT_EQ(deg_zero == deg_neg, false);
  EXPECT_EQ(deg_zero == deg_zero, true);
  EXPECT_EQ(deg_zero == deg_pos, false);

  EXPECT_EQ(deg_pos == deg_neg, false);
  EXPECT_EQ(deg_pos == deg_zero, false);
  EXPECT_EQ(deg_pos == deg_pos, true);
}

TEST(DegreeTest, operator_multiplication_assignment)
{
  std::srand(1);

  for (int i = 0; i < 1000; i++)
  {
    double factor1 = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    double factor2 = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    double product = factor1 * factor2;
    Degree multiplicand(factor1);
    ASSERT_EQ(static_cast<double>(multiplicand), factor1);
    multiplicand *= factor2;
    EXPECT_EQ(static_cast<double>(multiplicand), product);
    multiplicand *= -1.;
    EXPECT_EQ(static_cast<double>(multiplicand), -product);
    multiplicand *= 1.;
    EXPECT_EQ(static_cast<double>(multiplicand), -product);
    multiplicand *= 0.;
    EXPECT_EQ(static_cast<double>(multiplicand), 0.);
  }

  for (int i = 0; i < 1000; i++)
  {
    Degree multiplicand_neg_unit(-1.);
    Degree multiplicand_zero(0.);
    Degree multiplicand_pos_unit(1.);

    double multiplicator = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    multiplicand_neg_unit *= multiplicator;
    EXPECT_EQ(static_cast<double>(multiplicand_neg_unit), -multiplicator);
    multiplicand_pos_unit *= multiplicator;
    EXPECT_EQ(static_cast<double>(multiplicand_pos_unit), multiplicator);
    multiplicand_zero *= multiplicator;
    EXPECT_EQ(static_cast<double>(multiplicand_zero), 0.);
  }
}

TEST(DegreeTest, operator_multiplication)
{
  std::srand(1);

  for (int i = 0; i < 1000; i++)
  {
    double factor1 = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    double factor2 = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    double product = factor1 * factor2;
    Degree multiplicand(factor1);
    ASSERT_EQ(static_cast<double>(multiplicand), factor1);
    EXPECT_EQ(static_cast<double>(multiplicand * factor2), product);
    EXPECT_EQ(multiplicand * -1., -multiplicand);
    EXPECT_EQ(multiplicand * 1., multiplicand);
    EXPECT_EQ(static_cast<double>(multiplicand * 0.), 0.);
  }

  Degree multiplicand_neg_unit(-1.);
  Degree multiplicand_zero(0.);
  Degree multiplicand_pos_unit(1.);

  for (int i = 0; i < 1000; i++)
  {
    double multiplicator = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    EXPECT_EQ(static_cast<double>(multiplicand_neg_unit * multiplicator), -multiplicator);
    EXPECT_EQ(static_cast<double>(multiplicand_pos_unit * multiplicator), multiplicator);
    EXPECT_EQ(static_cast<double>(multiplicand_zero * multiplicator), 0.);
  }
}

TEST(DegreeTest, operator_minus_assignment)
{
  std::srand(1);

  for (int i = 0; i < 1000; i++)
  {
    double minuend_temp = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    double subtrahend_temp = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    double difference = minuend_temp - subtrahend_temp;
    Degree minuend(minuend_temp);
    Degree subtrahend(subtrahend_temp);
    ASSERT_EQ(static_cast<double>(minuend), minuend_temp);
    ASSERT_EQ(static_cast<double>(subtrahend), subtrahend_temp);
    minuend -= subtrahend;
    EXPECT_EQ(static_cast<double>(minuend), difference);
    minuend -= Degree(0);
    EXPECT_EQ(minuend, minuend);
  }
}

TEST(DegreeTest, operator_minus)
{
  std::srand(1);

  for (int i = 0; i < 1000; i++)
  {
    double minuend_temp = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    double subtrahend_temp = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    double difference = minuend_temp - subtrahend_temp;
    Degree minuend(minuend_temp);
    Degree subtrahend(subtrahend_temp);
    ASSERT_EQ(static_cast<double>(minuend), minuend_temp);
    ASSERT_EQ(static_cast<double>(subtrahend), subtrahend_temp);
    EXPECT_EQ(static_cast<double>(minuend - subtrahend), difference);
    EXPECT_EQ(minuend - Degree(0), minuend);
    EXPECT_EQ(Degree(0) - subtrahend, -subtrahend);
  }
}

TEST(DegreeTest, operator_unary_minus)
{
  std::srand(1);

  for (int i = 0; i < 1000; i++)
  {
    double temp = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    Degree deg(temp);
    ASSERT_EQ(static_cast<double>(deg), temp);
    EXPECT_EQ(static_cast<double>(-deg), -temp);
  }
}

TEST(DegreeTest, operator_assignment)
{
  std::srand(1);

  for (int i = 0; i < 1000; i++)
  {
    double temp1 = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    double temp2 = (static_cast<double>(std::rand()) - static_cast<double>(RAND_MAX) / 2.) / 10000.;
    Degree deg1(temp1);
    Degree deg2(temp2);
    ASSERT_EQ(static_cast<double>(deg1), temp1);
    ASSERT_EQ(static_cast<double>(deg2), temp2);
    deg1 = deg2;
    EXPECT_EQ(deg1, deg2);
  }
}

TEST(DegreeTest, operator_stream)
{
  std::stringstream expected_string;
  expected_string << "123";

  std::stringstream ss;
  ss << Degree(123);

  EXPECT_EQ(ss.str(), expected_string.str());

  expected_string.str(std::string());
  expected_string << "1.23";

  ss.str(std::string());
  ss << Degree(1.23);

  EXPECT_EQ(ss.str(), expected_string.str());
}

TEST(PSENscanInternalAngleTest, copy_constructor)
{
  PSENscanInternalAngle deg(1);

  ASSERT_EQ(static_cast<int>(deg), 1);
  deg = PSENscanInternalAngle(2);
  ASSERT_EQ(static_cast<int>(deg), 2);
}

TEST(PSENscanInternalAngleTest, int_constructor)
{
  PSENscanInternalAngle deg_min(std::numeric_limits<int>::lowest());
  PSENscanInternalAngle deg_neg(-100);
  PSENscanInternalAngle deg_zero(0);
  PSENscanInternalAngle deg_pos(100);
  PSENscanInternalAngle deg_max(std::numeric_limits<int>::max());

  EXPECT_EQ(static_cast<int>(deg_min), std::numeric_limits<int>::lowest());
  EXPECT_EQ(static_cast<int>(deg_neg), -100);
  EXPECT_EQ(static_cast<int>(deg_zero), 0);
  EXPECT_EQ(static_cast<int>(deg_pos), 100);
  EXPECT_EQ(static_cast<int>(deg_max), std::numeric_limits<int>::max());
}

TEST(PSENscanInternalAngleTest, degree_constructor)
{
  EXPECT_ANY_THROW(PSENscanInternalAngle deg_double_min((Degree(std::numeric_limits<double>::lowest()))));
  EXPECT_ANY_THROW(PSENscanInternalAngle deg_int_min((Degree(std::numeric_limits<int>::lowest()))));
  PSENscanInternalAngle deg_neg(Degree(-100));
  PSENscanInternalAngle deg_zero(Degree(0));
  PSENscanInternalAngle deg_pos(Degree(100));
  EXPECT_ANY_THROW(PSENscanInternalAngle deg_int_max((Degree(std::numeric_limits<int>::max()))));
  EXPECT_ANY_THROW(PSENscanInternalAngle deg_double_max((Degree(std::numeric_limits<double>::max()))));

  EXPECT_EQ(static_cast<int>(deg_neg), -1000);
  EXPECT_EQ(static_cast<int>(deg_zero), 0);
  EXPECT_EQ(static_cast<int>(deg_pos), 1000);
}

TEST(PSENscanInternalAngleTest, operator_smaller_than)
{
  PSENscanInternalAngle deg_neg_l(-100);
  PSENscanInternalAngle deg_neg_s(-1);
  PSENscanInternalAngle deg_zero(0);
  PSENscanInternalAngle deg_pos_s(1);
  PSENscanInternalAngle deg_pos_l(100);

  EXPECT_EQ(deg_neg_l < deg_neg_l, false);
  EXPECT_EQ(deg_neg_l < deg_neg_s, true);
  EXPECT_EQ(deg_neg_l < deg_zero, true);
  EXPECT_EQ(deg_neg_l < deg_pos_s, true);
  EXPECT_EQ(deg_neg_l < deg_pos_l, true);

  EXPECT_EQ(deg_neg_s < deg_neg_l, false);
  EXPECT_EQ(deg_neg_s < deg_neg_s, false);
  EXPECT_EQ(deg_neg_s < deg_zero, true);
  EXPECT_EQ(deg_neg_s < deg_pos_s, true);
  EXPECT_EQ(deg_neg_s < deg_pos_l, true);

  EXPECT_EQ(deg_zero < deg_neg_l, false);
  EXPECT_EQ(deg_zero < deg_neg_s, false);
  EXPECT_EQ(deg_zero < deg_zero, false);
  EXPECT_EQ(deg_zero < deg_pos_s, true);
  EXPECT_EQ(deg_zero < deg_pos_l, true);

  EXPECT_EQ(deg_pos_s < deg_neg_l, false);
  EXPECT_EQ(deg_pos_s < deg_neg_s, false);
  EXPECT_EQ(deg_pos_s < deg_zero, false);
  EXPECT_EQ(deg_pos_s < deg_pos_s, false);
  EXPECT_EQ(deg_pos_s < deg_pos_l, true);

  EXPECT_EQ(deg_pos_l < deg_neg_l, false);
  EXPECT_EQ(deg_pos_l < deg_neg_s, false);
  EXPECT_EQ(deg_pos_l < deg_zero, false);
  EXPECT_EQ(deg_pos_l < deg_pos_s, false);
  EXPECT_EQ(deg_pos_l < deg_pos_l, false);
}

TEST(PSENscanInternalAngleTest, operator_greater_than)
{
  PSENscanInternalAngle deg_neg_l(-100);
  PSENscanInternalAngle deg_neg_s(-1);
  PSENscanInternalAngle deg_zero(0);
  PSENscanInternalAngle deg_pos_s(1);
  PSENscanInternalAngle deg_pos_l(100);

  EXPECT_EQ(deg_neg_l > deg_neg_l, false);
  EXPECT_EQ(deg_neg_l > deg_neg_s, false);
  EXPECT_EQ(deg_neg_l > deg_zero, false);
  EXPECT_EQ(deg_neg_l > deg_pos_s, false);
  EXPECT_EQ(deg_neg_l > deg_pos_l, false);

  EXPECT_EQ(deg_neg_s > deg_neg_l, true);
  EXPECT_EQ(deg_neg_s > deg_neg_s, false);
  EXPECT_EQ(deg_neg_s > deg_zero, false);
  EXPECT_EQ(deg_neg_s > deg_pos_s, false);
  EXPECT_EQ(deg_neg_s > deg_pos_l, false);

  EXPECT_EQ(deg_zero > deg_neg_l, true);
  EXPECT_EQ(deg_zero > deg_neg_s, true);
  EXPECT_EQ(deg_zero > deg_zero, false);
  EXPECT_EQ(deg_zero > deg_pos_s, false);
  EXPECT_EQ(deg_zero > deg_pos_l, false);

  EXPECT_EQ(deg_pos_s > deg_neg_l, true);
  EXPECT_EQ(deg_pos_s > deg_neg_s, true);
  EXPECT_EQ(deg_pos_s > deg_zero, true);
  EXPECT_EQ(deg_pos_s > deg_pos_s, false);
  EXPECT_EQ(deg_pos_s > deg_pos_l, false);

  EXPECT_EQ(deg_pos_l > deg_neg_l, true);
  EXPECT_EQ(deg_pos_l > deg_neg_s, true);
  EXPECT_EQ(deg_pos_l > deg_zero, true);
  EXPECT_EQ(deg_pos_l > deg_pos_s, true);
  EXPECT_EQ(deg_pos_l > deg_pos_l, false);
}

TEST(PSENscanInternalAngleTest, operator_smaller_than_or_equal)
{
  PSENscanInternalAngle deg_neg_l(-100);
  PSENscanInternalAngle deg_neg_s(-1);
  PSENscanInternalAngle deg_zero(0);
  PSENscanInternalAngle deg_pos_s(1);
  PSENscanInternalAngle deg_pos_l(100);

  EXPECT_EQ(deg_neg_l <= deg_neg_l, true);
  EXPECT_EQ(deg_neg_l <= deg_neg_s, true);
  EXPECT_EQ(deg_neg_l <= deg_zero, true);
  EXPECT_EQ(deg_neg_l <= deg_pos_s, true);
  EXPECT_EQ(deg_neg_l <= deg_pos_l, true);

  EXPECT_EQ(deg_neg_s <= deg_neg_l, false);
  EXPECT_EQ(deg_neg_s <= deg_neg_s, true);
  EXPECT_EQ(deg_neg_s <= deg_zero, true);
  EXPECT_EQ(deg_neg_s <= deg_pos_s, true);
  EXPECT_EQ(deg_neg_s <= deg_pos_l, true);

  EXPECT_EQ(deg_zero <= deg_neg_l, false);
  EXPECT_EQ(deg_zero <= deg_neg_s, false);
  EXPECT_EQ(deg_zero <= deg_zero, true);
  EXPECT_EQ(deg_zero <= deg_pos_s, true);
  EXPECT_EQ(deg_zero <= deg_pos_l, true);

  EXPECT_EQ(deg_pos_s <= deg_neg_l, false);
  EXPECT_EQ(deg_pos_s <= deg_neg_s, false);
  EXPECT_EQ(deg_pos_s <= deg_zero, false);
  EXPECT_EQ(deg_pos_s <= deg_pos_s, true);
  EXPECT_EQ(deg_pos_s <= deg_pos_l, true);

  EXPECT_EQ(deg_pos_l <= deg_neg_l, false);
  EXPECT_EQ(deg_pos_l <= deg_neg_s, false);
  EXPECT_EQ(deg_pos_l <= deg_zero, false);
  EXPECT_EQ(deg_pos_l <= deg_pos_s, false);
  EXPECT_EQ(deg_pos_l <= deg_pos_l, true);
}

TEST(PSENscanInternalAngleTest, operator_greater_than_or_equal)
{
  PSENscanInternalAngle deg_neg_l(-100);
  PSENscanInternalAngle deg_neg_s(-1);
  PSENscanInternalAngle deg_zero(0);
  PSENscanInternalAngle deg_pos_s(1);
  PSENscanInternalAngle deg_pos_l(100);

  EXPECT_EQ(deg_neg_l >= deg_neg_l, true);
  EXPECT_EQ(deg_neg_l >= deg_neg_s, false);
  EXPECT_EQ(deg_neg_l >= deg_zero, false);
  EXPECT_EQ(deg_neg_l >= deg_pos_s, false);
  EXPECT_EQ(deg_neg_l >= deg_pos_l, false);

  EXPECT_EQ(deg_neg_s >= deg_neg_l, true);
  EXPECT_EQ(deg_neg_s >= deg_neg_s, true);
  EXPECT_EQ(deg_neg_s >= deg_zero, false);
  EXPECT_EQ(deg_neg_s >= deg_pos_s, false);
  EXPECT_EQ(deg_neg_s >= deg_pos_l, false);

  EXPECT_EQ(deg_zero >= deg_neg_l, true);
  EXPECT_EQ(deg_zero >= deg_neg_s, true);
  EXPECT_EQ(deg_zero >= deg_zero, true);
  EXPECT_EQ(deg_zero >= deg_pos_s, false);
  EXPECT_EQ(deg_zero >= deg_pos_l, false);

  EXPECT_EQ(deg_pos_s >= deg_neg_l, true);
  EXPECT_EQ(deg_pos_s >= deg_neg_s, true);
  EXPECT_EQ(deg_pos_s >= deg_zero, true);
  EXPECT_EQ(deg_pos_s >= deg_pos_s, true);
  EXPECT_EQ(deg_pos_s >= deg_pos_l, false);

  EXPECT_EQ(deg_pos_l >= deg_neg_l, true);
  EXPECT_EQ(deg_pos_l >= deg_neg_s, true);
  EXPECT_EQ(deg_pos_l >= deg_zero, true);
  EXPECT_EQ(deg_pos_l >= deg_pos_s, true);
  EXPECT_EQ(deg_pos_l >= deg_pos_l, true);
}

TEST(PSENscanInternalAngleTest, operator_is_equal)
{
  PSENscanInternalAngle deg_neg(-100);
  PSENscanInternalAngle deg_zero(0);
  PSENscanInternalAngle deg_pos(100);

  EXPECT_EQ(deg_neg == deg_neg, true);
  EXPECT_EQ(deg_neg == deg_zero, false);
  EXPECT_EQ(deg_neg == deg_pos, false);

  EXPECT_EQ(deg_zero == deg_neg, false);
  EXPECT_EQ(deg_zero == deg_zero, true);
  EXPECT_EQ(deg_zero == deg_pos, false);

  EXPECT_EQ(deg_pos == deg_neg, false);
  EXPECT_EQ(deg_pos == deg_zero, false);
  EXPECT_EQ(deg_pos == deg_pos, true);
}

TEST(PSENscanInternalAngleTest, operator_is_not_equal)
{
  PSENscanInternalAngle deg_neg(-100);
  PSENscanInternalAngle deg_zero(0);
  PSENscanInternalAngle deg_pos(100);

  EXPECT_EQ(deg_neg != deg_neg, false);
  EXPECT_EQ(deg_neg != deg_zero, true);
  EXPECT_EQ(deg_neg != deg_pos, true);

  EXPECT_EQ(deg_zero != deg_neg, true);
  EXPECT_EQ(deg_zero != deg_zero, false);
  EXPECT_EQ(deg_zero != deg_pos, true);

  EXPECT_EQ(deg_pos != deg_neg, true);
  EXPECT_EQ(deg_pos != deg_zero, true);
  EXPECT_EQ(deg_pos != deg_pos, false);
}

TEST(PSENscanInternalAngleTest, operator_minus_assignment)
{
  std::srand(1);

  for (int i = 0; i < 1000; i++)
  {
    int minuend_temp = std::rand() - RAND_MAX / 2;
    int subtrahend_temp = std::rand() - RAND_MAX / 2;
    int difference = minuend_temp - subtrahend_temp;
    PSENscanInternalAngle minuend(minuend_temp);
    PSENscanInternalAngle subtrahend(subtrahend_temp);
    ASSERT_EQ(static_cast<int>(minuend), minuend_temp);
    ASSERT_EQ(static_cast<int>(subtrahend), subtrahend_temp);
    minuend -= subtrahend;
    EXPECT_EQ(static_cast<int>(minuend), difference);
    minuend -= PSENscanInternalAngle(0);
    EXPECT_EQ(minuend, minuend);
  }
}

TEST(PSENscanInternalAngleTest, operator_minus)
{
  std::srand(1);

  for (int i = 0; i < 1000; i++)
  {
    int minuend_temp = std::rand() - RAND_MAX / 2;
    int subtrahend_temp = std::rand() - RAND_MAX / 2;
    int difference = minuend_temp - subtrahend_temp;
    PSENscanInternalAngle minuend(minuend_temp);
    PSENscanInternalAngle subtrahend(subtrahend_temp);
    ASSERT_EQ(static_cast<int>(minuend), minuend_temp);
    ASSERT_EQ(static_cast<int>(subtrahend), subtrahend_temp);
    EXPECT_EQ(static_cast<int>(minuend - subtrahend), difference);
    EXPECT_EQ(minuend - PSENscanInternalAngle(0), minuend);
    EXPECT_EQ(PSENscanInternalAngle(0) - subtrahend, -subtrahend);
  }
}

TEST(PSENscanInternalAngleTest, operator_plus_assignment)
{
  std::srand(1);

  for (int i = 0; i < 1000; i++)
  {
    int augend_temp = std::rand() - RAND_MAX / 2;
    int addend_temp = std::rand() - RAND_MAX / 2;
    int sum = augend_temp + addend_temp;
    PSENscanInternalAngle augend(augend_temp);
    PSENscanInternalAngle addend(addend_temp);
    ASSERT_EQ(static_cast<int>(augend), augend_temp);
    ASSERT_EQ(static_cast<int>(addend), addend_temp);
    augend += addend;
    EXPECT_EQ(static_cast<int>(augend), sum);
    augend += PSENscanInternalAngle(0);
    EXPECT_EQ(augend, augend);
  }
}

TEST(PSENscanInternalAngleTest, operator_plus)
{
  std::srand(1);

  for (int i = 0; i < 1000; i++)
  {
    int augend_temp = std::rand() - RAND_MAX / 2;
    int addend_temp = std::rand() - RAND_MAX / 2;
    int sum = augend_temp + addend_temp;
    PSENscanInternalAngle augend(augend_temp);
    PSENscanInternalAngle addend(addend_temp);
    ASSERT_EQ(static_cast<int>(augend), augend_temp);
    ASSERT_EQ(static_cast<int>(addend), addend_temp);
    EXPECT_EQ(static_cast<int>(augend + addend), sum);
    EXPECT_EQ(augend + PSENscanInternalAngle(0), augend);
    EXPECT_EQ(PSENscanInternalAngle(0) + addend, addend);
  }
}

TEST(PSENscanInternalAngleTest, operator_assignment)
{
  std::srand(1);

  for (int i = 0; i < 1000; i++)
  {
    double temp1 = std::rand() - RAND_MAX / 2;
    double temp2 = std::rand() - RAND_MAX / 2;
    PSENscanInternalAngle deg1(temp1);
    PSENscanInternalAngle deg2(temp2);
    ASSERT_EQ(static_cast<int>(deg1), temp1);
    ASSERT_EQ(static_cast<int>(deg2), temp2);
    deg1 = deg2;
    EXPECT_EQ(deg1, deg2);
  }
}

TEST(PSENscanInternalAngleTest, operator_stream)
{
  std::stringstream expected_string;
  expected_string << "123";

  std::stringstream ss;
  ss << PSENscanInternalAngle(123);

  EXPECT_EQ(ss.str(), expected_string.str());
}