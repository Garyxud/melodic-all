/*********************************************************************
 * C++ unit test for dbw_mkz_can/sonar_lut.h
 *********************************************************************/

#include <gtest/gtest.h>

// File under test
#include <dbw_mkz_can/sonar_lut.h>
using namespace dbw_mkz_can;

// Test converting sonar ranges
TEST(sonar_lut, sonarMetersFromBits)
{
  EXPECT_EQ((float) 0.00, sonarMetersFromBits(0));
  EXPECT_EQ((float) 0.30, sonarMetersFromBits(1));
  EXPECT_EQ((float) 0.45, sonarMetersFromBits(2));
  EXPECT_EQ((float) 0.60, sonarMetersFromBits(3));
  EXPECT_EQ((float) 0.75, sonarMetersFromBits(4));
  EXPECT_EQ((float) 0.90, sonarMetersFromBits(5));
  EXPECT_EQ((float) 1.05, sonarMetersFromBits(6));
  EXPECT_EQ((float) 1.20, sonarMetersFromBits(7));
  EXPECT_EQ((float) 1.35, sonarMetersFromBits(8));
  EXPECT_EQ((float) 1.50, sonarMetersFromBits(9));
  EXPECT_EQ((float) 1.65, sonarMetersFromBits(10));
  EXPECT_EQ((float) 1.80, sonarMetersFromBits(11));
  EXPECT_EQ((float) 1.95, sonarMetersFromBits(12));
  EXPECT_EQ((float) 2.10, sonarMetersFromBits(13));
  EXPECT_EQ((float) 2.25, sonarMetersFromBits(14));
  EXPECT_EQ((float) 2.40, sonarMetersFromBits(15));
  EXPECT_EQ((float)15.15, sonarMetersFromBits(100));
}

// Test converting sonar colors
TEST(sonar_lut, sonarColorFromRange)
{
  const uint32_t RED = 0xC0FF0000; // rgba = RED
  const uint32_t YLW = 0xC0FFFF00; // rgba = YELLOW
  const uint32_t GRN = 0xC000FF00; // rgba = GREEN

  EXPECT_EQ(RED, sonarColorFromRange(-INFINITY));
  EXPECT_EQ(RED, sonarColorFromRange(0.00));
  EXPECT_EQ(RED, sonarColorFromRange(0.30));
  EXPECT_EQ(RED, sonarColorFromRange(0.45));
  EXPECT_EQ(RED, sonarColorFromRange(0.60));
  EXPECT_EQ(YLW, sonarColorFromRange(0.75));
  EXPECT_EQ(YLW, sonarColorFromRange(0.90));
  EXPECT_EQ(YLW, sonarColorFromRange(1.05));
  EXPECT_EQ(YLW, sonarColorFromRange(1.20));
  EXPECT_EQ(GRN, sonarColorFromRange(1.35));
  EXPECT_EQ(GRN, sonarColorFromRange(1.50));
  EXPECT_EQ(GRN, sonarColorFromRange(INFINITY));
}

///@TODO: Create test for sonarBuildPointCloud2()

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

