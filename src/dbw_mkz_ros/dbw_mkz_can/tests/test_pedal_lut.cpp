/*********************************************************************
 * C++ unit test for dbw_mkz_can/pedal_lut.h
 *********************************************************************/

#include <gtest/gtest.h>

// File under test
#include <dbw_mkz_can/pedal_lut.h>
using namespace dbw_mkz_can;

// Test converting from brake pedal position to brake torque
TEST(pedal_lut, brakeTorqueFromPedal)
{
  // Out of range
  EXPECT_EQ((float)   0, brakeTorqueFromPedal(-INFINITY));
  EXPECT_EQ((float)   0, brakeTorqueFromPedal(-1.00));
  EXPECT_EQ((float)   0, brakeTorqueFromPedal(0.000));
  EXPECT_EQ((float)3412, brakeTorqueFromPedal(1.000));
  EXPECT_EQ((float)3412, brakeTorqueFromPedal(2.000));
  EXPECT_EQ((float)3412, brakeTorqueFromPedal(INFINITY));

  // Extreme values
  EXPECT_EQ((float)   0, brakeTorqueFromPedal(0.150));
  EXPECT_EQ((float)   0, brakeTorqueFromPedal(0.160));
  EXPECT_EQ((float)3412, brakeTorqueFromPedal(0.328));
  EXPECT_EQ((float)3412, brakeTorqueFromPedal(0.330));

  // Normal values
  EXPECT_NEAR((float) 520.0, brakeTorqueFromPedal(0.22), (float)1.0);
  EXPECT_NEAR((float) 668.5, brakeTorqueFromPedal(0.24), (float)1.0);
  EXPECT_NEAR((float)1749.5, brakeTorqueFromPedal(0.28), (float)1.0);
  EXPECT_NEAR((float)2435.0, brakeTorqueFromPedal(0.30), (float)1.0);
}

// Test converting from brake torque to brake pedal position
TEST(pedal_lut, brakePedalFromTorque)
{
  // Out of range
  EXPECT_EQ((float)0.150, brakePedalFromTorque(-INFINITY));
  EXPECT_EQ((float)0.150, brakePedalFromTorque(-1));
  EXPECT_EQ((float)0.330, brakePedalFromTorque(9999));
  EXPECT_EQ((float)0.330, brakePedalFromTorque(INFINITY));

  // Extreme values
  EXPECT_EQ((float)0.150, brakePedalFromTorque(0));
  EXPECT_EQ((float)0.330, brakePedalFromTorque(3412));

  // Jump from zero to non-zero
  EXPECT_NEAR((float)0.150, brakePedalFromTorque(0.0000), (float)0.0000);
  EXPECT_NEAR((float)0.175, brakePedalFromTorque(0.0001), (float)0.0001);

  // Jump near max
  EXPECT_NEAR((float)0.326, brakePedalFromTorque(3411.999), (float)0.0001);
  EXPECT_NEAR((float)0.330, brakePedalFromTorque(3412.000), (float)0.0000);

  // Normal values
  EXPECT_NEAR((float)0.195, brakePedalFromTorque(  50), (float)0.001);
  EXPECT_NEAR((float)0.253, brakePedalFromTorque(1000), (float)0.001);
  EXPECT_NEAR((float)0.288, brakePedalFromTorque(2000), (float)0.001);
}

// Test converting from brake percent to brake pedal position
TEST(pedal_lut, brakePedalFromPercent)
{
  // Out of range
  EXPECT_EQ((float)0.150, brakePedalFromPercent(-INFINITY));
  EXPECT_EQ((float)0.150, brakePedalFromPercent(-1));
  EXPECT_EQ((float)0.330, brakePedalFromPercent(2));
  EXPECT_EQ((float)0.330, brakePedalFromPercent(INFINITY));

  // Extreme values
  EXPECT_EQ((float)0.150, brakePedalFromPercent(0));
  EXPECT_EQ((float)0.330, brakePedalFromPercent(1));

  // Jump from zero to non-zero
  EXPECT_NEAR((float)0.150, brakePedalFromPercent(0.000000), (float)0.0000);
  EXPECT_NEAR((float)0.175, brakePedalFromPercent(0.000001), (float)0.0001);

  // Jump near max
  EXPECT_NEAR((float)0.326, brakePedalFromPercent(0.9999), (float)0.0001);
  EXPECT_NEAR((float)0.330, brakePedalFromPercent(1.0000), (float)0.0000);

  // Normal values
  EXPECT_NEAR((float)0.254, brakePedalFromPercent(0.3), (float)0.001);
  EXPECT_NEAR((float)0.278, brakePedalFromPercent(0.5), (float)0.001);
  EXPECT_NEAR((float)0.299, brakePedalFromPercent(0.7), (float)0.001);
}

// Test converting from throttle percent to throttle pedal position
TEST(pedal_lut, throttlePedalFromPercent)
{
  // Out of range
  EXPECT_EQ((float)0.150, throttlePedalFromPercent(-INFINITY));
  EXPECT_EQ((float)0.150, throttlePedalFromPercent(-1));
  EXPECT_EQ((float)0.800, throttlePedalFromPercent(2));
  EXPECT_EQ((float)0.800, throttlePedalFromPercent(INFINITY));

  // Extreme values
  EXPECT_EQ((float)0.150, throttlePedalFromPercent(0));
  EXPECT_EQ((float)0.800, throttlePedalFromPercent(1));

  // Jump from zero to non-zero
  EXPECT_EQ((float)0.150, throttlePedalFromPercent(0.000));
  EXPECT_EQ((float)0.165, throttlePedalFromPercent(0.001));

  // Normal values
  EXPECT_NEAR((float)0.347, throttlePedalFromPercent(0.3), (float)0.001);
  EXPECT_NEAR((float)0.477, throttlePedalFromPercent(0.5), (float)0.001);
  EXPECT_NEAR((float)0.606, throttlePedalFromPercent(0.7), (float)0.001);
}

// Test converting from throttle pedal position to throttle percent
TEST(pedal_lut, throttlePercentFromPedal)
{
  // Out of range
  EXPECT_EQ((float)0.0, throttlePercentFromPedal(-INFINITY));
  EXPECT_EQ((float)0.0, throttlePercentFromPedal(-1));
  EXPECT_EQ((float)1.0, throttlePercentFromPedal(1));
  EXPECT_EQ((float)1.0, throttlePercentFromPedal(INFINITY));

  // Extreme values
  EXPECT_EQ((float)0.0, throttlePercentFromPedal(0.15));
  EXPECT_EQ((float)1.0, throttlePercentFromPedal(0.80));

  // Normal values
  EXPECT_NEAR((float)0.3, throttlePercentFromPedal(0.347), (float)0.001);
  EXPECT_NEAR((float)0.5, throttlePercentFromPedal(0.477), (float)0.001);
  EXPECT_NEAR((float)0.7, throttlePercentFromPedal(0.606), (float)0.001);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

