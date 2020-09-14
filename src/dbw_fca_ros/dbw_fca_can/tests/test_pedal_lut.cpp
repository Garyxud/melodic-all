/*********************************************************************
 * C++ unit test for dbw_fca_can/pedal_lut.h
 *********************************************************************/

#include <gtest/gtest.h>

// File under test
#include <dbw_fca_can/pedal_lut.h>
using namespace dbw_fca_can;

// Test converting from brake pedal position to brake torque
TEST(pedal_lut, brakeTorqueFromPedal)
{
  // Out of range
  EXPECT_EQ((float)   0, brakeTorqueFromPedal(-INFINITY));
  EXPECT_EQ((float)   0, brakeTorqueFromPedal(-1.00));
  EXPECT_EQ((float)   0, brakeTorqueFromPedal(0.000));
  EXPECT_EQ((float)6888, brakeTorqueFromPedal(1.000));
  EXPECT_EQ((float)6888, brakeTorqueFromPedal(2.000));
  EXPECT_EQ((float)6888, brakeTorqueFromPedal(INFINITY));

  // Extreme values
  EXPECT_EQ((float)   0, brakeTorqueFromPedal(0.150));
  EXPECT_EQ((float)   0, brakeTorqueFromPedal(0.160));
  EXPECT_EQ((float)6888, brakeTorqueFromPedal(0.570));
  EXPECT_EQ((float)6888, brakeTorqueFromPedal(0.600));

  // Normal values
  EXPECT_NEAR((float) 166.4, brakeTorqueFromPedal(0.22), (float)1.0);
  EXPECT_NEAR((float) 969.6, brakeTorqueFromPedal(0.28), (float)1.0);
  EXPECT_NEAR((float)1728.0, brakeTorqueFromPedal(0.32), (float)1.0);
  EXPECT_NEAR((float)3170.4, brakeTorqueFromPedal(0.38), (float)1.0);
}

// Test converting from brake torque to brake pedal position
TEST(pedal_lut, brakePedalFromTorque)
{
  // Out of range
  EXPECT_EQ((float)0.150, brakePedalFromTorque(-INFINITY));
  EXPECT_EQ((float)0.150, brakePedalFromTorque(-1));
  EXPECT_EQ((float)0.600, brakePedalFromTorque(9999));
  EXPECT_EQ((float)0.600, brakePedalFromTorque(INFINITY));

  // Extreme values
  EXPECT_EQ((float)0.150, brakePedalFromTorque(0));
  EXPECT_EQ((float)0.600, brakePedalFromTorque(6888));

  // Jump from zero to non-zero
  EXPECT_NEAR((float)0.150, brakePedalFromTorque(0.0000), (float)0.0000);
  EXPECT_NEAR((float)0.166, brakePedalFromTorque(0.0001), (float)0.0001);

  // Jump near max
  EXPECT_NEAR((float)0.566, brakePedalFromTorque(6887.999), (float)0.0001);
  EXPECT_NEAR((float)0.600, brakePedalFromTorque(6888.000), (float)0.0000);

  // Normal values
  EXPECT_NEAR((float)0.196, brakePedalFromTorque(  50), (float)0.001);
  EXPECT_NEAR((float)0.282, brakePedalFromTorque(1000), (float)0.001);
  EXPECT_NEAR((float)0.333, brakePedalFromTorque(2000), (float)0.001);
}

// Test converting from brake percent to brake pedal position
#if 0 ///@TODO
TEST(pedal_lut, brakePedalFromPercent)
{
  // Out of range
  EXPECT_EQ((float)0.150, brakePedalFromPercent(-INFINITY));
  EXPECT_EQ((float)0.150, brakePedalFromPercent(-1));
  EXPECT_EQ((float)0.600, brakePedalFromPercent(2));
  EXPECT_EQ((float)0.600, brakePedalFromPercent(INFINITY));

  // Extreme values
  EXPECT_EQ((float)0.150, brakePedalFromPercent(0));
  EXPECT_EQ((float)0.600, brakePedalFromPercent(1));

  // Jump from zero to non-zero
  EXPECT_NEAR((float)0.150, brakePedalFromPercent(0.000000), (float)0.0000);
  EXPECT_NEAR((float)0.600, brakePedalFromPercent(0.000001), (float)0.0001);

  // Jump near max
  EXPECT_NEAR((float)0.566, brakePedalFromPercent(0.9999), (float)0.0001);
  EXPECT_NEAR((float)0.600, brakePedalFromPercent(1.0000), (float)0.0000);

  // Normal values
  EXPECT_NEAR((float)0.000, brakePedalFromPercent(0.3), (float)0.001);
  EXPECT_NEAR((float)0.000, brakePedalFromPercent(0.5), (float)0.001);
  EXPECT_NEAR((float)0.000, brakePedalFromPercent(0.7), (float)0.001);
}
#endif

// Test converting from throttle percent to throttle pedal position
TEST(pedal_lut, throttlePedalFromPercent)
{
  // Out of range
  EXPECT_EQ((float)0.080, throttlePedalFromPercent(-INFINITY));
  EXPECT_EQ((float)0.080, throttlePedalFromPercent(-1));
  EXPECT_EQ((float)0.892, throttlePedalFromPercent(2));
  EXPECT_EQ((float)0.892, throttlePedalFromPercent(INFINITY));

  // Extreme values
  EXPECT_EQ((float)0.080, throttlePedalFromPercent(0));
  EXPECT_EQ((float)0.892, throttlePedalFromPercent(1));

  // Jump from zero to non-zero
  EXPECT_EQ((float)0.080, throttlePedalFromPercent(0.000));
  EXPECT_EQ((float)0.114, throttlePedalFromPercent(0.001));

  // Normal values
  EXPECT_NEAR((float)0.343, throttlePedalFromPercent(0.3), (float)0.001);
  EXPECT_NEAR((float)0.497, throttlePedalFromPercent(0.5), (float)0.001);
  EXPECT_NEAR((float)0.655, throttlePedalFromPercent(0.7), (float)0.001);
}

// Test converting from throttle pedal position to throttle percent
TEST(pedal_lut, throttlePercentFromPedal)
{
  // Out of range
  EXPECT_EQ((float)0.0, throttlePercentFromPedal(-INFINITY));
  EXPECT_EQ((float)0.0, throttlePercentFromPedal(-1.0));
  EXPECT_EQ((float)1.0, throttlePercentFromPedal(1.0));
  EXPECT_EQ((float)1.0, throttlePercentFromPedal(INFINITY));

  // Extreme values
  EXPECT_EQ((float)0.0, throttlePercentFromPedal(0.080));
  EXPECT_EQ((float)1.0, throttlePercentFromPedal(0.892));

  // Normal values
  EXPECT_NEAR((float)0.3, throttlePercentFromPedal(0.343), (float)0.001);
  EXPECT_NEAR((float)0.5, throttlePercentFromPedal(0.497), (float)0.001);
  EXPECT_NEAR((float)0.7, throttlePercentFromPedal(0.655), (float)0.001);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

