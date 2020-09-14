#include <gtest/gtest.h>
#include <ixblue_stdbin_decoder/stdbin_decoder.h>

#include "datasets/log_STDBIN_V2.h"
#include "datasets/log_STDBIN_V3.h"
#include "datasets/log_STDBIN_V4.h"
#include "datasets/log_STDBIN_V5.h"

TEST(StdBinDecoder, WeCanParseAFrameWithSomeMissingFields)
{
    // Given a frame with only attitude :
    // clang-format off
    const std::vector<uint8_t> memory{
        'I',  'X',  /*IX blue header   */
        0x02,       /*Protocol Version */
        0x00, 0x00, 0x00, 0x01, /* navigation bitmask (0x00000001 means only AttitudeAndHeading) */
        0x00, 0x00, 0x00, 0x00, /* external data bitmask */
        0x00, 0x21, /* Telegram size */
        0x00, 0x00, 0x00, 0x05, /* navigation validity time (500 us) */
        0x00, 0x00, 0x01, 0x23, /* counter (0x123) */
        0x00, 0x00, 0xa0, 0x3f, /* Heading : 1.25f */
        0x00, 0x00, 0xc0, 0xbf, /* roll : -1.5f   */
        0xcd, 0xcc, 0x48, 0x41, /* Pitch : 12.55f */
    };
    // clang-format on

    ixblue_stdbin_decoder::StdBinDecoder parser;
    EXPECT_TRUE(parser.parse(memory));
    auto result = parser.getLastNavData();

    ASSERT_FALSE(result.attitudeHeadingDeviation.is_initialized());
    ASSERT_TRUE(result.attitudeHeading.is_initialized());
    // We don't test attitude heading is parsed with good values, we already have a test
    // for this.
}

TEST(StdBinDecoder, WeCanParseV2Protocol)
{
    ixblue_stdbin_decoder::StdBinDecoder parser;

    EXPECT_TRUE(parser.parse(log_STDBIN_V2));
    auto result = parser.getLastNavData();

    EXPECT_TRUE(result.attitudeHeading.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().heading_deg, 209.982);
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().roll_deg, 0.016);
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().pitch_deg, 0.206);

    EXPECT_TRUE(result.attitudeHeadingDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().heading_stdDev_deg, 2.4517534);
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().roll_stdDev_deg, 0.004243818);
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().pitch_stdDev_deg, 0.004769328);

    EXPECT_TRUE(result.rtHeaveSurgeSway.is_initialized());
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_heave_withoutBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_heave_atBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_surge_atBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_sway_atBdL, 0.0);

    EXPECT_TRUE(result.smartHeave.is_initialized());
    EXPECT_EQ(result.smartHeave.get().validityTime_100us, 9215311);
    EXPECT_FLOAT_EQ(result.smartHeave.get().smartHeave_m, 0.0);

    EXPECT_TRUE(result.headingRollPitchRate.is_initialized());
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().heading_rate, 0.0);
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().roll_rate, 0.0);
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().pitch_rate, -0.002);

    EXPECT_TRUE(result.rotationRateVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv1_degsec, 0.001287996);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv2_degsec, 0.0010229985);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv3_degsec, 0.0026694948);

    EXPECT_TRUE(result.accelerationVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv1_msec2, -0.00034000012);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv2_msec2, 0.000035000077);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv3_msec2, 0.000055000033);

    EXPECT_TRUE(result.position.is_initialized());
    EXPECT_DOUBLE_EQ(result.position.get().latitude_deg, 2.1332412116407853);
    EXPECT_DOUBLE_EQ(result.position.get().longitude_deg, 48.000037178805215);
    EXPECT_EQ(result.position.get().altitude_ref, 0);
    EXPECT_FLOAT_EQ(result.position.get().altitude_m, 1.0547082);

    EXPECT_TRUE(result.positionDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.positionDeviation.get().north_stddev_m, 14.192304);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().east_stddev_m, 14.168153);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().north_east_corr, 0.58203787);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().altitude_stddev_m, 7.6183596);

    EXPECT_TRUE(result.speedGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().north_msec, -0.0015761498);
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().east_msec, -0.027757198);
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().up_msec, 0.045121633);

    EXPECT_TRUE(result.speedGeographicFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().north_stddev_msec,
                    0.0114673115);
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().east_stddev_msec,
                    0.009463781);
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().up_stddev_msec,
                    0.029867083);

    EXPECT_TRUE(result.currentGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.currentGeographicFrame.get().north_msec, 0.0);
    EXPECT_FLOAT_EQ(result.currentGeographicFrame.get().east_msec, -0.0);

    EXPECT_TRUE(result.currentGeographicFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.currentGeographicFrameDeviation.get().north_stddev_msec, 0.0);
    EXPECT_FLOAT_EQ(result.currentGeographicFrameDeviation.get().east_stddev_msec, 0.0);

    EXPECT_TRUE(result.systemDate.is_initialized());
    EXPECT_EQ(result.systemDate.get().day, 1);
    EXPECT_EQ(result.systemDate.get().month, 1);
    EXPECT_EQ(result.systemDate.get().year, 2006);

    EXPECT_TRUE(result.sensorStatus.is_initialized());
    EXPECT_EQ(result.sensorStatus.get().status1, 0);
    EXPECT_EQ(result.sensorStatus.get().status2, 0);

    EXPECT_TRUE(result.insAlgorithmStatus.is_initialized());
    EXPECT_EQ(result.insAlgorithmStatus.get().status1, 0x12);
    EXPECT_EQ(result.insAlgorithmStatus.get().status2, 0x30000);
    EXPECT_EQ(result.insAlgorithmStatus.get().status3, 0x4000000);
    EXPECT_EQ(result.insAlgorithmStatus.get().status4, 0x200);

    EXPECT_TRUE(result.insSystemStatus.is_initialized());
    EXPECT_EQ(result.insSystemStatus.get().status1, 0x200);
    EXPECT_EQ(result.insSystemStatus.get().status2, 0x1000);
    EXPECT_EQ(result.insSystemStatus.get().status3, 0x0);

    EXPECT_TRUE(result.insUserStatus.is_initialized());
    EXPECT_EQ(result.insUserStatus.get().status, 0x4c000000);

    EXPECT_FALSE(result.ahrsAlgorithmStatus.is_initialized());

    EXPECT_FALSE(result.ahrsSystemStatus.is_initialized());

    EXPECT_FALSE(result.ahrsUserStatus.is_initialized());

    EXPECT_TRUE(result.heaveSurgeSwaySpeed.is_initialized());
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().realtime_heave_speed, 0.0);
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().surge_speed, 0.0);
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().sway_speed, 0.0);

    EXPECT_TRUE(result.speedVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv1_msec, 0.014972644);
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv2_msec, -0.023280365);
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv3_msec, 0.0449414);

    EXPECT_TRUE(result.accelerationGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().north_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().east_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().up_msec2, 9.809999);

    EXPECT_TRUE(result.courseSpeedoverGround.is_initialized());
    EXPECT_FLOAT_EQ(result.courseSpeedoverGround.get().course_over_ground, 266.96848);
    EXPECT_FLOAT_EQ(result.courseSpeedoverGround.get().speed_over_ground, 0.027778123);

    EXPECT_TRUE(result.temperatures.is_initialized());
    EXPECT_FLOAT_EQ(result.temperatures.get().mean_temp_fog, 26.158884);
    EXPECT_FLOAT_EQ(result.temperatures.get().mean_temp_acc, 26.158884);
    EXPECT_FLOAT_EQ(result.temperatures.get().board_temperature, 158.65643);

    EXPECT_FALSE(result.attitudeQuaternion.is_initialized()); /* From V3 */

    EXPECT_FALSE(result.attitudeQuaternionDeviation.is_initialized()); /* From V3 */

    EXPECT_FALSE(result.rawAccelerationVesselFrame.is_initialized()); /* From V3 */

    EXPECT_FALSE(result.accelerationVesselFrameDeviation.is_initialized()); /* From V3 */

    EXPECT_FALSE(result.rotationRateVesselFrameDeviation.is_initialized()); /* From V3 */
}

TEST(StdBinDecoder, WeCanParseV2ProtocolReceivedInTwoPartsWhereverCutpointIs)
{
    // By instanciating parser out of the loop, we also test that we can parse
    // multiple messages.
    ixblue_stdbin_decoder::StdBinDecoder parser;

    for(size_t i = 1; i < log_STDBIN_V2.size(); ++i)
    {
        std::vector<uint8_t> part1, part2;
        auto cutPoint = log_STDBIN_V2.begin();
        // we cut the packet everywhere:
        std::advance(cutPoint, i);
        std::copy(std::begin(log_STDBIN_V2), cutPoint, std::back_inserter(part1));
        std::copy(cutPoint, std::end(log_STDBIN_V2), std::back_inserter(part2));

        EXPECT_FALSE(parser.parse(part1));
        EXPECT_TRUE(parser.parse(part2));
        auto result = parser.getLastNavData();

        EXPECT_TRUE(result.attitudeHeading.is_initialized());
        EXPECT_FLOAT_EQ(result.attitudeHeading.get().heading_deg, 209.982);
        EXPECT_FLOAT_EQ(result.attitudeHeading.get().roll_deg, 0.016);
        EXPECT_FLOAT_EQ(result.attitudeHeading.get().pitch_deg, 0.206);
    }
}

TEST(StdBinDecoder, WeCanParseV3Protocol)
{
    ixblue_stdbin_decoder::StdBinDecoder parser;

    EXPECT_TRUE(parser.parse(log_STDBIN_V3));
    auto result = parser.getLastNavData();

    // --- Navigation data block
    EXPECT_TRUE(result.attitudeHeading.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().heading_deg, 0.0);
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().roll_deg, 0.809);
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().pitch_deg, -0.521);

    EXPECT_TRUE(result.attitudeHeadingDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().heading_stdDev_deg, 60.0);
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().roll_stdDev_deg, 4.9997573);
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().pitch_stdDev_deg, 4.999801);

    EXPECT_TRUE(result.rtHeaveSurgeSway.is_initialized());
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_heave_withoutBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_heave_atBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_surge_atBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_sway_atBdL, 0.0);

    EXPECT_TRUE(result.smartHeave.is_initialized());
    EXPECT_EQ(result.smartHeave.get().validityTime_100us, 565983535); // Sec
    EXPECT_FLOAT_EQ(result.smartHeave.get().smartHeave_m, 0.0);

    EXPECT_TRUE(result.headingRollPitchRate.is_initialized());
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().heading_rate, 0.0);
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().roll_rate, 0.0);
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().pitch_rate, -0.0029999998);

    EXPECT_TRUE(result.rotationRateVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv1_degsec, -0.0013599998);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv2_degsec, -0.0021450005);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv3_degsec, -0.000024999996);

    EXPECT_TRUE(result.accelerationVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv1_msec2, -0.0004);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv2_msec2, -0.00059999997);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv3_msec2, -0.07685006);

    EXPECT_TRUE(result.position.is_initialized());
    EXPECT_DOUBLE_EQ(result.position.get().latitude_deg, 48.899099791755994);
    EXPECT_DOUBLE_EQ(result.position.get().longitude_deg, 2.061999802664471);
    EXPECT_EQ(result.position.get().altitude_ref, 0);
    EXPECT_FLOAT_EQ(result.position.get().altitude_m, 3004.5398);

    EXPECT_TRUE(result.positionDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.positionDeviation.get().north_stddev_m, 50.0);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().east_stddev_m, 50.0);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().north_east_corr, -0.00005929864);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().altitude_stddev_m, 1.1056086);

    EXPECT_TRUE(result.speedGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().north_msec, -0.018215187);
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().east_msec, -0.012175002);
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().up_msec, 0.10336427);

    EXPECT_TRUE(result.speedGeographicFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().north_stddev_msec,
                    12.129728);
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().east_stddev_msec,
                    12.129744);
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().up_stddev_msec,
                    0.78773665);

    EXPECT_TRUE(result.currentGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.currentGeographicFrame.get().north_msec, 1.919778 * 1e-14);
    EXPECT_FLOAT_EQ(result.currentGeographicFrame.get().east_msec, 4.5171327 * 1e-15);

    EXPECT_TRUE(result.currentGeographicFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.currentGeographicFrameDeviation.get().north_stddev_msec,
                    0.50110954);
    EXPECT_FLOAT_EQ(result.currentGeographicFrameDeviation.get().east_stddev_msec,
                    0.5011094);

    EXPECT_TRUE(result.systemDate.is_initialized());
    EXPECT_EQ(result.systemDate.get().day, 14);
    EXPECT_EQ(result.systemDate.get().month, 3);
    EXPECT_EQ(result.systemDate.get().year, 2019);

    EXPECT_TRUE(result.sensorStatus.is_initialized());
    EXPECT_EQ(result.sensorStatus.get().status1, 0x0);
    EXPECT_EQ(result.sensorStatus.get().status2, 0x100);

    EXPECT_TRUE(result.insAlgorithmStatus.is_initialized());
    EXPECT_EQ(result.insAlgorithmStatus.get().status1, 0x81013112);
    EXPECT_EQ(result.insAlgorithmStatus.get().status2, 0x1003011);
    EXPECT_EQ(result.insAlgorithmStatus.get().status3, 0x4000100);
    EXPECT_EQ(result.insAlgorithmStatus.get().status4, 0x200);

    EXPECT_TRUE(result.insSystemStatus.is_initialized());
    EXPECT_EQ(result.insSystemStatus.get().status1, 0x08011e00);
    EXPECT_EQ(result.insSystemStatus.get().status2, 0x8eff);
    EXPECT_EQ(result.insSystemStatus.get().status3, 0x0);

    EXPECT_TRUE(result.insUserStatus.is_initialized());
    EXPECT_EQ(result.insUserStatus.get().status, 0x4c001102);

    EXPECT_FALSE(result.ahrsAlgorithmStatus.is_initialized());

    EXPECT_FALSE(result.ahrsSystemStatus.is_initialized());

    EXPECT_FALSE(result.ahrsUserStatus.is_initialized());

    EXPECT_TRUE(result.heaveSurgeSwaySpeed.is_initialized());
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().realtime_heave_speed, 0.0);
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().surge_speed, 0.0);
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().sway_speed, 0.0);

    EXPECT_TRUE(result.speedVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv1_msec, -0.017242743);
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv2_msec, 0.013600622);
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv3_msec, 0.10299126);

    EXPECT_TRUE(result.accelerationGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().north_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().east_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().up_msec2, 9.83);

    EXPECT_TRUE(result.courseSpeedoverGround.is_initialized());
    EXPECT_FLOAT_EQ(result.courseSpeedoverGround.get().course_over_ground, 213.74023);
    EXPECT_FLOAT_EQ(result.courseSpeedoverGround.get().speed_over_ground, 0.021863433);

    EXPECT_TRUE(result.temperatures.is_initialized());
    EXPECT_FLOAT_EQ(result.temperatures.get().mean_temp_fog, 28.700487);
    EXPECT_FLOAT_EQ(result.temperatures.get().mean_temp_acc, 37.533264);
    EXPECT_FLOAT_EQ(result.temperatures.get().board_temperature, 47.899994);

    EXPECT_TRUE(result.attitudeQuaternion.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q0, 0.9999646544456482);
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q1, -0.00706720445305109);
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q2, 0.00454969797283411);
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q3, -0.00003185356399626471);

    EXPECT_TRUE(result.attitudeQuaternionDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeQuaternionDeviation.get().quat_stddev_xi1,
                    0.08726222813129425);
    EXPECT_FLOAT_EQ(result.attitudeQuaternionDeviation.get().quat_stddev_xi2,
                    0.08726298809051514);
    EXPECT_FLOAT_EQ(result.attitudeQuaternionDeviation.get().quat_stddev_xi3,
                    1.0471975803375244);

    EXPECT_TRUE(result.rawAccelerationVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rawAccelerationVesselFrame.get().xv1_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.rawAccelerationVesselFrame.get().xv2_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.rawAccelerationVesselFrame.get().xv3_msec2, 7.999994);

    EXPECT_TRUE(result.accelerationVesselFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.accelerationVesselFrameDeviation.get().xv1_stddev_msec2,
                    16.82827);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrameDeviation.get().xv2_stddev_msec2,
                    16.827307);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrameDeviation.get().xv3_stddev_msec2,
                    1.1194783);

    EXPECT_TRUE(result.rotationRateVesselFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrameDeviation.get().xv1_stddev_degsec,
                    0.00008998095);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrameDeviation.get().xv2_stddev_degsec,
                    0.0029352242);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrameDeviation.get().xv3_stddev_degsec,
                    0.0032638267);

    // --- Extended navigation data block
    EXPECT_TRUE(result.rotationAccelerationVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rotationAccelerationVesselFrame.get().xv1_degsec2,
                    0.0003888396);
    EXPECT_FLOAT_EQ(result.rotationAccelerationVesselFrame.get().xv2_degsec2,
                    0.0017579537);
    EXPECT_FLOAT_EQ(result.rotationAccelerationVesselFrame.get().xv3_degsec2,
                    0.0013762031);

    EXPECT_TRUE(result.rotationAccelerationVesselFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(
        result.rotationAccelerationVesselFrameDeviation.get().xv1_stddev_degsec2,
        0.0000141353985);
    EXPECT_FLOAT_EQ(
        result.rotationAccelerationVesselFrameDeviation.get().xv2_stddev_degsec2,
        0.000014134977);
    EXPECT_FLOAT_EQ(
        result.rotationAccelerationVesselFrameDeviation.get().xv3_stddev_degsec2,
        0.000014134976);

    EXPECT_TRUE(result.rawRotationRateVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rawRotationRateVesselFrame.get().xv1_degsec, 0.0);
    EXPECT_FLOAT_EQ(result.rawRotationRateVesselFrame.get().xv2_degsec, 0.0);
    EXPECT_FLOAT_EQ(result.rawRotationRateVesselFrame.get().xv3_degsec, 0.0);

    EXPECT_FALSE(result.vehicleAttitudeHeading.is_initialized());

    EXPECT_FALSE(result.vehicleAttitudeHeadingDeviation.is_initialized());

    EXPECT_FALSE(result.vehiclePosition.is_initialized());

    EXPECT_FALSE(result.vehiclePositionDeviation.is_initialized());

    // --- External data block
    EXPECT_TRUE(result.utc.is_initialized());
    EXPECT_EQ(result.utc.get().validityTime_100us, 566980000); // sec
    EXPECT_EQ(result.utc.get().source, 0);

    EXPECT_TRUE(result.gnss1.is_initialized());
    EXPECT_EQ(result.gnss1.get().validityTime_100us, 566981130); // sec
    EXPECT_EQ(result.gnss1.get().gnss_id, 0);
    EXPECT_EQ(result.gnss1.get().gnss_quality, 4);
    EXPECT_DOUBLE_EQ(result.gnss1.get().latitude_deg, 48.89909998575846);
    EXPECT_DOUBLE_EQ(result.gnss1.get().longitude_deg, 2.062000000476837);
    EXPECT_FLOAT_EQ(result.gnss1.get().altitude_m, 3004.5);
    EXPECT_FLOAT_EQ(result.gnss1.get().latitude_stddev_m, 0.1);
    EXPECT_FLOAT_EQ(result.gnss1.get().longitude_stddev_m, 0.1);
    EXPECT_FLOAT_EQ(result.gnss1.get().altitude_stddev_m, 0.1);
    EXPECT_FLOAT_EQ(result.gnss1.get().lat_lon_stddev_m2, 0.0);
    EXPECT_FLOAT_EQ(result.gnss1.get().geoidal_separation_m, 0.0);

    EXPECT_TRUE(result.gnss2.is_initialized());
    EXPECT_EQ(result.gnss2.get().validityTime_100us, 566981130); // Sec
    EXPECT_EQ(result.gnss2.get().gnss_id, 1);
    EXPECT_EQ(result.gnss2.get().gnss_quality, 4);
    EXPECT_DOUBLE_EQ(result.gnss2.get().latitude_deg, 48.89909998575846);
    EXPECT_DOUBLE_EQ(result.gnss2.get().longitude_deg, 2.062000000476837);
    EXPECT_FLOAT_EQ(result.gnss2.get().altitude_m, 3004.5);
    EXPECT_FLOAT_EQ(result.gnss2.get().latitude_stddev_m, 0.1);
    EXPECT_FLOAT_EQ(result.gnss2.get().longitude_stddev_m, 0.2);
    EXPECT_FLOAT_EQ(result.gnss2.get().altitude_stddev_m, 0.3);
    EXPECT_FLOAT_EQ(result.gnss2.get().lat_lon_stddev_m2, -0.0);
    EXPECT_FLOAT_EQ(result.gnss2.get().geoidal_separation_m, 0.0);

    EXPECT_FALSE(result.gnssManual.is_initialized());

    EXPECT_TRUE(result.emlog1.is_initialized());
    EXPECT_EQ(result.emlog1.get().validityTime_100us, 566980110);
    EXPECT_EQ(result.emlog1.get().emlog_id, 0);
    EXPECT_FLOAT_EQ(result.emlog1.get().xv1_waterSpeed_ms, 60.0);
    EXPECT_FLOAT_EQ(result.emlog1.get().xv1_speed_stddev_ms, 0.01);

    EXPECT_FALSE(result.emlog2.is_initialized());

    EXPECT_TRUE(result.usbl1.is_initialized());
    EXPECT_EQ(result.usbl1.get().validityTime_100us, 566980558);
    EXPECT_EQ(result.usbl1.get().usbl_id, 0);
    std::array<uint8_t, 8> beacon_usbl1 = {
        {'A', 'B', 'C', '\0', '\0', '\0', '\0', '\0'}}; // TODO : VERIF
    EXPECT_EQ(result.usbl1.get().beacon_id, beacon_usbl1);
    EXPECT_EQ(result.usbl1.get().latitude_deg, 48.9);
    EXPECT_DOUBLE_EQ(result.usbl1.get().longitude_deg, 2.0639999985694883);
    EXPECT_DOUBLE_EQ(result.usbl1.get().altitude_m, 0.0);
    EXPECT_FLOAT_EQ(result.usbl1.get().north_stddev_m, 2.0);
    EXPECT_FLOAT_EQ(result.usbl1.get().east_stddev_m, 2.0);
    EXPECT_FLOAT_EQ(result.usbl1.get().lat_lon_cov_m2, 0.0);
    EXPECT_FLOAT_EQ(result.usbl1.get().altitude_stddev_m, 2.0);

    EXPECT_TRUE(result.usbl2.is_initialized());
    EXPECT_EQ(result.usbl2.get().validityTime_100us, 566980110);
    EXPECT_EQ(result.usbl2.get().usbl_id, 1);
    std::array<uint8_t, 8> beacon_usbl2 = {
        {'D', 'E', 'F', '\0', '\0', '\0', '\0', '\0'}}; // TODO : VERIF
    EXPECT_EQ(result.usbl2.get().beacon_id, beacon_usbl2);
    EXPECT_EQ(result.usbl2.get().latitude_deg, -48.898);
    EXPECT_DOUBLE_EQ(result.usbl2.get().longitude_deg, 289.5933);
    EXPECT_DOUBLE_EQ(result.usbl2.get().altitude_m, 0.0);
    EXPECT_FLOAT_EQ(result.usbl2.get().north_stddev_m, 10.0);
    EXPECT_FLOAT_EQ(result.usbl2.get().east_stddev_m, 10.0);
    EXPECT_FLOAT_EQ(result.usbl2.get().lat_lon_cov_m2, 0.0);
    EXPECT_FLOAT_EQ(result.usbl2.get().altitude_stddev_m, 10.0);

    EXPECT_FALSE(result.usbl3.is_initialized());

    EXPECT_TRUE(result.depth.is_initialized());
    EXPECT_EQ(result.depth.get().validityTime_100us, 566980110); // Sec
    EXPECT_EQ(result.depth.get().depth_m, -102.0);
    EXPECT_EQ(result.depth.get().depth_stddev_m, 100.0);

    EXPECT_TRUE(result.dvlGroundSpeed1.is_initialized());
    EXPECT_EQ(result.dvlGroundSpeed1.get().validityTime_100us, 566980110);
    EXPECT_EQ(result.dvlGroundSpeed1.get().dvl_id, 0);
    EXPECT_FLOAT_EQ(result.dvlGroundSpeed1.get().xv1_groundspeed_ms, 39.0);
    EXPECT_FLOAT_EQ(result.dvlGroundSpeed1.get().xv2_groundspeed_ms, 39.0);
    EXPECT_FLOAT_EQ(result.dvlGroundSpeed1.get().xv3_groundspeed_ms, 39.0);
    EXPECT_FLOAT_EQ(result.dvlGroundSpeed1.get().dvl_speedofsound_ms, 1400.0);
    EXPECT_FLOAT_EQ(result.dvlGroundSpeed1.get().dvl_altitude_m, 20.0);
    EXPECT_FLOAT_EQ(result.dvlGroundSpeed1.get().xv1_stddev_ms, 3.0);
    EXPECT_FLOAT_EQ(result.dvlGroundSpeed1.get().xv2_stddev_ms, 3.0);
    EXPECT_FLOAT_EQ(result.dvlGroundSpeed1.get().xv3_stddev_ms, 3.0);

    EXPECT_TRUE(result.dvlWaterSpeed1.is_initialized());
    EXPECT_EQ(result.dvlWaterSpeed1.get().validityTime_100us, 566980110);
    EXPECT_EQ(result.dvlWaterSpeed1.get().dvl_id, 0);
    EXPECT_FLOAT_EQ(result.dvlWaterSpeed1.get().xv1_waterspeed_ms, 18.571428);
    EXPECT_FLOAT_EQ(result.dvlWaterSpeed1.get().xv2_waterspeed_ms, 18.571428);
    EXPECT_FLOAT_EQ(result.dvlWaterSpeed1.get().xv3_waterspeed_ms, 18.571428);
    EXPECT_FLOAT_EQ(result.dvlWaterSpeed1.get().dvl_speedofsound_ms, 1400.0);
    EXPECT_FLOAT_EQ(result.dvlWaterSpeed1.get().xv1_stddev_ms, 3.0);
    EXPECT_FLOAT_EQ(result.dvlWaterSpeed1.get().xv2_stddev_ms, 3.0);
    EXPECT_FLOAT_EQ(result.dvlWaterSpeed1.get().xv3_stddev_ms, 3.0);

    EXPECT_TRUE(result.soundVelocity.is_initialized());
    // EXPECT_EQ(result.soundVelocity.get().validityTime_100us, 0);
    EXPECT_EQ(result.soundVelocity.get().ext_speedofsound_ms, 1300.0);

    EXPECT_FALSE(result.dmi.is_initialized());

    EXPECT_TRUE(result.lbl1.is_initialized());
    EXPECT_EQ(result.lbl1.get().validityTime_100us, 566980558);
    EXPECT_EQ(result.lbl1.get().rfu, 0);
    std::array<uint8_t, 8> beacon_lbl1 = {
        {'0', '\0', '\0', '\0', '\0', '\0', '\0', '\x1'}}; // TODO : verif
    EXPECT_EQ(result.lbl1.get().beacon_id, beacon_lbl1);
    EXPECT_DOUBLE_EQ(result.lbl1.get().beacon_latitude_deg, -48.89800001780192);
    EXPECT_DOUBLE_EQ(result.lbl1.get().beacon_longitude_deg, 289.5960000038147);
    EXPECT_FLOAT_EQ(result.lbl1.get().beacon_altitude_m, 0.0);
    EXPECT_FLOAT_EQ(result.lbl1.get().range_m, 238.0);
    EXPECT_FLOAT_EQ(result.lbl1.get().range_stddev_m, 10.0);

    EXPECT_FALSE(result.lbl2.is_initialized());

    EXPECT_FALSE(result.lbl3.is_initialized());

    EXPECT_FALSE(result.lbl4.is_initialized());

    EXPECT_FALSE(result.eventMarkerA.is_initialized());

    EXPECT_FALSE(result.eventMarkerB.is_initialized());

    EXPECT_FALSE(result.eventMarkerC.is_initialized());

    EXPECT_FALSE(result.dvlGroundSpeed2.is_initialized());

    EXPECT_FALSE(result.dvlWaterSpeed2.is_initialized());

    EXPECT_FALSE(result.turretAngles.is_initialized());

    EXPECT_FALSE(result.vtg1.is_initialized());

    EXPECT_FALSE(result.vtg2.is_initialized());

    EXPECT_FALSE(result.logBook.is_initialized());
}

TEST(StdBinDecoder, WeCanParseV4Protocol)
{
    ixblue_stdbin_decoder::StdBinDecoder parser;

    EXPECT_TRUE(parser.parse(log_STDBIN_V4));
    auto result = parser.getLastNavData();

    // --- Navigation data block
    EXPECT_TRUE(result.attitudeHeading.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().heading_deg, 357.428);
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().roll_deg, -0.04);
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().pitch_deg, 0.178);

    EXPECT_TRUE(result.attitudeHeadingDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().heading_stdDev_deg, 54.47794);
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().roll_stdDev_deg, 0.010677356);
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().pitch_stdDev_deg, 0.016798064);

    EXPECT_TRUE(result.rtHeaveSurgeSway.is_initialized());
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_heave_withoutBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_heave_atBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_surge_atBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_sway_atBdL, 0.0);

    EXPECT_TRUE(result.smartHeave.is_initialized());
    EXPECT_EQ(result.smartHeave.get().validityTime_100us, 8715306); // Sec
    EXPECT_FLOAT_EQ(result.smartHeave.get().smartHeave_m, 0.0);

    EXPECT_TRUE(result.headingRollPitchRate.is_initialized());
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().heading_rate, -0.002);
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().roll_rate, -0.005);
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().pitch_rate, 0.0);

    EXPECT_TRUE(result.rotationRateVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv1_degsec, -0.0061404924);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv2_degsec, -0.00091849366);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv3_degsec, 0.0024884976);

    EXPECT_TRUE(result.accelerationVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv1_msec2, 0.00004);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv2_msec2, 0.00040999975);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv3_msec2, 0.0007599999);

    EXPECT_TRUE(result.position.is_initialized());
    EXPECT_DOUBLE_EQ(result.position.get().latitude_deg, 2.133332706885363);
    EXPECT_DOUBLE_EQ(result.position.get().longitude_deg, 47.999999832595385);
    EXPECT_EQ(result.position.get().altitude_ref, 0);
    EXPECT_FLOAT_EQ(result.position.get().altitude_m, 0.15949036);

    EXPECT_TRUE(result.positionDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.positionDeviation.get().north_stddev_m, 14.204068);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().east_stddev_m, 14.156303);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().north_east_corr, 0.03173876);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().altitude_stddev_m, 2.828566);

    EXPECT_TRUE(result.speedGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().north_msec, -0.0003105167);
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().east_msec, -0.011570915);
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().up_msec, 0.04521207);

    EXPECT_TRUE(result.speedGeographicFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().north_stddev_msec,
                    0.006963054);
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().east_stddev_msec,
                    0.0066441046);
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().up_stddev_msec,
                    0.009995009);

    EXPECT_TRUE(result.currentGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.currentGeographicFrame.get().north_msec, 1.919778 * 0.0);
    EXPECT_FLOAT_EQ(result.currentGeographicFrame.get().east_msec, 4.5171327 * 0.0);

    EXPECT_TRUE(result.currentGeographicFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.currentGeographicFrameDeviation.get().north_stddev_msec, 0.0);
    EXPECT_FLOAT_EQ(result.currentGeographicFrameDeviation.get().east_stddev_msec, 0.0);

    EXPECT_TRUE(result.systemDate.is_initialized());
    EXPECT_EQ(result.systemDate.get().day, 1);
    EXPECT_EQ(result.systemDate.get().month, 1);
    EXPECT_EQ(result.systemDate.get().year, 2006);

    EXPECT_TRUE(result.sensorStatus.is_initialized());
    EXPECT_EQ(result.sensorStatus.get().status1, 0x0);
    EXPECT_EQ(result.sensorStatus.get().status2, 0x0);

    EXPECT_TRUE(result.insAlgorithmStatus.is_initialized());
    EXPECT_EQ(result.insAlgorithmStatus.get().status1, 0x00000012);
    EXPECT_EQ(result.insAlgorithmStatus.get().status2, 0x00030000);
    EXPECT_EQ(result.insAlgorithmStatus.get().status3, 0x04000000);
    EXPECT_EQ(result.insAlgorithmStatus.get().status4, 0x00000200);

    EXPECT_TRUE(result.insSystemStatus.is_initialized());
    EXPECT_EQ(result.insSystemStatus.get().status1, 0x00000200);
    EXPECT_EQ(result.insSystemStatus.get().status2, 0x00001000);
    EXPECT_EQ(result.insSystemStatus.get().status3, 0x0);

    EXPECT_TRUE(result.insUserStatus.is_initialized());
    EXPECT_EQ(result.insUserStatus.get().status, 0x4c000000);

    EXPECT_FALSE(result.ahrsAlgorithmStatus.is_initialized());

    EXPECT_FALSE(result.ahrsSystemStatus.is_initialized());

    EXPECT_FALSE(result.ahrsUserStatus.is_initialized());

    EXPECT_TRUE(result.heaveSurgeSwaySpeed.is_initialized());
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().realtime_heave_speed, 0.0);
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().surge_speed, 0.0);
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().sway_speed, 0.0);

    EXPECT_TRUE(result.speedVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv1_msec, 0.000061522915);
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv2_msec, 0.011496336);
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv3_msec, 0.04489224);

    EXPECT_TRUE(result.accelerationGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().north_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().east_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().up_msec2, 9.809999);

    EXPECT_TRUE(result.courseSpeedoverGround.is_initialized());
    EXPECT_FLOAT_EQ(result.courseSpeedoverGround.get().course_over_ground, 268.42966);
    EXPECT_FLOAT_EQ(result.courseSpeedoverGround.get().speed_over_ground, 0.011529781);

    EXPECT_TRUE(result.temperatures.is_initialized());
    EXPECT_FLOAT_EQ(result.temperatures.get().mean_temp_fog, 25.993078);
    EXPECT_FLOAT_EQ(result.temperatures.get().mean_temp_acc, 25.993078);
    EXPECT_FLOAT_EQ(result.temperatures.get().board_temperature, 158.65643);

    EXPECT_TRUE(result.attitudeQuaternion.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q0, 0.9997469186782837);
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q1, 0.00038782256888225675);
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q2, -0.0015508179785683751);
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q3, -0.02244032360613346);

    EXPECT_TRUE(result.attitudeQuaternionDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeQuaternionDeviation.get().quat_stddev_xi1,
                    0.00018635205924510956);
    EXPECT_FLOAT_EQ(result.attitudeQuaternionDeviation.get().quat_stddev_xi2,
                    0.00029318343149498105);
    EXPECT_FLOAT_EQ(result.attitudeQuaternionDeviation.get().quat_stddev_xi3,
                    0.9508194327354431);

    EXPECT_TRUE(result.rawAccelerationVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rawAccelerationVesselFrame.get().xv1_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.rawAccelerationVesselFrame.get().xv2_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.rawAccelerationVesselFrame.get().xv3_msec2, 8.000145);

    EXPECT_TRUE(result.accelerationVesselFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.accelerationVesselFrameDeviation.get().xv1_stddev_msec2,
                    0.009960833);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrameDeviation.get().xv2_stddev_msec2,
                    0.009610326);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrameDeviation.get().xv3_stddev_msec2,
                    0.01413504);

    EXPECT_TRUE(result.rotationRateVesselFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrameDeviation.get().xv1_stddev_degsec,
                    0.0005122008);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrameDeviation.get().xv2_stddev_degsec,
                    0.004000983);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrameDeviation.get().xv3_stddev_degsec,
                    0.00051041954);

    // --- Extended navigation data block
    EXPECT_TRUE(result.rotationAccelerationVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rotationAccelerationVesselFrame.get().xv1_degsec2,
                    0.000536854);
    EXPECT_FLOAT_EQ(result.rotationAccelerationVesselFrame.get().xv2_degsec2,
                    0.002185823);
    EXPECT_FLOAT_EQ(result.rotationAccelerationVesselFrame.get().xv3_degsec2,
                    -0.00046422562);

    EXPECT_TRUE(result.rotationAccelerationVesselFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(
        result.rotationAccelerationVesselFrameDeviation.get().xv1_stddev_degsec2,
        0.000706749);
    EXPECT_FLOAT_EQ(
        result.rotationAccelerationVesselFrameDeviation.get().xv2_stddev_degsec2,
        0.0007067489);
    EXPECT_FLOAT_EQ(
        result.rotationAccelerationVesselFrameDeviation.get().xv3_stddev_degsec2,
        0.000706749);

    EXPECT_TRUE(result.rawRotationRateVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rawRotationRateVesselFrame.get().xv1_degsec, 0.0);
    EXPECT_FLOAT_EQ(result.rawRotationRateVesselFrame.get().xv2_degsec, 0.0);
    EXPECT_FLOAT_EQ(result.rawRotationRateVesselFrame.get().xv3_degsec, 0.0);

    EXPECT_FALSE(result.vehicleAttitudeHeading.is_initialized());

    EXPECT_FALSE(result.vehicleAttitudeHeadingDeviation.is_initialized());

    EXPECT_FALSE(result.vehiclePosition.is_initialized());

    EXPECT_FALSE(result.vehiclePositionDeviation.is_initialized());

    // --- External data block ==> No external data in the log
    EXPECT_FALSE(result.utc.is_initialized());

    EXPECT_FALSE(result.gnss1.is_initialized());

    EXPECT_FALSE(result.gnss2.is_initialized());

    EXPECT_FALSE(result.emlog1.is_initialized());

    EXPECT_FALSE(result.emlog2.is_initialized());

    EXPECT_FALSE(result.usbl1.is_initialized());

    EXPECT_FALSE(result.usbl2.is_initialized());

    EXPECT_FALSE(result.usbl3.is_initialized());

    EXPECT_FALSE(result.depth.is_initialized());

    EXPECT_FALSE(result.dvlGroundSpeed1.is_initialized());

    EXPECT_FALSE(result.dvlWaterSpeed1.is_initialized());

    EXPECT_FALSE(result.soundVelocity.is_initialized());

    EXPECT_FALSE(result.dmi.is_initialized());

    EXPECT_FALSE(result.lbl1.is_initialized());

    EXPECT_FALSE(result.lbl2.is_initialized());

    EXPECT_FALSE(result.lbl3.is_initialized());

    EXPECT_FALSE(result.lbl4.is_initialized());

    EXPECT_FALSE(result.eventMarkerA.is_initialized());

    EXPECT_FALSE(result.eventMarkerB.is_initialized());

    EXPECT_FALSE(result.eventMarkerC.is_initialized());

    EXPECT_FALSE(result.dvlGroundSpeed2.is_initialized());

    EXPECT_FALSE(result.dvlWaterSpeed2.is_initialized());

    EXPECT_FALSE(result.turretAngles.is_initialized());

    EXPECT_FALSE(result.vtg1.is_initialized());

    EXPECT_FALSE(result.vtg2.is_initialized());

    EXPECT_FALSE(result.logBook.is_initialized());
}

TEST(StdBinDecoder, WeCanParseV5Protocol)
{
    ixblue_stdbin_decoder::StdBinDecoder parser;

    EXPECT_TRUE(parser.parse(log_STDBIN_V5));
    auto result = parser.getLastNavData();

    // --- Navigation data block
    EXPECT_TRUE(result.attitudeHeading.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().heading_deg, 357.404);
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().roll_deg, -0.04);
    EXPECT_FLOAT_EQ(result.attitudeHeading.get().pitch_deg, 0.178);

    EXPECT_TRUE(result.attitudeHeadingDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().heading_stdDev_deg, 54.47794);
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().roll_stdDev_deg, 0.010677356);
    EXPECT_FLOAT_EQ(result.attitudeHeadingDeviation.get().pitch_stdDev_deg, 0.016798064);

    EXPECT_TRUE(result.rtHeaveSurgeSway.is_initialized());
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_heave_withoutBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_heave_atBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_surge_atBdL, 0.0);
    EXPECT_FLOAT_EQ(result.rtHeaveSurgeSway.get().rt_sway_atBdL, 0.0);

    EXPECT_TRUE(result.smartHeave.is_initialized());
    EXPECT_EQ(result.smartHeave.get().validityTime_100us, 8715456); // Sec
    EXPECT_FLOAT_EQ(result.smartHeave.get().smartHeave_m, 0.0);

    EXPECT_TRUE(result.headingRollPitchRate.is_initialized());
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().heading_rate, -0.002);
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().roll_rate, -0.005);
    EXPECT_FLOAT_EQ(result.headingRollPitchRate.get().pitch_rate, -0.001);

    EXPECT_TRUE(result.rotationRateVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv1_degsec, -0.006139992);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv2_degsec, -0.0009184938);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrame.get().xv3_degsec, 0.0024874974);

    EXPECT_TRUE(result.accelerationVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv1_msec2, 0.000034999997);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv2_msec2, 0.00041499975);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrame.get().xv3_msec2, 0.000715);

    EXPECT_TRUE(result.position.is_initialized());
    EXPECT_DOUBLE_EQ(result.position.get().latitude_deg, 2.133332701513265);
    EXPECT_DOUBLE_EQ(result.position.get().longitude_deg, 47.99999983221664);
    EXPECT_EQ(result.position.get().altitude_ref, 0);
    EXPECT_FLOAT_EQ(result.position.get().altitude_m, 0.16032125);

    EXPECT_TRUE(result.positionDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.positionDeviation.get().north_stddev_m, 14.204068);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().east_stddev_m, 14.156303);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().north_east_corr, 0.03173876);
    EXPECT_FLOAT_EQ(result.positionDeviation.get().altitude_stddev_m, 2.828566);

    EXPECT_TRUE(result.speedGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().north_msec, -0.00031041596);
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().east_msec, -0.011805205);
    EXPECT_FLOAT_EQ(result.speedGeographicFrame.get().up_msec, 0.045028113);

    EXPECT_TRUE(result.speedGeographicFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().north_stddev_msec,
                    0.0069630453);
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().east_stddev_msec,
                    0.006644114);
    EXPECT_FLOAT_EQ(result.speedGeographicFrameDeviation.get().up_stddev_msec,
                    0.009995009);

    EXPECT_TRUE(result.currentGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.currentGeographicFrame.get().north_msec, 1.919778 * 0.0);
    EXPECT_FLOAT_EQ(result.currentGeographicFrame.get().east_msec, 4.5171327 * 0.0);

    EXPECT_TRUE(result.currentGeographicFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.currentGeographicFrameDeviation.get().north_stddev_msec, 0.0);
    EXPECT_FLOAT_EQ(result.currentGeographicFrameDeviation.get().east_stddev_msec, 0.0);

    EXPECT_TRUE(result.systemDate.is_initialized());
    EXPECT_EQ(result.systemDate.get().day, 1);
    EXPECT_EQ(result.systemDate.get().month, 1);
    EXPECT_EQ(result.systemDate.get().year, 2006);

    EXPECT_TRUE(result.sensorStatus.is_initialized());
    EXPECT_EQ(result.sensorStatus.get().status1, 0x0);
    EXPECT_EQ(result.sensorStatus.get().status2, 0x0);

    EXPECT_TRUE(result.insAlgorithmStatus.is_initialized());
    EXPECT_EQ(result.insAlgorithmStatus.get().status1, 0x00000012);
    EXPECT_EQ(result.insAlgorithmStatus.get().status2, 0x00030000);
    EXPECT_EQ(result.insAlgorithmStatus.get().status3, 0x04000000);
    EXPECT_EQ(result.insAlgorithmStatus.get().status4, 0x00000200);

    EXPECT_TRUE(result.insSystemStatus.is_initialized());
    EXPECT_EQ(result.insSystemStatus.get().status1, 0x00000200);
    EXPECT_EQ(result.insSystemStatus.get().status2, 0x00001000);
    EXPECT_EQ(result.insSystemStatus.get().status3, 0x0);

    EXPECT_TRUE(result.insUserStatus.is_initialized());
    EXPECT_EQ(result.insUserStatus.get().status, 0x4c000000);

    EXPECT_FALSE(result.ahrsAlgorithmStatus.is_initialized());

    EXPECT_FALSE(result.ahrsSystemStatus.is_initialized());

    EXPECT_FALSE(result.ahrsUserStatus.is_initialized());

    EXPECT_TRUE(result.heaveSurgeSwaySpeed.is_initialized());
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().realtime_heave_speed, 0.0);
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().surge_speed, 0.0);
    EXPECT_FLOAT_EQ(result.heaveSurgeSwaySpeed.get().sway_speed, 0.0);

    EXPECT_TRUE(result.speedVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv1_msec, 0.00010496828);
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv2_msec, 0.011636287);
    EXPECT_FLOAT_EQ(result.speedVesselFrame.get().xv3_msec, 0.04487192);

    EXPECT_TRUE(result.accelerationGeographicFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().north_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().east_msec2, -0.01);
    EXPECT_FLOAT_EQ(result.accelerationGeographicFrame.get().up_msec2, 9.809999);

    EXPECT_TRUE(result.courseSpeedoverGround.is_initialized());
    EXPECT_FLOAT_EQ(result.courseSpeedoverGround.get().course_over_ground, 268.6071);
    EXPECT_FLOAT_EQ(result.courseSpeedoverGround.get().speed_over_ground, 0.011670535);

    EXPECT_TRUE(result.temperatures.is_initialized());
    EXPECT_FLOAT_EQ(result.temperatures.get().mean_temp_fog, 25.993126);
    EXPECT_FLOAT_EQ(result.temperatures.get().mean_temp_acc, 25.993126);
    EXPECT_FLOAT_EQ(result.temperatures.get().board_temperature, 158.65643);

    EXPECT_TRUE(result.attitudeQuaternion.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q0, 0.999742329120636);
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q1, 0.00038825065712444484);
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q2, -0.0015505076153203845);
    EXPECT_FLOAT_EQ(result.attitudeQuaternion.get().q3, -0.022644637152552605);

    EXPECT_TRUE(result.attitudeQuaternionDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.attitudeQuaternionDeviation.get().quat_stddev_xi1,
                    0.00018635205924510956);
    EXPECT_FLOAT_EQ(result.attitudeQuaternionDeviation.get().quat_stddev_xi2,
                    0.00029318343149498105);
    EXPECT_FLOAT_EQ(result.attitudeQuaternionDeviation.get().quat_stddev_xi3,
                    0.9508194327354431);

    EXPECT_TRUE(result.rawAccelerationVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rawAccelerationVesselFrame.get().xv1_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.rawAccelerationVesselFrame.get().xv2_msec2, 0.0);
    EXPECT_FLOAT_EQ(result.rawAccelerationVesselFrame.get().xv3_msec2, 8.000145);

    EXPECT_TRUE(result.accelerationVesselFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.accelerationVesselFrameDeviation.get().xv1_stddev_msec2,
                    0.009960833);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrameDeviation.get().xv2_stddev_msec2,
                    0.009610326);
    EXPECT_FLOAT_EQ(result.accelerationVesselFrameDeviation.get().xv3_stddev_msec2,
                    0.01413504);

    EXPECT_TRUE(result.rotationRateVesselFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrameDeviation.get().xv1_stddev_degsec,
                    0.0005122008);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrameDeviation.get().xv2_stddev_degsec,
                    0.004000983);
    EXPECT_FLOAT_EQ(result.rotationRateVesselFrameDeviation.get().xv3_stddev_degsec,
                    0.00051041954);

    // --- Extended navigation data block
    EXPECT_TRUE(result.rotationAccelerationVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rotationAccelerationVesselFrame.get().xv1_degsec2,
                    0.00032014688);
    EXPECT_FLOAT_EQ(result.rotationAccelerationVesselFrame.get().xv2_degsec2,
                    0.001597333);
    EXPECT_FLOAT_EQ(result.rotationAccelerationVesselFrame.get().xv3_degsec2,
                    -0.00042017436);

    EXPECT_TRUE(result.rotationAccelerationVesselFrameDeviation.is_initialized());
    EXPECT_FLOAT_EQ(
        result.rotationAccelerationVesselFrameDeviation.get().xv1_stddev_degsec2,
        0.000706749);
    EXPECT_FLOAT_EQ(
        result.rotationAccelerationVesselFrameDeviation.get().xv2_stddev_degsec2,
        0.0007067489);
    EXPECT_FLOAT_EQ(
        result.rotationAccelerationVesselFrameDeviation.get().xv3_stddev_degsec2,
        0.000706749);

    EXPECT_TRUE(result.rawRotationRateVesselFrame.is_initialized());
    EXPECT_FLOAT_EQ(result.rawRotationRateVesselFrame.get().xv1_degsec, 0.0);
    EXPECT_FLOAT_EQ(result.rawRotationRateVesselFrame.get().xv2_degsec, 0.0);
    EXPECT_FLOAT_EQ(result.rawRotationRateVesselFrame.get().xv3_degsec, 0.0);

    EXPECT_FALSE(result.vehicleAttitudeHeading.is_initialized());

    EXPECT_FALSE(result.vehicleAttitudeHeadingDeviation.is_initialized());

    EXPECT_FALSE(result.vehiclePosition.is_initialized());

    EXPECT_FALSE(result.vehiclePositionDeviation.is_initialized());

    // --- External data block ==> No external data in the log
    EXPECT_FALSE(result.utc.is_initialized());

    EXPECT_FALSE(result.gnss1.is_initialized());

    EXPECT_FALSE(result.gnss2.is_initialized());

    EXPECT_FALSE(result.emlog1.is_initialized());

    EXPECT_FALSE(result.emlog2.is_initialized());

    EXPECT_FALSE(result.usbl1.is_initialized());

    EXPECT_FALSE(result.usbl2.is_initialized());

    EXPECT_FALSE(result.usbl3.is_initialized());

    EXPECT_FALSE(result.depth.is_initialized());

    EXPECT_FALSE(result.dvlGroundSpeed1.is_initialized());

    EXPECT_FALSE(result.dvlWaterSpeed1.is_initialized());

    EXPECT_FALSE(result.soundVelocity.is_initialized());

    EXPECT_FALSE(result.dmi.is_initialized());

    EXPECT_FALSE(result.lbl1.is_initialized());

    EXPECT_FALSE(result.lbl2.is_initialized());

    EXPECT_FALSE(result.lbl3.is_initialized());

    EXPECT_FALSE(result.lbl4.is_initialized());

    EXPECT_FALSE(result.eventMarkerA.is_initialized());

    EXPECT_FALSE(result.eventMarkerB.is_initialized());

    EXPECT_FALSE(result.eventMarkerC.is_initialized());

    EXPECT_FALSE(result.dvlGroundSpeed2.is_initialized());

    EXPECT_FALSE(result.dvlWaterSpeed2.is_initialized());

    EXPECT_FALSE(result.turretAngles.is_initialized());

    EXPECT_FALSE(result.vtg1.is_initialized());

    EXPECT_FALSE(result.vtg2.is_initialized());

    EXPECT_FALSE(result.logBook.is_initialized());
}

TEST(StdBinDecoder, WeCanParseAnAnswerFrame)
{
    // clang-format off
    const std::vector<uint8_t> memory{
        'A',  'N',  /* IX blue header   */
        0x03,       /* Protocol Version */
        0x00, 0x0d, /* Telegram size */
        0xde, 0xad, 0xbe, 0xef, /* payload */
        0x01, 0x02, 0x03, 0x04, /* dummy checksum */
    };
    // clang-format on

    ixblue_stdbin_decoder::StdBinDecoder parser;
    ASSERT_TRUE(parser.parse(memory));
    const auto header = parser.getLastHeaderData();
    const auto nav = parser.getLastNavData();
    const auto answer = parser.getLastAnswerData();
    ASSERT_EQ(header.messageType,
              ixblue_stdbin_decoder::Data::NavHeader::MessageType::Answer);
    EXPECT_FALSE(nav.attitudeHeading.is_initialized());
    const std::vector<uint8_t> expectedAnswer{0xde, 0xad, 0xbe, 0xef};
    EXPECT_EQ(answer, expectedAnswer);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
