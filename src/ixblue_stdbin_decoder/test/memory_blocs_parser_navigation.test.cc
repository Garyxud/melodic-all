#include <gtest/gtest.h>

#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/acceleration_geographic_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/acceleration_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/acceleration_vessel_frame_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ahrs_algorithm_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ahrs_system_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ahrs_user_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/attitude_heading.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/attitude_heading_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/attitude_quaternion.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/attitude_quaternion_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/course_speed_over_ground.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/current_geographic_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/current_geographic_frame_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/heading_roll_pitch_rate.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/heave_surge_sway_speed.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ins_algorithm_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ins_system_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/ins_user_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/position.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/position_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/raw_acceleration_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/realtime_heave_surge_sway.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/rotation_rate_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/rotation_rate_vessel_frame_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/sensor_status.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/smart_heave.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/speed_geographic_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/speed_geographic_frame_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/speed_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/system_date.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/navigation_data/temperatures.h>

using namespace ixblue_stdbin_decoder;

TEST(MemoryBocksParser, ParseAttitudeAndHeading)
{
    // Heading : 1.25f( 0x3fa00000 ), Roll : 12.55f (0x4148cccd), Pitch : -1.5f
    // (0xbfc00000)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbf, 0xc0, 0x00, 0x00
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::AttitudeHeading parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.attitudeHeading.is_initialized());
    ASSERT_FLOAT_EQ(1.25, binaryNav.attitudeHeading.get().heading_deg);
    ASSERT_FLOAT_EQ(12.55, binaryNav.attitudeHeading.get().roll_deg);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.attitudeHeading.get().pitch_deg);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseAttitudeAndHeadingDeviation)
{
    // HeadingSD : 1.25f( 0x3fa00000 ), RollSD : 12.55f (0x4148cccd), PitchSD : -1.5f
    // (0xbfc00000)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbf,0xc0, 0x00, 0x00
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::AttitudeHeadingDeviation parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.attitudeHeadingDeviation.is_initialized());
    ASSERT_FLOAT_EQ(1.25, binaryNav.attitudeHeadingDeviation.get().heading_stdDev_deg);
    ASSERT_FLOAT_EQ(12.55, binaryNav.attitudeHeadingDeviation.get().roll_stdDev_deg);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.attitudeHeadingDeviation.get().pitch_stdDev_deg);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseRealTimeHeaveSurgeSway)
{
    // Heave without BdL : 1.25f( 0x3fa00000 ), Heave : 12.55f (0x4148cccd), Surge : -1.5f
    // (0xbfc00000), Sway : -0.005f(0xbba3d70a)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbf, 0xc0, 0x00, 0x00,
        0xbb, 0xa3, 0xd7, 0x0a
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::RealTimeHeaveSurgeSway parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.rtHeaveSurgeSway.is_initialized());
    ASSERT_FLOAT_EQ(1.25, binaryNav.rtHeaveSurgeSway.get().rt_heave_withoutBdL);
    ASSERT_FLOAT_EQ(12.55, binaryNav.rtHeaveSurgeSway.get().rt_heave_atBdL);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.rtHeaveSurgeSway.get().rt_surge_atBdL);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.rtHeaveSurgeSway.get().rt_sway_atBdL);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBlockParser, ParseSmartHeave)
{
    // Time : 125 = 0x0000007D (in big endian)
    // Heave : 1.25f( 0x3fa00000 )
    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x00, 0x7D,
        0x3f, 0xa0, 0x00, 0x00,
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::SmartHeave parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.smartHeave.is_initialized());
    ASSERT_FLOAT_EQ(1.25, binaryNav.smartHeave.get().smartHeave_m);
    ASSERT_EQ(125, binaryNav.smartHeave.get().validityTime_100us);
}

TEST(MemoryBocksParser, ParseHeadingRollPitchRate)
{
    // Heading rate : -1.5f (0xbfc00000), Roll rate : 1.25f( 0x3fa00000 ),
    // Pitch rate : 12.55f (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::HeadingRollPitchRate parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.headingRollPitchRate.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.headingRollPitchRate.get().heading_rate);
    ASSERT_FLOAT_EQ(1.25, binaryNav.headingRollPitchRate.get().roll_rate);
    ASSERT_FLOAT_EQ(12.55, binaryNav.headingRollPitchRate.get().pitch_rate);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseRotationRateVesselFrame)
{
    // XV1 : -1.5f (0xbfc00000), XV2 : 1.25f( 0x3fa00000 ),
    // XV3 : 12.55f (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::RotationRateVesselFrame parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.rotationRateVesselFrame.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.rotationRateVesselFrame.get().xv1_degsec);
    ASSERT_FLOAT_EQ(1.25, binaryNav.rotationRateVesselFrame.get().xv2_degsec);
    ASSERT_FLOAT_EQ(12.55, binaryNav.rotationRateVesselFrame.get().xv3_degsec);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseAccelerationVesselFrame)
{
    // XV1 : -1.5f (0xbfc00000), XV2 : 1.25f( 0x3fa00000 ),
    // XV3 : 12.55 (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::AccelerationVesselFrame parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.accelerationVesselFrame.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.accelerationVesselFrame.get().xv1_msec2);
    ASSERT_FLOAT_EQ(1.25, binaryNav.accelerationVesselFrame.get().xv2_msec2);
    ASSERT_FLOAT_EQ(12.55, binaryNav.accelerationVesselFrame.get().xv3_msec2);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParsePosition)
{
    // latitude : 25.68d (0x4039ae147ae147ae), longitude : -4.75d (0xc013000000000000),
    // altitude ref : 1 (0x01), altitude : 154.21f (0x431a35c3)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x40, 0x39, 0xae, 0x14,
        0x7a, 0xe1, 0x47, 0xae,
        0xc0, 0x13, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x01,
        0x43, 0x1a, 0x35, 0xc3
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::Position parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.position.is_initialized());
    ASSERT_DOUBLE_EQ(25.68, binaryNav.position.get().latitude_deg);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.position.get().longitude_deg);
    ASSERT_EQ(1, binaryNav.position.get().altitude_ref);
    ASSERT_FLOAT_EQ(154.21, binaryNav.position.get().altitude_m);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParsePositionDeviation)
{
    // NorthSD : 1.25f( 0x3fa00000 ), EastSD : 12.55f (0x4148cccd), NEcorr : -1.5f
    // (0xbfc00000), altSD : -0.005f(0xbba3d70a)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbf, 0xc0, 0x00, 0x00,
        0xbb, 0xa3, 0xd7, 0x0a
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::PositionDeviation parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.positionDeviation.is_initialized());
    ASSERT_FLOAT_EQ(1.25, binaryNav.positionDeviation.get().north_stddev_m);
    ASSERT_FLOAT_EQ(12.55, binaryNav.positionDeviation.get().east_stddev_m);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.positionDeviation.get().north_east_corr);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.positionDeviation.get().altitude_stddev_m);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseSpeedGeographicFrame)
{
    // North : -1.5f (0xbfc00000), East : 1.25f( 0x3fa00000 ),
    // Up : 12.55f (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::SpeedGeographicFrame parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.speedGeographicFrame.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.speedGeographicFrame.get().north_msec);
    ASSERT_FLOAT_EQ(1.25, binaryNav.speedGeographicFrame.get().east_msec);
    ASSERT_FLOAT_EQ(12.55, binaryNav.speedGeographicFrame.get().up_msec);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseSpeedGeographicFrameDeviation)
{
    // NorthSD : -1.5f (0xbfc00000), EastSD : 1.25f( 0x3fa00000 ),
    // UpSD : 12.55f (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::SpeedGeographicFrameDeviation parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.speedGeographicFrameDeviation.is_initialized());
    ASSERT_FLOAT_EQ(-1.5,
                    binaryNav.speedGeographicFrameDeviation.get().north_stddev_msec);
    ASSERT_FLOAT_EQ(1.25, binaryNav.speedGeographicFrameDeviation.get().east_stddev_msec);
    ASSERT_FLOAT_EQ(12.55, binaryNav.speedGeographicFrameDeviation.get().up_stddev_msec);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseCurrentGeographicFrameDeviation)
{
    // NorthSD : -1.5f (0xbfc00000), EastSD : 1.25f( 0x3fa00000 )
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::CurrentGeographicFrameDeviation parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.currentGeographicFrameDeviation.is_initialized());
    ASSERT_FLOAT_EQ(-1.5,
                    binaryNav.currentGeographicFrameDeviation.get().north_stddev_msec);
    ASSERT_FLOAT_EQ(1.25,
                    binaryNav.currentGeographicFrameDeviation.get().east_stddev_msec);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseSystemDate)
{
    // Day : 28 (0x1c), Month : 8 (0x08), Year : 2020 (0x07e4)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x1c,
        0x08,
        0x07, 0xe4
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::SystemDate parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.systemDate.is_initialized());
    ASSERT_EQ(28, binaryNav.systemDate.get().day);
    ASSERT_EQ(8, binaryNav.systemDate.get().month);
    ASSERT_EQ(2020, binaryNav.systemDate.get().year);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseSensorStatus)
{
    // Status 1 : 532 (0x00000214), Status 2 : 17 (0x00000011)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x02, 0x14,
        0x00, 0x00, 0x00, 0x11
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::SensorStatus parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.sensorStatus.is_initialized());
    ASSERT_EQ(532, binaryNav.sensorStatus.get().status1);
    ASSERT_EQ(17, binaryNav.sensorStatus.get().status2);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseINSAlgorithmStatus)
{
    // Status 1 : 532 (0x00000214), Status 2 : 42 (0x0000002a)
    // Status 3 : 47826 (0x0000bad2), Status 4 : 17 (0x00000011)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x02, 0x14,
        0x00, 0x00, 0x00, 0x2a,
        0x00, 0x00, 0xba, 0xd2,
        0x00, 0x00, 0x00, 0x11
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::INSAlgorithmStatus parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.insAlgorithmStatus.is_initialized());
    ASSERT_EQ(532, binaryNav.insAlgorithmStatus.get().status1);
    ASSERT_EQ(42, binaryNav.insAlgorithmStatus.get().status2);
    ASSERT_EQ(47826, binaryNav.insAlgorithmStatus.get().status3);
    ASSERT_EQ(17, binaryNav.insAlgorithmStatus.get().status4);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseINSSystemStatus)
{
    // Status 1 : 532 (0x00000214), Status 2 : 42 (0x0000002a)
    // Status 3 : 47826 (0x0000bad2)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x02, 0x14,
        0x00, 0x00, 0x00, 0x2a,
        0x00, 0x00, 0xba, 0xd2
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::INSSystemStatus parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.insSystemStatus.is_initialized());
    ASSERT_EQ(532, binaryNav.insSystemStatus.get().status1);
    ASSERT_EQ(42, binaryNav.insSystemStatus.get().status2);
    ASSERT_EQ(47826, binaryNav.insSystemStatus.get().status3);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseINSUserStatus)
{
    // Status : 532 (0x00000214)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x02, 0x14,
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::INSUserStatus parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.insUserStatus.is_initialized());
    ASSERT_EQ(532, binaryNav.insUserStatus.get().status);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseAHRSAlgorithmStatus)
{
    // Status : 532 (0x00000214)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x02, 0x14
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::AHRSAlgorithmStatus parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.ahrsAlgorithmStatus.is_initialized());
    ASSERT_EQ(532, binaryNav.ahrsAlgorithmStatus.get().status);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseAHRSSystemStatus)
{
    // Status 1 : 532 (0x00000214), Status 2 : 42 (0x0000002a)
    // Status 3 : 47826 (0x0000bad2)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x02, 0x14,
        0x00, 0x00, 0x00, 0x2a,
        0x00, 0x00, 0xba, 0xd2
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::AHRSSystemStatus parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.ahrsSystemStatus.is_initialized());
    ASSERT_EQ(532, binaryNav.ahrsSystemStatus.get().status1);
    ASSERT_EQ(42, binaryNav.ahrsSystemStatus.get().status2);
    ASSERT_EQ(47826, binaryNav.ahrsSystemStatus.get().status3);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseAHRSUserStatus)
{
    // Status : 532 (0x00000214)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x02, 0x14
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::AHRSUserStatus parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.ahrsUserStatus.is_initialized());
    ASSERT_EQ(532, binaryNav.ahrsUserStatus.get().status);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseHeaveSurgeSwaySpeed)
{
    // rtHeaveSpeed : -1.5f (0xbfc00000), SurgeSpeed : 1.25f( 0x3fa00000 ),
    // SwaySpeed : 12.55f (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::HeaveSurgeSwaySpeed parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.heaveSurgeSwaySpeed.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.heaveSurgeSwaySpeed.get().realtime_heave_speed);
    ASSERT_FLOAT_EQ(1.25, binaryNav.heaveSurgeSwaySpeed.get().surge_speed);
    ASSERT_FLOAT_EQ(12.55, binaryNav.heaveSurgeSwaySpeed.get().sway_speed);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseSpeedVesselFrame)
{
    // xv1 : -1.5f (0xbfc00000), xv2 : 1.25f( 0x3fa00000 ), xv3 : 12.55f (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::SpeedVesselFrame parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.speedVesselFrame.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.speedVesselFrame.get().xv1_msec);
    ASSERT_FLOAT_EQ(1.25, binaryNav.speedVesselFrame.get().xv2_msec);
    ASSERT_FLOAT_EQ(12.55, binaryNav.speedVesselFrame.get().xv3_msec);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseAccelerationGeographicFrame)
{
    // north : -1.5f (0xbfc00000), east : 1.25f( 0x3fa00000 ), up : 12.55f (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::AccelerationGeographicFrame parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.accelerationGeographicFrame.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.accelerationGeographicFrame.get().north_msec2);
    ASSERT_FLOAT_EQ(1.25, binaryNav.accelerationGeographicFrame.get().east_msec2);
    ASSERT_FLOAT_EQ(12.55, binaryNav.accelerationGeographicFrame.get().up_msec2);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseourseSpeedoverGrounde)
{
    // course : -1.5f (0xbfc00000), speed : 1.25f( 0x3fa00000 )
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::CourseSpeedoverGround parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.courseSpeedoverGround.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.courseSpeedoverGround.get().course_over_ground);
    ASSERT_FLOAT_EQ(1.25, binaryNav.courseSpeedoverGround.get().speed_over_ground);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseTemperatures)
{
    // FOG : -1.5f (0xbfc00000), Acc : 1.25f( 0x3fa00000 ), Board : 12.55f (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::Temperatures parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.temperatures.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.temperatures.get().mean_temp_fog);
    ASSERT_FLOAT_EQ(1.25, binaryNav.temperatures.get().mean_temp_acc);
    ASSERT_FLOAT_EQ(12.55, binaryNav.temperatures.get().board_temperature);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseAttitudeQuaternion)
{
    // Q0 : 1.25f ( 0x3fa00000 ), Q1 : 12.55f (0x4148cccd), Q2 : -1.5f (0xbfc00000), Q3 :
    // -0.005f (0xbba3d70a)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbf, 0xc0, 0x00, 0x00,
        0xbb, 0xa3, 0xd7, 0x0a
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::AttitudeQuaternion parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.attitudeQuaternion.is_initialized());
    ASSERT_FLOAT_EQ(1.25, binaryNav.attitudeQuaternion.get().q0);
    ASSERT_FLOAT_EQ(12.55, binaryNav.attitudeQuaternion.get().q1);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.attitudeQuaternion.get().q2);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.attitudeQuaternion.get().q3);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseAttitudeQuaternionDeviation)
{
    // xi1 : -1.5f (0xbfc00000), xi2 : 1.25f( 0x3fa00000 ), xi3 : 12.55f (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::AttitudeQuaternionDeviation parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.attitudeQuaternionDeviation.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.attitudeQuaternionDeviation.get().quat_stddev_xi1);
    ASSERT_FLOAT_EQ(1.25, binaryNav.attitudeQuaternionDeviation.get().quat_stddev_xi2);
    ASSERT_FLOAT_EQ(12.55, binaryNav.attitudeQuaternionDeviation.get().quat_stddev_xi3);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseRawAccelerationVesselFrame)
{
    // xv1 : -1.5f (0xbfc00000), xv2 : 1.25f( 0x3fa00000 ), xv3 : 12.55f (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::RawAccelerationVesselFrame parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.rawAccelerationVesselFrame.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.rawAccelerationVesselFrame.get().xv1_msec2);
    ASSERT_FLOAT_EQ(1.25, binaryNav.rawAccelerationVesselFrame.get().xv2_msec2);
    ASSERT_FLOAT_EQ(12.55, binaryNav.rawAccelerationVesselFrame.get().xv3_msec2);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseAccelerationVesselFrameDeviation)
{
    // xv1SD : -1.5f (0xbfc00000), xv2SD : 1.25f( 0x3fa00000 ), xv3SD : 12.55f
    // (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::AccelerationVesselFrameDeviation parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.accelerationVesselFrameDeviation.is_initialized());
    ASSERT_FLOAT_EQ(-1.5,
                    binaryNav.accelerationVesselFrameDeviation.get().xv1_stddev_msec2);
    ASSERT_FLOAT_EQ(1.25,
                    binaryNav.accelerationVesselFrameDeviation.get().xv2_stddev_msec2);
    ASSERT_FLOAT_EQ(12.55,
                    binaryNav.accelerationVesselFrameDeviation.get().xv3_stddev_msec2);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseRotationRateVesselFrameDeviation)
{
    // xv1SD : -1.5f (0xbfc00000), xv2SD : 1.25f( 0x3fa00000 ), xv3SD : 12.55f
    // (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::RotationRateVesselFrameDeviation parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.rotationRateVesselFrameDeviation.is_initialized());
    ASSERT_FLOAT_EQ(-1.5,
                    binaryNav.rotationRateVesselFrameDeviation.get().xv1_stddev_degsec);
    ASSERT_FLOAT_EQ(1.25,
                    binaryNav.rotationRateVesselFrameDeviation.get().xv2_stddev_degsec);
    ASSERT_FLOAT_EQ(12.55,
                    binaryNav.rotationRateVesselFrameDeviation.get().xv3_stddev_degsec);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
