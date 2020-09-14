#include <gtest/gtest.h>

#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/raw_rotation_rate_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/rotation_acceleration_vessel_frame.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/rotation_acceleration_vessel_frame_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/vehicle_attitude_heading.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/vehicle_attitude_heading_deviation.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/vehicle_position.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/extended_navigation_data/vehicle_position_deviation.h>

using namespace ixblue_stdbin_decoder;

TEST(MemoryBocksParser, ParseRotationAccelerationVesselFrame)
{
    // xv1 : -1.5f (0xbfc00000), xv2 : 1.25f( 0x3fa00000 ), xv3 : 12.55f
    // (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::RotationAccelerationVesselFrame parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.rotationAccelerationVesselFrame.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.rotationAccelerationVesselFrame.get().xv1_degsec2);
    ASSERT_FLOAT_EQ(1.25, binaryNav.rotationAccelerationVesselFrame.get().xv2_degsec2);
    ASSERT_FLOAT_EQ(12.55, binaryNav.rotationAccelerationVesselFrame.get().xv3_degsec2);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseRotationAccelerationVesselFrameDeviation)
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

    Parser::RotationAccelerationVesselFrameDeviation parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.rotationAccelerationVesselFrameDeviation.is_initialized());
    ASSERT_FLOAT_EQ(
        -1.5,
        binaryNav.rotationAccelerationVesselFrameDeviation.get().xv1_stddev_degsec2);
    ASSERT_FLOAT_EQ(
        1.25,
        binaryNav.rotationAccelerationVesselFrameDeviation.get().xv2_stddev_degsec2);
    ASSERT_FLOAT_EQ(
        12.55,
        binaryNav.rotationAccelerationVesselFrameDeviation.get().xv3_stddev_degsec2);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseRawRotationRateVesselFrame)
{
    // xv1 : -1.5f (0xbfc00000), xv2 : 1.25f( 0x3fa00000 ), xv3 : 12.55f
    // (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::RawRotationRateVesselFrame parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.rawRotationRateVesselFrame.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.rawRotationRateVesselFrame.get().xv1_degsec);
    ASSERT_FLOAT_EQ(1.25, binaryNav.rawRotationRateVesselFrame.get().xv2_degsec);
    ASSERT_FLOAT_EQ(12.55, binaryNav.rawRotationRateVesselFrame.get().xv3_degsec);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseVehicleAttitudeHeading)
{
    // xv1 : -1.5f (0xbfc00000), xv2 : 1.25f( 0x3fa00000 ), xv3 : 12.55f
    // (0x4148cccd)
    // clang-format off
    const std::vector<uint8_t> memory{
        0xbf, 0xc0, 0x00, 0x00,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::VehicleAttitudeHeading parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.vehicleAttitudeHeading.is_initialized());
    ASSERT_FLOAT_EQ(-1.5, binaryNav.vehicleAttitudeHeading.get().heading_deg);
    ASSERT_FLOAT_EQ(1.25, binaryNav.vehicleAttitudeHeading.get().roll_deg);
    ASSERT_FLOAT_EQ(12.55, binaryNav.vehicleAttitudeHeading.get().pitch_deg);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseVehicleAttitudeHeadingDeviation)
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

    Parser::VehicleAttitudeHeadingDeviation parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.vehicleAttitudeHeadingDeviation.is_initialized());
    ASSERT_FLOAT_EQ(-1.5,
                    binaryNav.vehicleAttitudeHeadingDeviation.get().heading_stddev_deg);
    ASSERT_FLOAT_EQ(1.25,
                    binaryNav.vehicleAttitudeHeadingDeviation.get().roll_stddev_deg);
    ASSERT_FLOAT_EQ(12.55,
                    binaryNav.vehicleAttitudeHeadingDeviation.get().pitch_stddev_deg);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseVehiclePosition)
{
    // latitude : 25.68d (0x4039ae147ae147ae), longitude : -4.75d (0xc013000000000000),
    // altitude ref : 1 (0x01), altitude : 154.21f (0x43 1a 35 c3)
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

    Parser::VehiclePosition parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.vehiclePosition.is_initialized());
    ASSERT_DOUBLE_EQ(25.68, binaryNav.vehiclePosition.get().latitude_deg);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.vehiclePosition.get().longitude_deg);
    ASSERT_EQ(1, binaryNav.vehiclePosition.get().altitude_ref);
    ASSERT_FLOAT_EQ(154.21, binaryNav.vehiclePosition.get().altitude_m);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseVehiclePositionDeviation)
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

    Parser::VehiclePositionDeviation parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.vehiclePositionDeviation.is_initialized());
    ASSERT_FLOAT_EQ(1.25, binaryNav.vehiclePositionDeviation.get().north_stddev_m);
    ASSERT_FLOAT_EQ(12.55, binaryNav.vehiclePositionDeviation.get().east_stddev_m);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.vehiclePositionDeviation.get().north_east_corr);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.vehiclePositionDeviation.get().altitude_stddev_m);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}