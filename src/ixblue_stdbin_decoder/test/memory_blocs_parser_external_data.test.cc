#include <array>
#include <gtest/gtest.h>

#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/depth.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/dmi.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/dvl_ground_speed.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/dvl_water_speed.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/emlog.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/eventmarker.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/gnss.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/lbl.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/logbook.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/sound_velocity.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/turret_angles.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/usbl.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/utc.h>
#include <ixblue_stdbin_decoder/memory_blocs_parsers/external_data/vtg.h>

using namespace ixblue_stdbin_decoder;

TEST(MemoryBocksParser, ParseUtc)
{
    // Validity Time : 254 (0x000000fe), source : 24 (0x18)
    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x00, 0xfe,
        0x18
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::Utc parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.utc.is_initialized());
    ASSERT_EQ(254, binaryNav.utc.get().validityTime_100us);
    ASSERT_EQ(24, binaryNav.utc.get().source);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseGnss)
{
    // 1 --> Validity Time : 254 (0x000000fe), GNSS Id : 24 (0x18), GNSS Quality : 28
    // (0x1c), latitude : 25.68d (0x4039ae147ae147ae), longitude : -4.75d
    // (0xc013000000000000), altitude : 154.21f (0x431a35c3), latitudeSD : 1.25f(
    // 0x3fa00000 ), longitudeSD : 12.55f (0x4148cccd), altSD : -1.5f (0xbfc00000),
    // latlonCOR : -0.005f (0xbba3d70a), geoidalSep : 8.562 (0x4108fdf4)

    // 2 --> Validity Time : 254 (0x000000fe), GNSS Id : 24 (0x18), GNSS Quality : 28
    // (0x1c), latitude : -4.75d (0xc013000000000000), longitude : 25.68d
    // (0x4039ae147ae147ae), altitude : 154.21f (0x431a35c3), latitudeSD : -1.5f
    // (0xbfc00000), longitudeSD : 12.55f (0x4148cccd), altSD : 1.25f( 0x3fa00000 ),
    // latlonCOR : -0.005f (0xbba3d70a), geoidalSep : 8.562 (0x4108fdf4)

    // 3 --> Validity Time : 58 (0x0000003a), GNSS Id : 4 (0x04), GNSS Quality : 28
    // (0x1c), latitude : 25.68d (0x4039ae147ae147ae), longitude : -4.75d
    // (0xc013000000000000), altitude : 154.21f (0x431a35c3), latitudeSD : 1.25f(
    // 0x3fa00000 ), longitudeSD : 12.55f (0x4148cccd), altSD : -1.5f (0xbfc00000),
    // latlonCOR : -0.005f (0xbba3d70a), geoidalSep : 8.562 (0x4108fdf4)

    // clang-format off
    const std::vector<uint8_t> memory1{
        0x00, 0x00, 0x00, 0xfe,
        0x18,
        0x1c,
        0x40, 0x39, 0xae, 0x14,
        0x7a, 0xe1, 0x47, 0xae,
        0xc0, 0x13, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbf, 0xc0, 0x00, 0x00,
        0xbb, 0xa3, 0xd7, 0x0a,
        0x41, 0x08, 0xfd, 0xf4
    };

    const std::vector<uint8_t> memory2{
        0x00, 0x00, 0x00, 0xfe,
        0x18,
        0x1c,
        0xc0, 0x13, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x40, 0x39, 0xae, 0x14,
        0x7a, 0xe1, 0x47, 0xae,
        0x43, 0x1a, 0x35, 0xc3,
        0xbf, 0xc0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0x3f, 0xa0, 0x00, 0x00,
        0xbb, 0xa3, 0xd7, 0x0a,
        0x41, 0x08, 0xfd, 0xf4
    };

    const std::vector<uint8_t> memory3{
        0x00, 0x00, 0x00, 0x3a,
        0x04,
        0x1c,
        0x40, 0x39, 0xae, 0x14,
        0x7a, 0xe1, 0x47, 0xae,
        0xc0, 0x13, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbf, 0xc0, 0x00, 0x00,
        0xbb, 0xa3, 0xd7, 0x0a,
        0x41, 0x08, 0xfd, 0xf4
    };
    // clang-format on

    std::vector<uint8_t> gnss_memory(memory1);
    gnss_memory.insert(gnss_memory.end(), memory2.begin(), memory2.end());
    gnss_memory.insert(gnss_memory.end(), memory3.begin(), memory3.end());

    auto buffer = boost::asio::buffer(const_cast<const uint8_t*>(gnss_memory.data()),
                                      gnss_memory.size());

    Parser::Gnss1 parser1;
    Parser::Gnss2 parser2;
    Parser::GnssManual parser3;

    Data::BinaryNav binaryNav;

    parser1.parse(buffer, binaryNav);
    parser2.parse(buffer, binaryNav);
    parser3.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.gnss1.is_initialized());
    ASSERT_EQ(254, binaryNav.gnss1.get().validityTime_100us);
    ASSERT_EQ(24, binaryNav.gnss1.get().gnss_id);
    ASSERT_EQ(28, binaryNav.gnss1.get().gnss_quality);
    ASSERT_DOUBLE_EQ(25.68, binaryNav.gnss1.get().latitude_deg);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.gnss1.get().longitude_deg);
    ASSERT_FLOAT_EQ(154.21, binaryNav.gnss1.get().altitude_m);
    ASSERT_FLOAT_EQ(1.25, binaryNav.gnss1.get().latitude_stddev_m);
    ASSERT_FLOAT_EQ(12.55, binaryNav.gnss1.get().longitude_stddev_m);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.gnss1.get().altitude_stddev_m);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.gnss1.get().lat_lon_stddev_m2);
    ASSERT_FLOAT_EQ(8.562, binaryNav.gnss1.get().geoidal_separation_m);

    ASSERT_TRUE(binaryNav.gnss2.is_initialized());
    ASSERT_EQ(254, binaryNav.gnss2.get().validityTime_100us);
    ASSERT_EQ(24, binaryNav.gnss2.get().gnss_id);
    ASSERT_EQ(28, binaryNav.gnss2.get().gnss_quality);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.gnss2.get().latitude_deg);
    ASSERT_DOUBLE_EQ(25.68, binaryNav.gnss2.get().longitude_deg);
    ASSERT_FLOAT_EQ(154.21, binaryNav.gnss2.get().altitude_m);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.gnss2.get().latitude_stddev_m);
    ASSERT_FLOAT_EQ(12.55, binaryNav.gnss2.get().longitude_stddev_m);
    ASSERT_FLOAT_EQ(1.25, binaryNav.gnss2.get().altitude_stddev_m);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.gnss2.get().lat_lon_stddev_m2);
    ASSERT_FLOAT_EQ(8.562, binaryNav.gnss2.get().geoidal_separation_m);

    ASSERT_TRUE(binaryNav.gnssManual.is_initialized());
    ASSERT_EQ(58, binaryNav.gnssManual.get().validityTime_100us);
    ASSERT_EQ(4, binaryNav.gnssManual.get().gnss_id);
    ASSERT_EQ(28, binaryNav.gnssManual.get().gnss_quality);
    ASSERT_DOUBLE_EQ(25.68, binaryNav.gnssManual.get().latitude_deg);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.gnssManual.get().longitude_deg);
    ASSERT_FLOAT_EQ(154.21, binaryNav.gnssManual.get().altitude_m);
    ASSERT_FLOAT_EQ(1.25, binaryNav.gnssManual.get().latitude_stddev_m);
    ASSERT_FLOAT_EQ(12.55, binaryNav.gnssManual.get().longitude_stddev_m);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.gnssManual.get().altitude_stddev_m);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.gnssManual.get().lat_lon_stddev_m2);
    ASSERT_FLOAT_EQ(8.562, binaryNav.gnssManual.get().geoidal_separation_m);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseEmlog)
{
    // 1 -> Validity Time : 254 (0x000000fe), EMLOG Id : 24 (0x18),
    // Waterspeed : 154.21f (0x431a35c3), WaterspeedSD : 1.25f(0x3fa00000)

    // 2 -> Validity Time : 58 (0x0000003a), EMLOG Id : 4 (0x04),
    // Waterspeed : 1.25f(0x3fa00000), WaterspeedSD : 12.55f (0x4148cccd)

    // clang-format off
    const std::vector<uint8_t> memory1{
        0x00, 0x00, 0x00, 0xfe,
        0x18,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00
    };

    const std::vector<uint8_t> memory2{
        0x00, 0x00, 0x00, 0x3a,
        0x04,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on

    std::vector<uint8_t> emlog_memory(memory1);
    emlog_memory.insert(emlog_memory.end(), memory2.begin(), memory2.end());

    auto buffer = boost::asio::buffer(const_cast<const uint8_t*>(emlog_memory.data()),
                                      emlog_memory.size());

    Parser::Emlog1 parser1;
    Parser::Emlog2 parser2;

    Data::BinaryNav binaryNav;

    parser1.parse(buffer, binaryNav);
    parser2.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.emlog1.is_initialized());
    ASSERT_EQ(254, binaryNav.emlog1.get().validityTime_100us);
    ASSERT_EQ(24, binaryNav.emlog1.get().emlog_id);
    ASSERT_FLOAT_EQ(154.21, binaryNav.emlog1.get().xv1_waterSpeed_ms);
    ASSERT_FLOAT_EQ(1.25, binaryNav.emlog1.get().xv1_speed_stddev_ms);

    ASSERT_TRUE(binaryNav.emlog2.is_initialized());
    ASSERT_EQ(58, binaryNav.emlog2.get().validityTime_100us);
    ASSERT_EQ(4, binaryNav.emlog2.get().emlog_id);
    ASSERT_FLOAT_EQ(1.25, binaryNav.emlog2.get().xv1_waterSpeed_ms);
    ASSERT_FLOAT_EQ(12.55, binaryNav.emlog2.get().xv1_speed_stddev_ms);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseUsbl)
{
    // 1 --> Validity Time : 254 (0x000000fe), USBL Id : 24 (0x18), Beacon id : abcdefgh
    // (0x6162636465666768), latitude : 25.68d (0x4039ae147ae147ae), longitude : -4.75d
    // (0xc013000000000000), altitude : 154.21f (0x431a35c3), latitudeSD : 1.25f(
    // 0x3fa00000 ), longitudeSD : 12.55f (0x4148cccd), latlonCOR : -0.005f (0xbba3d70a),
    // altSD : -1.5f (0xbfc00000)

    // 2 --> Validity Time : 58 (0x0000003a), USBL Id : 4 (0x04), Beacon id : abcdefgh
    // (0x6162636465666768), latitude : 25.68d (0x4039ae147ae147ae), longitude : -4.75d
    // (0xc013000000000000), altitude : 154.21f (0x431a35c3), latitudeSD : 1.25f(
    // 0x3fa00000 ), longitudeSD : 12.55f (0x4148cccd), latlonCOR : -0.005f (0xbba3d70a),
    // altSD : -1.5f (0xbfc00000)

    // 3 --> Validity Time : 254 (0x000000fe), USBL Id : 24 (0x18), Beacon id : abcdefgh
    // (0x6162636465666768), latitude : 25.68d (0x4039ae147ae147ae), longitude : -4.75d
    // (0xc013000000000000), altitude : 12.55f (0x4148cccd), latitudeSD : 1.25f(
    // 0x3fa00000 ), longitudeSD : 154.21f (0x431a35c3), latlonCOR : -0.005f (0xbba3d70a),
    // altSD : -1.5f (0xbfc00000)

    // clang-format off
    const std::vector<uint8_t> memory1{
        0x00, 0x00, 0x00, 0xfe,
        0x18,
        0x61, 0x62, 0x63, 0x64,
        0x65, 0x66, 0x67, 0x68,
        0x40, 0x39, 0xae, 0x14,
        0x7a, 0xe1, 0x47, 0xae,
        0xc0, 0x13, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbb, 0xa3, 0xd7, 0x0a,
        0xbf, 0xc0, 0x00, 0x00
    };

    const std::vector<uint8_t> memory2{
        0x00, 0x00, 0x00, 0x3a,
        0x04,
        0x61, 0x62, 0x63, 0x64,
        0x65, 0x66, 0x67, 0x68,
        0x40, 0x39, 0xae, 0x14,
        0x7a, 0xe1, 0x47, 0xae,
        0xc0, 0x13, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbb, 0xa3, 0xd7, 0x0a,
        0xbf, 0xc0, 0x00, 0x00
    };

    const std::vector<uint8_t> memory3{
        0x00, 0x00, 0x00, 0xfe,
        0x18,
        0x61, 0x62, 0x63, 0x64,
        0x65, 0x66, 0x67, 0x68,
        0x40, 0x39, 0xae, 0x14,
        0x7a, 0xe1, 0x47, 0xae,
        0xc0, 0x13, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0x3f, 0xa0, 0x00, 0x00,
        0x43, 0x1a, 0x35, 0xc3,
        0xbb, 0xa3, 0xd7, 0x0a,
        0xbf, 0xc0, 0x00, 0x00
    };
    // clang-format on

    std::vector<uint8_t> usbl_memory(memory1);
    usbl_memory.insert(usbl_memory.end(), memory2.begin(), memory2.end());
    usbl_memory.insert(usbl_memory.end(), memory3.begin(), memory3.end());

    auto buffer = boost::asio::buffer(const_cast<const uint8_t*>(usbl_memory.data()),
                                      usbl_memory.size());

    Parser::Usbl1 parser1;
    Parser::Usbl2 parser2;
    Parser::Usbl3 parser3;

    Data::BinaryNav binaryNav;

    parser1.parse(buffer, binaryNav);
    parser2.parse(buffer, binaryNav);
    parser3.parse(buffer, binaryNav);

    std::array<uint8_t, 8> ref_val = {{'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'}};

    ASSERT_TRUE(binaryNav.usbl1.is_initialized());
    ASSERT_EQ(254, binaryNav.usbl1.get().validityTime_100us);
    ASSERT_EQ(24, binaryNav.usbl1.get().usbl_id);
    ASSERT_EQ(ref_val, binaryNav.usbl1.get().beacon_id);
    ASSERT_DOUBLE_EQ(25.68, binaryNav.usbl1.get().latitude_deg);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.usbl1.get().longitude_deg);
    ASSERT_FLOAT_EQ(154.21, binaryNav.usbl1.get().altitude_m);
    ASSERT_FLOAT_EQ(1.25, binaryNav.usbl1.get().north_stddev_m);
    ASSERT_FLOAT_EQ(12.55, binaryNav.usbl1.get().east_stddev_m);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.usbl1.get().lat_lon_cov_m2);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.usbl1.get().altitude_stddev_m);

    ASSERT_TRUE(binaryNav.usbl2.is_initialized());
    ASSERT_EQ(58, binaryNav.usbl2.get().validityTime_100us);
    ASSERT_EQ(4, binaryNav.usbl2.get().usbl_id);
    ASSERT_EQ(ref_val, binaryNav.usbl2.get().beacon_id);
    ASSERT_DOUBLE_EQ(25.68, binaryNav.usbl2.get().latitude_deg);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.usbl2.get().longitude_deg);
    ASSERT_FLOAT_EQ(154.21, binaryNav.usbl2.get().altitude_m);
    ASSERT_FLOAT_EQ(1.25, binaryNav.usbl2.get().north_stddev_m);
    ASSERT_FLOAT_EQ(12.55, binaryNav.usbl2.get().east_stddev_m);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.usbl2.get().lat_lon_cov_m2);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.usbl2.get().altitude_stddev_m);

    ASSERT_TRUE(binaryNav.usbl3.is_initialized());
    ASSERT_EQ(254, binaryNav.usbl3.get().validityTime_100us);
    ASSERT_EQ(24, binaryNav.usbl3.get().usbl_id);
    ASSERT_EQ(ref_val, binaryNav.usbl3.get().beacon_id);
    ASSERT_DOUBLE_EQ(25.68, binaryNav.usbl3.get().latitude_deg);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.usbl3.get().longitude_deg);
    ASSERT_FLOAT_EQ(12.55, binaryNav.usbl3.get().altitude_m);
    ASSERT_FLOAT_EQ(1.25, binaryNav.usbl3.get().north_stddev_m);
    ASSERT_FLOAT_EQ(154.21, binaryNav.usbl3.get().east_stddev_m);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.usbl3.get().lat_lon_cov_m2);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.usbl3.get().altitude_stddev_m);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseDepth)
{
    // Validity Time : 254 (0x000000fe)
    // Depth : 154.21f (0x431a35c3), DepthSD : 1.25f( 0x3fa00000 )

    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x00, 0xfe,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::Depth parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.depth.is_initialized());
    ASSERT_EQ(254, binaryNav.depth.get().validityTime_100us);
    ASSERT_FLOAT_EQ(154.21, binaryNav.depth.get().depth_m);
    ASSERT_FLOAT_EQ(1.25, binaryNav.depth.get().depth_stddev_m);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseDvlGroundSpeed)
{
    // 1 --> Validity Time : 254 (0x000000fe), DVL Id : 24 (0x18),
    // xv1 : 154.21f (0x431a35c3), xv2 : 1.25f (0x3fa00000), xv3 : 12.55f (0x4148cccd),
    // Speed of Sound : -0.005f (0xbba3d70a), altitude : -1.5f (0xbfc00000),
    // xv1SD : 8.56f (0x4108f5c3), xv2SD : -2.4f (0xc019999a), xv3SD : 42.12f (0x42287ae1)

    // 2 --> Validity Time : 58 (0x0000003a), DVL Id : 4 (0x04),
    // xv1 : 154.21f (0x431a35c3), xv2 : 1.25f (0x3fa00000), xv3 : 12.55f (0x4148cccd),
    // Speed of Sound : -0.005f (0xbba3d70a), altitude : -1.5f (0xbfc00000),
    // xv1SD : 8.56f (0x4108f5c3), xv2SD : -2.4f (0xc019999a), xv3SD : 42.12f (0x42287ae1)

    // clang-format off
    const std::vector<uint8_t> memory1{
        0x00, 0x00, 0x00, 0xfe,
        0x18,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbb, 0xa3, 0xd7, 0x0a,
        0xbf, 0xc0, 0x00, 0x00,
        0x41, 0x08, 0xf5, 0xc3,
        0xc0, 0x19, 0x99, 0x9a,
        0x42, 0x28, 0x7a, 0xe1
    };

    const std::vector<uint8_t> memory2{
        0x00, 0x00, 0x00, 0x3a,
        0x04,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbb, 0xa3, 0xd7, 0x0a,
        0xbf, 0xc0, 0x00, 0x00,
        0x41, 0x08, 0xf5, 0xc3,
        0xc0, 0x19, 0x99, 0x9a,
        0x42, 0x28, 0x7a, 0xe1
    };
    // clang-format on
    std::vector<uint8_t> dvlgroundspeed_memory(memory1);
    dvlgroundspeed_memory.insert(dvlgroundspeed_memory.end(), memory2.begin(),
                                 memory2.end());

    auto buffer =
        boost::asio::buffer(const_cast<const uint8_t*>(dvlgroundspeed_memory.data()),
                            dvlgroundspeed_memory.size());

    Parser::DvlGroundSpeed1 parser1;
    Parser::DvlGroundSpeed2 parser2;

    Data::BinaryNav binaryNav;

    parser1.parse(buffer, binaryNav);
    parser2.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.dvlGroundSpeed1.is_initialized());
    ASSERT_EQ(254, binaryNav.dvlGroundSpeed1.get().validityTime_100us);
    ASSERT_EQ(24, binaryNav.dvlGroundSpeed1.get().dvl_id);
    ASSERT_FLOAT_EQ(154.21, binaryNav.dvlGroundSpeed1.get().xv1_groundspeed_ms);
    ASSERT_FLOAT_EQ(1.25, binaryNav.dvlGroundSpeed1.get().xv2_groundspeed_ms);
    ASSERT_FLOAT_EQ(12.55, binaryNav.dvlGroundSpeed1.get().xv3_groundspeed_ms);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.dvlGroundSpeed1.get().dvl_speedofsound_ms);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.dvlGroundSpeed1.get().dvl_altitude_m);
    ASSERT_FLOAT_EQ(8.56, binaryNav.dvlGroundSpeed1.get().xv1_stddev_ms);
    ASSERT_FLOAT_EQ(-2.4, binaryNav.dvlGroundSpeed1.get().xv2_stddev_ms);
    ASSERT_FLOAT_EQ(42.12, binaryNav.dvlGroundSpeed1.get().xv3_stddev_ms);

    ASSERT_TRUE(binaryNav.dvlGroundSpeed2.is_initialized());
    ASSERT_EQ(58, binaryNav.dvlGroundSpeed2.get().validityTime_100us);
    ASSERT_EQ(4, binaryNav.dvlGroundSpeed2.get().dvl_id);
    ASSERT_FLOAT_EQ(154.21, binaryNav.dvlGroundSpeed2.get().xv1_groundspeed_ms);
    ASSERT_FLOAT_EQ(1.25, binaryNav.dvlGroundSpeed2.get().xv2_groundspeed_ms);
    ASSERT_FLOAT_EQ(12.55, binaryNav.dvlGroundSpeed2.get().xv3_groundspeed_ms);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.dvlGroundSpeed2.get().dvl_speedofsound_ms);
    ASSERT_FLOAT_EQ(-1.5, binaryNav.dvlGroundSpeed2.get().dvl_altitude_m);
    ASSERT_FLOAT_EQ(8.56, binaryNav.dvlGroundSpeed2.get().xv1_stddev_ms);
    ASSERT_FLOAT_EQ(-2.4, binaryNav.dvlGroundSpeed2.get().xv2_stddev_ms);
    ASSERT_FLOAT_EQ(42.12, binaryNav.dvlGroundSpeed2.get().xv3_stddev_ms);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseDvlWaterSpeed)
{
    // 1 --> Validity Time : 254 (0x000000fe), DVL Id : 24 (0x18),
    // xv1 : 154.21f (0x431a35c3), xv2 : 1.25f (0x3fa00000), xv3 : 12.55f (0x4148cccd),
    // Speed of Sound : -0.005f (0xbba3d70a)
    // xv1SD : 8.56f (0x4108f5c3), xv2SD : -2.4f (0xc019999a), xv3SD : 42.12f (0x42287ae1)

    // 2 --> Validity Time : 58 (0x0000003a), DVL Id : 4 (0x04),
    // xv1 : 154.21f (0x431a35c3), xv2 : 1.25f (0x3fa00000), xv3 : 12.55f (0x4148cccd),
    // Speed of Sound : -0.005f (0xbba3d70a)
    // xv1SD : 8.56f (0x4108f5c3), xv2SD : -2.4f (0xc019999a), xv3SD : 42.12f (0x42287ae1)

    // clang-format off
    const std::vector<uint8_t> memory1{
        0x00, 0x00, 0x00, 0xfe,
        0x18,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbb, 0xa3, 0xd7, 0x0a,
        0x41, 0x08, 0xf5, 0xc3,
        0xc0, 0x19, 0x99, 0x9a,
        0x42, 0x28, 0x7a, 0xe1
    };

    const std::vector<uint8_t> memory2{
        0x00, 0x00, 0x00, 0x3a,
        0x04,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbb, 0xa3, 0xd7, 0x0a,
        0x41, 0x08, 0xf5, 0xc3,
        0xc0, 0x19, 0x99, 0x9a,
        0x42, 0x28, 0x7a, 0xe1
    };
    // clang-format on

    std::vector<uint8_t> dvlwaterspeed_memory(memory1);
    dvlwaterspeed_memory.insert(dvlwaterspeed_memory.end(), memory2.begin(),
                                memory2.end());

    auto buffer =
        boost::asio::buffer(const_cast<const uint8_t*>(dvlwaterspeed_memory.data()),
                            dvlwaterspeed_memory.size());

    Parser::DvlWaterSpeed1 parser1;
    Parser::DvlWaterSpeed2 parser2;

    Data::BinaryNav binaryNav;

    parser1.parse(buffer, binaryNav);
    parser2.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.dvlWaterSpeed1.is_initialized());
    ASSERT_EQ(254, binaryNav.dvlWaterSpeed1.get().validityTime_100us);
    ASSERT_EQ(24, binaryNav.dvlWaterSpeed1.get().dvl_id);
    ASSERT_FLOAT_EQ(154.21, binaryNav.dvlWaterSpeed1.get().xv1_waterspeed_ms);
    ASSERT_FLOAT_EQ(1.25, binaryNav.dvlWaterSpeed1.get().xv2_waterspeed_ms);
    ASSERT_FLOAT_EQ(12.55, binaryNav.dvlWaterSpeed1.get().xv3_waterspeed_ms);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.dvlWaterSpeed1.get().dvl_speedofsound_ms);
    ASSERT_FLOAT_EQ(8.56, binaryNav.dvlWaterSpeed1.get().xv1_stddev_ms);
    ASSERT_FLOAT_EQ(-2.4, binaryNav.dvlWaterSpeed1.get().xv2_stddev_ms);
    ASSERT_FLOAT_EQ(42.12, binaryNav.dvlWaterSpeed1.get().xv3_stddev_ms);

    ASSERT_TRUE(binaryNav.dvlWaterSpeed2.is_initialized());
    ASSERT_EQ(58, binaryNav.dvlWaterSpeed2.get().validityTime_100us);
    ASSERT_EQ(4, binaryNav.dvlWaterSpeed2.get().dvl_id);
    ASSERT_FLOAT_EQ(154.21, binaryNav.dvlWaterSpeed2.get().xv1_waterspeed_ms);
    ASSERT_FLOAT_EQ(1.25, binaryNav.dvlWaterSpeed2.get().xv2_waterspeed_ms);
    ASSERT_FLOAT_EQ(12.55, binaryNav.dvlWaterSpeed2.get().xv3_waterspeed_ms);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.dvlWaterSpeed2.get().dvl_speedofsound_ms);
    ASSERT_FLOAT_EQ(8.56, binaryNav.dvlWaterSpeed2.get().xv1_stddev_ms);
    ASSERT_FLOAT_EQ(-2.4, binaryNav.dvlWaterSpeed2.get().xv2_stddev_ms);
    ASSERT_FLOAT_EQ(42.12, binaryNav.dvlWaterSpeed2.get().xv3_stddev_ms);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseSoundVelocity)
{
    // Validity Time : 254 (0x000000fe), Speed of sound : 154.21f (0x431a35c3)

    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x00, 0xfe,
        0x43, 0x1a, 0x35, 0xc3
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::SoundVelocity parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.soundVelocity.is_initialized());
    ASSERT_EQ(254, binaryNav.soundVelocity.get().validityTime_100us);
    ASSERT_FLOAT_EQ(154.21, binaryNav.soundVelocity.get().ext_speedofsound_ms);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseDmi)
{
    // Validity Time : 254 (0x000000fe), Pulse count : 154 (0x0000009a)

    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x00, 0xfe,
        0x00, 0x00, 0x00, 0x9a
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::Dmi parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.dmi.is_initialized());
    ASSERT_EQ(254, binaryNav.dmi.get().validityTime_100us);
    ASSERT_FLOAT_EQ(154, binaryNav.dmi.get().pulse_count);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseLbl)
{
    // 1 --> Validity Time : 254 (0x000000fe),  RFU : 0 (0x00), Beacon id : abcdefgh
    // (0x6162636465666768), latitude : 25.68d (0x4039ae147ae147ae), longitude : -4.75d
    // (0xc013000000000000), altitude : 154.21f (0x431a35c3), range : 1.25f(
    // 0x3fa00000 ), rangeSD : 12.55f (0x4148cccd)

    // 2 --> Validity Time : 58 (0x0000003a),  RFU : 0 (0x00), Beacon id : abcdefgh
    // (0x6162636465666768), latitude : 25.68d (0x4039ae147ae147ae), longitude : -4.75d
    // (0xc013000000000000), altitude : 154.21f (0x431a35c3), range : 1.25f(
    // 0x3fa00000 ), rangeSD : 12.55f (0x4148cccd)

    // 3 --> Validity Time : 254 (0x000000fe),  RFU : 0 (0x00), Beacon id : abcdefgh
    // (0x6162636465666768), latitude : -4.75d(0xc013000000000000), longitude : 25.68d
    // (0x4039ae147ae147ae), altitude : 154.21f (0x431a35c3), range : 1.25f( 0x3fa00000 ),
    // rangeSD : 12.55f (0x4148cccd)

    // 4 --> Validity Time : 58 (0x0000003a),  RFU : 0 (0x00), Beacon id : abcdefgh
    // (0x6162636465666768), latitude : -4.75d(0xc013000000000000), longitude : 25.68d
    // (0x4039ae147ae147ae), altitude : 154.21f (0x431a35c3), range : 1.25f(
    // 0x3fa00000 ), rangeSD : 12.55f (0x4148cccd)

    // clang-format off
    const std::vector<uint8_t> memory1{
        0x00, 0x00, 0x00, 0xfe,
        0x00,
        0x61, 0x62, 0x63, 0x64,
        0x65, 0x66, 0x67, 0x68,
        0x40, 0x39, 0xae, 0x14,
        0x7a, 0xe1, 0x47, 0xae,
        0xc0, 0x13, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };

    const std::vector<uint8_t> memory2{
        0x00, 0x00, 0x00, 0x3a,
        0x00,
        0x61, 0x62, 0x63, 0x64,
        0x65, 0x66, 0x67, 0x68,
        0x40, 0x39, 0xae, 0x14,
        0x7a, 0xe1, 0x47, 0xae,
        0xc0, 0x13, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };

    const std::vector<uint8_t> memory3{
        0x00, 0x00, 0x00, 0xfe,
        0x00,
        0x61, 0x62, 0x63, 0x64,
        0x65, 0x66, 0x67, 0x68,
        0xc0, 0x13, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x40, 0x39, 0xae, 0x14,
        0x7a, 0xe1, 0x47, 0xae,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };

    const std::vector<uint8_t> memory4{
        0x00, 0x00, 0x00, 0x3a,
        0x00,
        0x61, 0x62, 0x63, 0x64,
        0x65, 0x66, 0x67, 0x68,
        0xc0, 0x13, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x40, 0x39, 0xae, 0x14,
        0x7a, 0xe1, 0x47, 0xae,
        0x43, 0x1a, 0x35, 0xc3,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd
    };
    // clang-format on

    std::vector<uint8_t> lbl_memory(memory1);
    lbl_memory.insert(lbl_memory.end(), memory2.begin(), memory2.end());
    lbl_memory.insert(lbl_memory.end(), memory3.begin(), memory3.end());
    lbl_memory.insert(lbl_memory.end(), memory4.begin(), memory4.end());

    auto buffer = boost::asio::buffer(const_cast<const uint8_t*>(lbl_memory.data()),
                                      lbl_memory.size());

    Parser::Lbl1 parser1;
    Parser::Lbl2 parser2;
    Parser::Lbl3 parser3;
    Parser::Lbl4 parser4;

    Data::BinaryNav binaryNav;

    parser1.parse(buffer, binaryNav);
    parser2.parse(buffer, binaryNav);
    parser3.parse(buffer, binaryNav);
    parser4.parse(buffer, binaryNav);

    std::array<uint8_t, 8> ref_val = {{'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'}};

    ASSERT_TRUE(binaryNav.lbl1.is_initialized());
    ASSERT_EQ(254, binaryNav.lbl1.get().validityTime_100us);
    ASSERT_EQ(0, binaryNav.lbl1.get().rfu);
    ASSERT_EQ(ref_val, binaryNav.lbl1.get().beacon_id);
    ASSERT_DOUBLE_EQ(25.68, binaryNav.lbl1.get().beacon_latitude_deg);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.lbl1.get().beacon_longitude_deg);
    ASSERT_FLOAT_EQ(154.21, binaryNav.lbl1.get().beacon_altitude_m);
    ASSERT_FLOAT_EQ(1.25, binaryNav.lbl1.get().range_m);
    ASSERT_FLOAT_EQ(12.55, binaryNav.lbl1.get().range_stddev_m);

    ASSERT_TRUE(binaryNav.lbl2.is_initialized());
    ASSERT_EQ(58, binaryNav.lbl2.get().validityTime_100us);
    ASSERT_EQ(0, binaryNav.lbl2.get().rfu);
    ASSERT_EQ(ref_val, binaryNav.lbl2.get().beacon_id);
    ASSERT_DOUBLE_EQ(25.68, binaryNav.lbl2.get().beacon_latitude_deg);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.lbl2.get().beacon_longitude_deg);
    ASSERT_FLOAT_EQ(154.21, binaryNav.lbl2.get().beacon_altitude_m);
    ASSERT_FLOAT_EQ(1.25, binaryNav.lbl2.get().range_m);
    ASSERT_FLOAT_EQ(12.55, binaryNav.lbl2.get().range_stddev_m);

    ASSERT_TRUE(binaryNav.lbl3.is_initialized());
    ASSERT_EQ(254, binaryNav.lbl3.get().validityTime_100us);
    ASSERT_EQ(0, binaryNav.lbl3.get().rfu);
    ASSERT_EQ(ref_val, binaryNav.lbl3.get().beacon_id);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.lbl3.get().beacon_latitude_deg);
    ASSERT_DOUBLE_EQ(25.68, binaryNav.lbl3.get().beacon_longitude_deg);
    ASSERT_FLOAT_EQ(154.21, binaryNav.lbl3.get().beacon_altitude_m);
    ASSERT_FLOAT_EQ(1.25, binaryNav.lbl3.get().range_m);
    ASSERT_FLOAT_EQ(12.55, binaryNav.lbl3.get().range_stddev_m);

    ASSERT_TRUE(binaryNav.lbl4.is_initialized());
    ASSERT_EQ(58, binaryNav.lbl4.get().validityTime_100us);
    ASSERT_EQ(0, binaryNav.lbl4.get().rfu);
    ASSERT_EQ(ref_val, binaryNav.lbl4.get().beacon_id);
    ASSERT_DOUBLE_EQ(-4.75, binaryNav.lbl4.get().beacon_latitude_deg);
    ASSERT_DOUBLE_EQ(25.68, binaryNav.lbl4.get().beacon_longitude_deg);
    ASSERT_FLOAT_EQ(154.21, binaryNav.lbl4.get().beacon_altitude_m);
    ASSERT_FLOAT_EQ(1.25, binaryNav.lbl4.get().range_m);
    ASSERT_FLOAT_EQ(12.55, binaryNav.lbl4.get().range_stddev_m);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseEventMarker)
{
    // 1 --> Validity Time : 254 (0x000000fe), event id : 24 (0x18)
    // Pulse count : 154 (0x0000009a)

    // 2 --> Validity Time : 58 (0x0000003a), event id : 24 (0x18)
    // Pulse count : 254 (0x000000fe)

    // 3 --> Validity Time : 254 (0x000000fe), event id : 4 (0x04)
    // Pulse count : 58 (0x0000003a)

    // clang-format off
    const std::vector<uint8_t> memory1{
        0x00, 0x00, 0x00, 0xfe,
        0x18,
        0x00, 0x00, 0x00, 0x9a
    };

    const std::vector<uint8_t> memory2{
        0x00, 0x00, 0x00, 0x3a,
        0x18,
        0x00, 0x00, 0x00, 0xfe
    };

    const std::vector<uint8_t> memory3{
        0x00, 0x00, 0x00, 0xfe,
        0x04,
        0x00, 0x00, 0x00, 0x3a
    };
    // clang-format on
    std::vector<uint8_t> eventmarker_memory(memory1);
    eventmarker_memory.insert(eventmarker_memory.end(), memory2.begin(), memory2.end());
    eventmarker_memory.insert(eventmarker_memory.end(), memory3.begin(), memory3.end());

    auto buffer = boost::asio::buffer(
        const_cast<const uint8_t*>(eventmarker_memory.data()), eventmarker_memory.size());

    Parser::EventMarkerA parser1;
    Parser::EventMarkerB parser2;
    Parser::EventMarkerC parser3;

    Data::BinaryNav binaryNav;

    parser1.parse(buffer, binaryNav);
    parser2.parse(buffer, binaryNav);
    parser3.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.eventMarkerA.is_initialized());
    ASSERT_EQ(254, binaryNav.eventMarkerA.get().validityTime_100us);
    ASSERT_EQ(24, binaryNav.eventMarkerA.get().event_id);
    ASSERT_EQ(154, binaryNav.eventMarkerA.get().event_count);

    ASSERT_TRUE(binaryNav.eventMarkerB.is_initialized());
    ASSERT_EQ(58, binaryNav.eventMarkerB.get().validityTime_100us);
    ASSERT_EQ(24, binaryNav.eventMarkerB.get().event_id);
    ASSERT_EQ(254, binaryNav.eventMarkerB.get().event_count);

    ASSERT_TRUE(binaryNav.eventMarkerC.is_initialized());
    ASSERT_EQ(254, binaryNav.eventMarkerC.get().validityTime_100us);
    ASSERT_EQ(4, binaryNav.eventMarkerC.get().event_id);
    ASSERT_EQ(58, binaryNav.eventMarkerC.get().event_count);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseTurretAngles)
{
    // Validity Time : 254 (0x000000fe),
    // Angle : 1.25f (0x3fa00000), Roll : 12.55f (0x4148cccd),
    // Elevation putch : -0.005f (0xbba3d70a)

    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x00, 0xfe,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbb, 0xa3, 0xd7, 0x0a
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::TurretAngles parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.turretAngles.is_initialized());
    ASSERT_EQ(254, binaryNav.turretAngles.get().validityTime_100us);
    ASSERT_FLOAT_EQ(1.25, binaryNav.turretAngles.get().headingbearingdrift_angle_deg);
    ASSERT_FLOAT_EQ(12.55, binaryNav.turretAngles.get().roll_deg);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.turretAngles.get().elevationpitch_deg);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseVtg)
{
    // 1 --> Validity Time : 254 (0x000000fe), VTG Id : 24 (0x18),
    // Angle : 1.25f (0x3fa00000), Roll : 12.55f (0x4148cccd),
    // Elevation putch : -0.005f (0xbba3d70a)

    // 2 --> Validity Time : 58 (0x0000003a), VTG Id : 4 (0x04),
    // Angle : 1.25f (0x3fa00000), Roll : 12.55f (0x4148cccd),
    // Elevation putch : -0.005f (0xbba3d70a)

    // clang-format off
    const std::vector<uint8_t> memory1{
        0x00, 0x00, 0x00, 0xfe,
        0x18,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbb, 0xa3, 0xd7, 0x0a
    };

    const std::vector<uint8_t> memory2{
        0x00, 0x00, 0x00, 0x3a,
        0x04,
        0x3f, 0xa0, 0x00, 0x00,
        0x41, 0x48, 0xcc, 0xcd,
        0xbb, 0xa3, 0xd7, 0x0a
    };
    // clang-format on
    std::vector<uint8_t> vtg_memory(memory1);
    vtg_memory.insert(vtg_memory.end(), memory2.begin(), memory2.end());

    auto buffer = boost::asio::buffer(const_cast<const uint8_t*>(vtg_memory.data()),
                                      vtg_memory.size());

    Parser::Vtg1 parser1;
    Parser::Vtg2 parser2;

    Data::BinaryNav binaryNav;

    parser1.parse(buffer, binaryNav);
    parser2.parse(buffer, binaryNav);

    ASSERT_TRUE(binaryNav.vtg1.is_initialized());
    ASSERT_EQ(254, binaryNav.vtg1.get().validityTime_100us);
    ASSERT_EQ(24, binaryNav.vtg1.get().vtg_id);
    ASSERT_FLOAT_EQ(1.25, binaryNav.vtg1.get().true_course_deg);
    ASSERT_FLOAT_EQ(12.55, binaryNav.vtg1.get().magnetic_course_deg);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.vtg1.get().speed_over_ground_ms);

    ASSERT_TRUE(binaryNav.vtg2.is_initialized());
    ASSERT_EQ(58, binaryNav.vtg2.get().validityTime_100us);
    ASSERT_EQ(4, binaryNav.vtg2.get().vtg_id);
    ASSERT_FLOAT_EQ(1.25, binaryNav.vtg2.get().true_course_deg);
    ASSERT_FLOAT_EQ(12.55, binaryNav.vtg2.get().magnetic_course_deg);
    ASSERT_FLOAT_EQ(-0.005, binaryNav.vtg2.get().speed_over_ground_ms);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

TEST(MemoryBocksParser, ParseLogBook)
{
    // Validity Time : 254 (0x000000fe), event id : 785 (0x00000311)
    // Pulse count : abcd (0x61626364)

    // clang-format off
    const std::vector<uint8_t> memory{
        0x00, 0x00, 0x00, 0xfe,
        0x00, 0x00, 0x03, 0x11,
        0x61, 0x62, 0x63, 0x64,
        0x65, 0x66, 0x67, 0x68,
        0x61, 0x62, 0x63, 0x64,
        0x65, 0x66, 0x67, 0x68,
        0x61, 0x62, 0x63, 0x64,
        0x65, 0x66, 0x67, 0x68,
        0x61, 0x62, 0x63, 0x64,
        0x65, 0x66, 0x67, 0x68
    };
    // clang-format on
    auto buffer = boost::asio::buffer(memory.data(), memory.size());

    Parser::LogBook parser;
    Data::BinaryNav binaryNav;
    parser.parse(buffer, binaryNav);

    std::array<uint8_t, 32> ref_val = {
        {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h',
         'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'}};

    ASSERT_TRUE(binaryNav.logBook.is_initialized());
    ASSERT_EQ(254, binaryNav.logBook.get().validityTime_100us);
    ASSERT_EQ(785, binaryNav.logBook.get().log_id);
    ASSERT_EQ(ref_val, binaryNav.logBook.get().custom_text);

    // We also check that buffer was completly consumed by parser.
    ASSERT_EQ(0, boost::asio::buffer_size(buffer));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
