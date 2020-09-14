// Copyright (c) 2019-2020 Pilz GmbH & Co. KG
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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include "psen_scan/mock_psen_scan_udp_interface.h"
#include "psen_scan/scanner.h"
#include "psen_scan/laserscan.h"
#include "psen_scan/scanner_data.h"
#include "psen_scan/coherent_monitoring_frames_exception.h"
#include "psen_scan/psen_scan_fatal_exception.h"
#include "psen_scan/parse_monitoring_frame_exception.h"
#include "psen_scan/diagnostic_information_exception.h"
#include "psen_scan/udp_read_timeout_exception.h"

using namespace psen_scan;
using ::testing::_;
using ::testing::Assign;
using ::testing::AtLeast;
using ::testing::AtMost;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::SetArgPointee;
using ::testing::SetArgReferee;
using ::testing::Throw;

#define READ_FRAME(x) WillOnce(DoAll(fillArg0(expected_monitoring_frames_.at(x)), Return(sizeof(MonitoringFrame))))
#define RETURN_IP(ip_str) Return(udp::endpoint(boost::asio::ip::address_v4::from_string(ip_str), 2000))

namespace psen_scan_test
{
class ScannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    udp_interface_ptr = std::unique_ptr<MockPSENscanUDPInterface>(new MockPSENscanUDPInterface());

    MonitoringFrame expected_monitoring_frame = {};

    // Frame 0
    expected_monitoring_frame.opcode_ = MONITORING_FRAME_OPCODE;
    expected_monitoring_frame.scanner_id_ = (uint8_t)0;
    expected_monitoring_frame.resolution_ = (uint8_t)1;
    expected_monitoring_frame.scan_counter_ = (uint32_t)4294967295U;
    expected_monitoring_frame.number_of_samples_ = (uint16_t)500;
    expected_monitoring_frame.from_theta_ = (uint16_t)0;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 1
    expected_monitoring_frame.from_theta_ = 500;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 2
    expected_monitoring_frame.from_theta_ = 1000;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 3
    expected_monitoring_frame.from_theta_ = 1500;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 4
    expected_monitoring_frame.from_theta_ = 2000;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 5
    expected_monitoring_frame.from_theta_ = 2500;
    expected_monitoring_frame.number_of_samples_ = 250;
    expected_monitoring_frame.scan_counter_ = 0U;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // ---------------------------------------------------- //
    // From this point only frames with errors are declared //
    // ---------------------------------------------------- //

    // Frame 6
    expected_monitoring_frame.opcode_ = 0U;  // this is the bug it is != MONITORING_FRAME_OPCODE
    expected_monitoring_frame.from_theta_ = 0;
    expected_monitoring_frame.number_of_samples_ = 500;
    expected_monitoring_frame.scan_counter_ = 0;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 7
    expected_monitoring_frame.opcode_ = 123U;  // this is the bug it is != MONITORING_FRAME_OPCODE
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 8
    expected_monitoring_frame.opcode_ = MONITORING_FRAME_OPCODE;
    expected_monitoring_frame.scanner_id_ = 0;
    expected_monitoring_frame.number_of_samples_ = 550;  // No bug but its a limit / border
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 9
    expected_monitoring_frame.number_of_samples_ = 551;  // Above the limit by one
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 10
    expected_monitoring_frame.number_of_samples_ = 500;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.ossd1_short_circuit_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 11
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.ossd1_short_circuit_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.short_circuit_at_least_two_ossd_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 12
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.short_circuit_at_least_two_ossd_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.integrity_check_problem_on_any_ossd_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 13
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.integrity_check_problem_on_any_ossd_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.internal_error_1_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 14
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.internal_error_1_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.window_cleaning_alarm_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 15
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.window_cleaning_alarm_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.power_supply_problem_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 16
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.power_supply_problem_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.network_problem_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 17
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.network_problem_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.dust_circuit_failure_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 18
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.dust_circuit_failure_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.measure_problem_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 19
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.measure_problem_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.incoherence_data_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 20
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.incoherence_data_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.zone_invalid_input_transition_or_integrity_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 21
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.zone_invalid_input_transition_or_integrity_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.zone_invalid_input_configuration_connection_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 22
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.zone_invalid_input_configuration_connection_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.window_cleaning_warning_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 23
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.window_cleaning_warning_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.internal_communication_problem_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 24
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.internal_communication_problem_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.generic_error_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 25
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.generic_error_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.display_communication_problem_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 26
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.display_communication_problem_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.temperature_measurement_problem_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 27
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.temperature_measurement_problem_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.configuration_error_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 28
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.configuration_error_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.out_of_range_error_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 29
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.out_of_range_error_ = 0;
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.temperature_range_error_ = 1;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 30
    expected_monitoring_frame.diagnostic_area_.diagnostic_information_.temperature_range_error_ = 0;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 31
    expected_monitoring_frame.from_theta_ = 500;
    expected_monitoring_frame.resolution_ = 2;  // resolution changed
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 32
    expected_monitoring_frame.from_theta_ = 0;
    expected_monitoring_frame.resolution_ = 1;
    expected_monitoring_frame.scanner_id_ = 1;  // scanner_id not equal 0
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 33
    expected_monitoring_frame.scan_counter_ = 0;
    expected_monitoring_frame.scanner_id_ = 0;
    expected_monitoring_frames_.push_back(expected_monitoring_frame);

    // Frame 34
    expected_monitoring_frame.from_theta_ = 500;
    expected_monitoring_frame.scan_counter_ = 1;  // scan counter changed
    expected_monitoring_frames_.push_back(expected_monitoring_frame);
  }

  std::unique_ptr<MockPSENscanUDPInterface> udp_interface_ptr;
  std::vector<MonitoringFrame> expected_monitoring_frames_;
};

ACTION_P(fillArg0, monitoring_frame)
{
  ::testing::StaticAssertTypeEq<MonitoringFrame, monitoring_frame_type>();
  boost::asio::buffer_copy(arg0, boost::asio::buffer(&monitoring_frame, sizeof(MonitoringFrame)));
}

TEST_F(ScannerTest, getCompleteScan_ideal)
{
  EXPECT_CALL(*udp_interface_ptr, read(_, _))
      .Times(AtMost(6))
      .READ_FRAME(0)
      .READ_FRAME(1)
      .READ_FRAME(2)
      .READ_FRAME(3)
      .READ_FRAME(4)
      .READ_FRAME(5);

  Scanner scanner("1.2.3.4",
                  0x01020305,
                  1234,
                  "p4sswort",
                  PSENscanInternalAngle(0),
                  PSENscanInternalAngle(2750),
                  std::move(udp_interface_ptr));

  LaserScan scan = scanner.getCompleteScan();

  EXPECT_EQ(scan.min_scan_angle_, PSENscanInternalAngle(0));
  EXPECT_EQ(scan.max_scan_angle_, PSENscanInternalAngle(2750));
  EXPECT_EQ(scan.resolution_, PSENscanInternalAngle(1));
  std::vector<uint16_t> expected_measures(2750, 0);
  EXPECT_EQ(scan.measures_, expected_measures);
}

TEST_F(ScannerTest, getCompleteScan_missing_frame_middle)
{
  EXPECT_CALL(*udp_interface_ptr, read(_, _)).Times(AtMost(2)).READ_FRAME(0).READ_FRAME(2);

  Scanner scanner("1.2.3.4",
                  0x01020305,
                  1234,
                  "p4sswort",
                  PSENscanInternalAngle(0),
                  PSENscanInternalAngle(2750),
                  std::move(udp_interface_ptr));

  ASSERT_THROW(scanner.getCompleteScan(), CoherentMonitoringFramesException);
}

TEST_F(ScannerTest, getCompleteScan_missing_frame_first)
{
  EXPECT_CALL(*udp_interface_ptr, read(_, _)).Times(AtMost(1)).READ_FRAME(1);

  Scanner scanner("1.2.3.4",
                  0x01020305,
                  1234,
                  "p4sswort",
                  PSENscanInternalAngle(0),
                  PSENscanInternalAngle(2750),
                  std::move(udp_interface_ptr));

  ASSERT_THROW(scanner.getCompleteScan(), CoherentMonitoringFramesException);
}

TEST_F(ScannerTest, getCompleteScan_missing_frame_last)
{
  EXPECT_CALL(*udp_interface_ptr, read(_, _))
      .Times(AtMost(6))
      .READ_FRAME(0)
      .READ_FRAME(1)
      .READ_FRAME(2)
      .READ_FRAME(3)
      .READ_FRAME(4)
      .READ_FRAME(0);

  Scanner scanner("1.2.3.4",
                  0x01020305,
                  1234,
                  "p4sswort",
                  PSENscanInternalAngle(0),
                  PSENscanInternalAngle(2750),
                  std::move(udp_interface_ptr));

  ASSERT_THROW(scanner.getCompleteScan(), CoherentMonitoringFramesException);
}

TEST_F(ScannerTest, getCompleteScan_correct_return_value)
{
  EXPECT_CALL(*udp_interface_ptr, read(_, _))
      .Times(AtMost(6))
      .READ_FRAME(0)
      .READ_FRAME(1)
      .READ_FRAME(2)
      .READ_FRAME(3)
      .READ_FRAME(4)
      .READ_FRAME(5);

  Scanner scanner("1.2.3.4",
                  0x01020305,
                  1234,
                  "p4sswort",
                  PSENscanInternalAngle(90),
                  PSENscanInternalAngle(300),
                  std::move(udp_interface_ptr));

  LaserScan scan = scanner.getCompleteScan();

  EXPECT_EQ(scan.min_scan_angle_, PSENscanInternalAngle(90));
  EXPECT_EQ(scan.max_scan_angle_, PSENscanInternalAngle(300));
  EXPECT_EQ(scan.resolution_, PSENscanInternalAngle(1));
  std::vector<uint16_t> expected_measures(210, 0);
  EXPECT_EQ(scan.measures_, expected_measures);
}

TEST_F(ScannerTest, ConstructorWrongArguments)
{
  EXPECT_THROW(Scanner scanner("1.2.3.450",  // This throws exception
                               0x01020305,
                               1234,
                               "p4sswort",
                               PSENscanInternalAngle(0),
                               PSENscanInternalAngle(2750),
                               std::move(udp_interface_ptr)),
               PSENScanFatalException);
  EXPECT_THROW(Scanner scanner("1.2.3.4",
                               0x01020305,
                               65536,  // This throws exception
                               "p4sswort",
                               PSENscanInternalAngle(0),
                               PSENscanInternalAngle(2750),
                               std::move(udp_interface_ptr)),
               PSENScanFatalException);
  EXPECT_THROW(Scanner scanner("1.2.3.4",
                               0x01020305,
                               1023,  // Print Warning
                               "p4sswort",
                               PSENscanInternalAngle(0),
                               PSENscanInternalAngle(2750),
                               nullptr  // This throws exception
                               ),
               PSENScanFatalException);
  EXPECT_THROW(Scanner scanner("1.2.3.4",
                               0x01020305,
                               1023,  // Print Warning
                               "p4sswort",
                               PSENscanInternalAngle(2),
                               PSENscanInternalAngle(1),  // This throws exception
                               std::move(udp_interface_ptr)),
               PSENScanFatalException);
  EXPECT_THROW(Scanner scanner("1.2.3.4",
                               0x01020305,
                               1023,  // Print Warning
                               "p4sswort",
                               PSENscanInternalAngle(0),
                               MAX_SCAN_ANGLE + PSENscanInternalAngle(1),  // This throws exception
                               std::move(udp_interface_ptr)),
               PSENScanFatalException);
}

TEST_F(ScannerTest, StartStop)
{
  EXPECT_CALL(*udp_interface_ptr, write(_)).Times(2);

  Scanner scanner("1.2.3.4",
                  0x01020305,
                  1234,
                  "p4sswort",
                  PSENscanInternalAngle(0),
                  PSENscanInternalAngle(2750),
                  std::move(udp_interface_ptr));

  scanner.start();
  scanner.stop();
}

TEST_F(ScannerTest, testParseMonitoringFrameException)
{
  EXPECT_CALL(*udp_interface_ptr, read(_, _))
      .Times(5)
      .READ_FRAME(6)
      .READ_FRAME(7)
      .READ_FRAME(32)
      .READ_FRAME(8)
      .READ_FRAME(9);

  Scanner scanner("1.2.3.4",
                  0x01020305,
                  1234,
                  "p4sswort",
                  PSENscanInternalAngle(0),
                  PSENscanInternalAngle(2750),
                  std::move(udp_interface_ptr));

  EXPECT_THROW(scanner.getCompleteScan(), ParseMonitoringFrameException);
  EXPECT_THROW(scanner.getCompleteScan(), ParseMonitoringFrameException);
  EXPECT_THROW(scanner.getCompleteScan(), ParseMonitoringFrameException);
  EXPECT_THROW(scanner.getCompleteScan(), ParseMonitoringFrameException);
}

TEST_F(ScannerTest, testDiagnosticInformationException)
{
  EXPECT_CALL(*udp_interface_ptr, read(_, _))
      .Times(20)
      .READ_FRAME(10)
      .READ_FRAME(11)
      .READ_FRAME(12)
      .READ_FRAME(13)
      .READ_FRAME(14)
      .READ_FRAME(15)
      .READ_FRAME(16)
      .READ_FRAME(17)
      .READ_FRAME(18)
      .READ_FRAME(19)
      .READ_FRAME(20)
      .READ_FRAME(21)
      .READ_FRAME(22)
      .READ_FRAME(23)
      .READ_FRAME(24)
      .READ_FRAME(25)
      .READ_FRAME(26)
      .READ_FRAME(27)
      .READ_FRAME(28)
      .READ_FRAME(29);

  Scanner scanner("1.2.3.4",
                  0x01020305,
                  1234,
                  "p4sswort",
                  PSENscanInternalAngle(0),
                  PSENscanInternalAngle(2750),
                  std::move(udp_interface_ptr));

  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
  EXPECT_THROW(scanner.getCompleteScan(), DiagnosticInformationException);
}

TEST_F(ScannerTest, testCoherentMonitoringFramesException)
{
  EXPECT_CALL(*udp_interface_ptr, read(_, _))
      .Times(5)
      .READ_FRAME(30)
      .READ_FRAME(31)
      .READ_FRAME(32)
      .READ_FRAME(33)
      .READ_FRAME(34);

  Scanner scanner("1.2.3.4",
                  0x01020305,
                  1234,
                  "p4sswort",
                  PSENscanInternalAngle(0),
                  PSENscanInternalAngle(2750),
                  std::move(udp_interface_ptr));

  EXPECT_THROW(scanner.getCompleteScan(), CoherentMonitoringFramesException);
  EXPECT_THROW(scanner.getCompleteScan(), ParseMonitoringFrameException);
  EXPECT_THROW(scanner.getCompleteScan(), CoherentMonitoringFramesException);
}

TEST_F(ScannerTest, testFetchMonitoringFrameException)
{
  EXPECT_CALL(*udp_interface_ptr, write(_)).Times(2);

  EXPECT_CALL(*udp_interface_ptr, read(_, _))
      .Times(4)
      .READ_FRAME(0)
      .WillOnce(DoAll(fillArg0(expected_monitoring_frames_.at(1)), Return(sizeof(MonitoringFrame) - 1)))
      .WillOnce(Throw(ScannerReadTimeout("")))
      .READ_FRAME(2);

  Scanner scanner("1.2.3.4",
                  0x01020305,
                  1234,
                  "p4sswort",
                  PSENscanInternalAngle(0),
                  PSENscanInternalAngle(2750),
                  std::move(udp_interface_ptr));

  EXPECT_THROW(scanner.getCompleteScan(), CoherentMonitoringFramesException);
}

TEST_F(ScannerTest, new_scanner)
{
  std::unique_ptr<Scanner> scanner = std::unique_ptr<Scanner>(new Scanner("1.2.3.4",
                                                                          0x01020305,
                                                                          1234,
                                                                          "p4sswort",
                                                                          PSENscanInternalAngle(0),
                                                                          PSENscanInternalAngle(2750),
                                                                          std::move(udp_interface_ptr)));
}
}  // namespace psen_scan_test
