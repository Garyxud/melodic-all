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

#include <algorithm>
#include <iostream>

#include "psen_scan/scanner.h"
#include "psen_scan/scanner_data.h"
#include "psen_scan/fetch_monitoring_frame_exception.h"
#include "psen_scan/coherent_monitoring_frames_exception.h"
#include "psen_scan/diagnostic_information_exception.h"
#include "psen_scan/parse_monitoring_frame_exception.h"
#include "psen_scan/psen_scan_fatal_exception.h"
#include "psen_scan/udp_read_timeout_exception.h"
#include <psen_scan/timeout_adjust_func.h>

namespace psen_scan
{
constexpr std::chrono::seconds MIN_FETCH_FRAME_TIMEOUT{ std::chrono::seconds(1) };

bool isValidIpAddress(const char* ipAddress)
{
  struct sockaddr_in sa
  {
  };
  int result = inet_pton(AF_INET, ipAddress, &(sa.sin_addr));
  return result == 1;
}

/**
 * @brief Construct a new Scanner:: Scanner object
 *
 * @param scanner_ip IP-Adress of Laserscanner
 * @param host_ip IP-Adress of host machine
 * @param host_udp_port UDP Port on which packets from Laserscanner should be received
 * @param password Password for Laserscanner
 * @param angle_start Start angle of Laserscanner measurements in tenths of degree
 * @param angle_end End angle of Laserscanner measurements in tenths of degree
 * @param communication_interface Pointer to the communication interface
 */
Scanner::Scanner(const std::string& scanner_ip,
                 const uint32_t& host_ip,
                 const uint32_t& host_udp_port,
                 const std::string& password,
                 const PSENscanInternalAngle& angle_start,
                 const PSENscanInternalAngle& angle_end,
                 std::unique_ptr<ScannerCommunicationInterface> communication_interface)
  : scanner_ip_(scanner_ip)
  , start_monitoring_frame_(password, host_ip, host_udp_port)
  , stop_monitoring_frame_(password)
  , angle_start_(angle_start)
  , angle_end_(angle_end)
  , previous_monitoring_frame_({})
  , communication_interface_(std::move(communication_interface))
{
  if (!isValidIpAddress(scanner_ip_.c_str()))
  {
    throw PSENScanFatalException("Scanner IP is invalid!");
  }

  if (host_udp_port > 65535)  // MAX_UINT16
  {
    throw PSENScanFatalException("Host UDP Port is too big!");
  }

  if (host_udp_port < 1024)
  {
    std::cout << "Attention: UDP Port is in IANA Standard Port range (below 1024)! "
              << "Please consider using a port number above 1024." << std::endl;
  }

  if (angle_start >= angle_end)
  {
    throw PSENScanFatalException("Attention: Start angle has to be smaller than end angle!");
  }

  if (angle_end > MAX_SCAN_ANGLE)
  {
    throw PSENScanFatalException("Attention: End angle has to be smaller than the physical Maximum!");
  }

  if (!communication_interface_)
  {
    throw PSENScanFatalException("Nullpointer isn't a valid argument!");
  }

  communication_interface_->open();
}

/**
 * @brief Send start signal to Laserscanner
 *
 */
void Scanner::start()
{
  communication_interface_->write(boost::asio::buffer(&start_monitoring_frame_, sizeof(StartMonitoringFrame)));
}

/**
 * @brief Send stop signal to Laserscanner
 *
 */
void Scanner::stop()
{
  communication_interface_->write(boost::asio::buffer(&stop_monitoring_frame_, sizeof(StopMonitoringFrame)));
}

/**
 * @brief Gets one MonitoringFrame from Laserscanner
 *
 * @return MonitoringFrame
 *
 * @throws FetchMonitoringFrameException
 */
MonitoringFrame Scanner::fetchMonitoringFrame(std::chrono::steady_clock::duration timeout)
{
  MonitoringFrame monitoring_frame;
  std::size_t bytes_received;
  auto buf = boost::asio::buffer(&monitoring_frame, sizeof(MonitoringFrame));
  try
  {
    bytes_received = communication_interface_->read(buf, timeout);
    if (bytes_received != sizeof(MonitoringFrame))
    {
      throw FetchMonitoringFrameException("Received Frame length doesn't match MonitoringFrame length!");
    }
  }
  catch (const ScannerReadTimeout& e)
  {
    stop();
    sleep(1);
    start();
    throw FetchMonitoringFrameException(e.what() + static_cast<std::string>(" Restarting Scanner!"));
  }
  return monitoring_frame;
}

/**
 * @brief Parses a MonitoringFrame to check whether all fields are as expected
 *
 * @param monitoring_frame MonitoringFrame to check
 * @return true Fields are ok
 * @return false At least one field is not ok
 *
 * @throws ParseMonitoringFrameException
 */
bool Scanner::parseFields(const MonitoringFrame& monitoring_frame)
{
  if (monitoring_frame.opcode_ != MONITORING_FRAME_OPCODE)
  {
    previous_monitoring_frame_ = monitoring_frame;
    throw ParseMonitoringFrameException("MonitoringFrame's Opcode doesn't match expected value!");
  }

  if (monitoring_frame.scanner_id_ != 0)
  {
    previous_monitoring_frame_ = monitoring_frame;
    std::string err = "MonitoringFrame's ScannerID doesn't belong to master! \n";
    err.append("Please contact the maintainer if you need master+slave functionality!");
    throw ParseMonitoringFrameException(err);
  }

  if (monitoring_frame.number_of_samples_ > MAX_NUMBER_OF_SAMPLES)
  {
    previous_monitoring_frame_ = monitoring_frame;
    throw ParseMonitoringFrameException("MonitoringFrame's number of samples exceeds the maximum allowed amount!");
  }

  return true;
}

/**
 * @brief Checks if DiagnosticInformation Bitfield contains no errors
 *
 * @param diag_info DiagnosticInformation Bitfield to check
 * @return true DiagnosticInformation Bitfield contains no errors
 * @return false DiagnosticInformation Bitfield contains atleast one error
 *
 * @throws DiagnosticInformationException
 */
bool Scanner::isDiagnosticInformationOk(const DiagnosticInformation& diag_info)
{
  if (diag_info.ossd1_short_circuit_)
  {
    throw DiagnosticInformationException("OSSD1 Overcurrent/Short Circuit!");
  }

  if (diag_info.short_circuit_at_least_two_ossd_)
  {
    throw DiagnosticInformationException("Short Circuit at least between two OSSDs!");
  }

  if (diag_info.integrity_check_problem_on_any_ossd_)
  {
    throw DiagnosticInformationException("Integrity check problem on any OSSD!");
  }

  if (diag_info.internal_error_1_ || diag_info.internal_error_2_ || diag_info.internal_error_3_ ||
      diag_info.internal_error_4_ || diag_info.internal_error_5_)
  {
    throw DiagnosticInformationException("Internal Error!");
  }

  if (diag_info.window_cleaning_alarm_)
  {
    throw DiagnosticInformationException("Window cleaning alarm!");
  }

  if (diag_info.power_supply_problem_)
  {
    throw DiagnosticInformationException("Power supply problem!");
  }

  if (diag_info.network_problem_)
  {
    throw DiagnosticInformationException("Network problem!");
  }

  if (diag_info.dust_circuit_failure_)
  {
    throw DiagnosticInformationException("Dust circuit failure!");
  }

  if (diag_info.measure_problem_)
  {
    throw DiagnosticInformationException("Measure problem!");
  }

  if (diag_info.incoherence_data_)
  {
    throw DiagnosticInformationException("Incoherence data!");
  }

  if (diag_info.zone_invalid_input_transition_or_integrity_)
  {
    throw DiagnosticInformationException("Zone: Invalid Input transition or integrity!");
  }

  if (diag_info.zone_invalid_input_configuration_connection_)
  {
    throw DiagnosticInformationException("Zone: Invalid Input configuration/connection!");
  }

  if (diag_info.window_cleaning_warning_)
  {
    throw DiagnosticInformationException("Window cleaning warning!");
  }

  if (diag_info.internal_communication_problem_)
  {
    throw DiagnosticInformationException("Internal communication problem!");
  }

  if (diag_info.generic_error_)
  {
    throw DiagnosticInformationException("Generic error!");
  }

  if (diag_info.display_communication_problem_)
  {
    throw DiagnosticInformationException("Display commuication problem!");
  }

  if (diag_info.temperature_measurement_problem_)
  {
    throw DiagnosticInformationException("Temperature measurement problem!");
  }

  if (diag_info.configuration_error_)
  {
    throw DiagnosticInformationException("Configuration error!");
  }

  if (diag_info.out_of_range_error_)
  {
    throw DiagnosticInformationException("Out of range error!");
  }

  if (diag_info.temperature_range_error_)
  {
    throw DiagnosticInformationException("Temperature range error!");
  }

  return true;
}

/**
 * @brief Reads from UDP Interface until complete laserscan object can be formed.
 *
 * @return LaserScan Complete laserscan object
 *
 * @throws CoherentMonitoringFramesException
 */
LaserScan Scanner::getCompleteScan()
{
  LaserScan scan(PSENscanInternalAngle(0), angle_start_, angle_end_);

  MonitoringFrame monitoring_frame;
  bool firstrun = true;
  do
  {
    std::chrono::steady_clock::duration fetch_frame_timeout{ MIN_FETCH_FRAME_TIMEOUT };
    bool exception_occured = false;
    do
    {
      exception_occured = false;
      try
      {
        monitoring_frame = fetchMonitoringFrame(fetch_frame_timeout);
        parseFields(monitoring_frame);
        isDiagnosticInformationOk(monitoring_frame.diagnostic_area_.diagnostic_information_);
      }
      catch (const FetchMonitoringFrameException& e)
      {
        std::cerr << e.what() << '\n';
        exception_occured = true;
        fetch_frame_timeout = psen_scan_utils::adjustTimeout(fetch_frame_timeout);
      }
    } while (exception_occured);

    if (firstrun && MIN_SCAN_ANGLE != PSENscanInternalAngle(monitoring_frame.from_theta_))
    {
      previous_monitoring_frame_ = monitoring_frame;
      throw CoherentMonitoringFramesException("First Monitoring frame missing!");
    }

    if (!firstrun && monitoring_frame.from_theta_ < previous_monitoring_frame_.from_theta_)
    {
      previous_monitoring_frame_ = monitoring_frame;
      throw CoherentMonitoringFramesException("New Cycle has begun! Disregard old values!");
    }

    if (PSENscanInternalAngle(monitoring_frame.from_theta_) != MIN_SCAN_ANGLE &&
        previous_monitoring_frame_.resolution_ != monitoring_frame.resolution_)
    {
      previous_monitoring_frame_ = monitoring_frame;
      throw CoherentMonitoringFramesException("Resolution of new MonitoringFrame doesn't match previous "
                                              "resolution(s)!");
    }

    if (!firstrun && ((monitoring_frame.to_theta() != MAX_SCAN_ANGLE &&
                       previous_monitoring_frame_.scan_counter_ != monitoring_frame.scan_counter_) ||
                      (monitoring_frame.to_theta() == MAX_SCAN_ANGLE &&
                       previous_monitoring_frame_.scan_counter_ + 1 != monitoring_frame.scan_counter_)))
    {
      previous_monitoring_frame_ = monitoring_frame;
      throw CoherentMonitoringFramesException("ScanCounter of new MonitoringFrame doesn't match previous ScanCounter!");
    }

    if (MIN_SCAN_ANGLE != PSENscanInternalAngle(monitoring_frame.from_theta_) &&
        previous_monitoring_frame_.to_theta() != PSENscanInternalAngle(monitoring_frame.from_theta_))
    {
      previous_monitoring_frame_ = monitoring_frame;
      throw CoherentMonitoringFramesException("Start angle of new MonitoringFrame doesn't match angle of previous "
                                              "MonitoringFrame!");
    }

    uint16_t length = std::min(monitoring_frame.number_of_samples_,
                               MAX_NUMBER_OF_SAMPLES);  // TODO(gsansone): Is Exception. Remove check?
    scan.measures_.insert(
        scan.measures_.end(), monitoring_frame.measures_.begin(), monitoring_frame.measures_.begin() + length);

    previous_monitoring_frame_ = monitoring_frame;
    firstrun = false;  // next run is the second run or higher
  } while (monitoring_frame.to_theta() != MAX_SCAN_ANGLE);
  scan.resolution_ = PSENscanInternalAngle(monitoring_frame.resolution_);

  auto begin_position = static_cast<int>(angle_start_) / static_cast<int>(scan.resolution_);
  if (begin_position > 0)
  {
    scan.measures_.erase(scan.measures_.begin(), scan.measures_.begin() + begin_position);
  }

  PSENscanInternalAngle temp_max_scan_angle = MAX_SCAN_ANGLE;
  auto end_position = static_cast<int>(temp_max_scan_angle - angle_end_) / static_cast<int>(scan.resolution_);
  if (end_position > 0)
  {
    scan.measures_.erase(scan.measures_.end() - end_position, scan.measures_.end());
  }

  return scan;
}

}  // namespace psen_scan
