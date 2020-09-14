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

#ifndef PSEN_SCAN_SCANNER_H
#define PSEN_SCAN_SCANNER_H

#include <memory>

#include "psen_scan/scanner_frames.h"
#include "psen_scan/scanner_communication_interface.h"
#include "psen_scan/laserscan.h"

namespace psen_scan
{
// LCOV_EXCL_START
class vScanner
{
public:
  virtual ~vScanner() = default;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual LaserScan getCompleteScan() = 0;
};
// LCOV_EXCL_STOP

/**
 * @brief Class for Modeling a PSENscan safety laser scanner
 *
 */
class Scanner : public vScanner
{
public:
  Scanner(const std::string& scanner_ip,
          const uint32_t& host_ip,
          const uint32_t& host_udp_port,
          const std::string& password,
          const PSENscanInternalAngle& angle_start,
          const PSENscanInternalAngle& angle_end,
          std::unique_ptr<ScannerCommunicationInterface> communication_interface);

  void start();
  void stop();
  LaserScan getCompleteScan();

private:
  std::string scanner_ip_;                      /**< IP-Adress of Laserscanner */
  StartMonitoringFrame start_monitoring_frame_; /**< Start Monitoring Command Frame */
  StopMonitoringFrame stop_monitoring_frame_;   /**< Stop Monitoring Command Frame */
  PSENscanInternalAngle angle_start_;           /**< Start angle of Laserscanner measurements */
  PSENscanInternalAngle angle_end_;             /**< End angle of Laserscanner measurements */
  MonitoringFrame previous_monitoring_frame_;   /**< Buffer for incoming Laserscanner data */
  std::unique_ptr<ScannerCommunicationInterface> communication_interface_;

private:
  MonitoringFrame fetchMonitoringFrame(std::chrono::steady_clock::duration timeout);
  bool isDiagnosticInformationOk(const DiagnosticInformation& diag_info);
  bool parseFields(const MonitoringFrame& monitoring_frame);
};

}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_H
