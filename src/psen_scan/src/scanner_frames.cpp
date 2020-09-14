// Copyright (c) 2019 Pilz GmbH & Co. KG
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

#include <cstring>
#include "psen_scan/scanner_frames.h"
#include "psen_scan/scanner_data.h"
#include <boost/crc.hpp>

namespace psen_scan
{
/**
 * @brief Construct a new Start Monitoring Frame:: Start Monitoring Frame object
 *
 * @param password Password for Laserscanner
 * @param host_ip IP-Address the Laserscanner should send to
 * @param host_udp_port UDP-Port the Laserscanner should send to
 */
StartMonitoringFrame::StartMonitoringFrame(const std::string& password,
                                           const uint32_t& host_ip,
                                           const uint32_t& host_udp_port)
  : RESERVED_(0)
  , password_("")
  , OPCODE_(START_MONITORING_OPCODE)
  , host_ip_(host_ip)
  , host_udp_port_(host_udp_port)
  , FIXED_SEQUENCE_(START_MONITORING_FIXED_SEQUENCE)
  , RESERVED2_({ 0 })
{
  strncpy(password_, password.c_str(), sizeof(password_));

  boost::crc_32_type result;
  result.process_bytes(&RESERVED_, sizeof(StartMonitoringFrame) - sizeof(crc_));
  crc_ = result.checksum();
}

/**
 * @brief Construct a new Stop Monitoring Frame:: Stop Monitoring Frame object
 *
 * @param password Password for Laserscanner
 */
StopMonitoringFrame::StopMonitoringFrame(const std::string& password)
  : RESERVED_(0), password_(""), OPCODE_(STOP_MONITORING_OPCODE)
{
  strncpy(password_, password.c_str(), sizeof(password_));

  boost::crc_32_type result;
  result.process_bytes(&RESERVED_, sizeof(StopMonitoringFrame) - sizeof(crc_));
  crc_ = result.checksum();
}

}  // namespace psen_scan