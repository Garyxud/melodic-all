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

#ifndef PSEN_SCAN_UDP_INTERFACE_H
#define PSEN_SCAN_UDP_INTERFACE_H

#include <chrono>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <psen_scan/scanner_communication_interface.h>
#include "psen_scan/scanner_data.h"

namespace psen_scan
{
/**
 * @brief Class for the UDP communication with the scanner.
 *
 */
class PSENscanUDPInterface : public ScannerCommunicationInterface
{
public:
  PSENscanUDPInterface(const std::string& scanner_ip,
                       const uint32_t& host_udp_port,
                       const unsigned short scanner_port_write = PSEN_SCAN_PORT_WRITE,
                       const unsigned short scanner_port_read = PSEN_SCAN_PORT_READ);

  virtual ~PSENscanUDPInterface();

public:
  void open() override;
  void close() override;

  void write(const boost::asio::mutable_buffers_1& buffer) override;
  std::size_t read(boost::asio::mutable_buffers_1& buffer, const std::chrono::steady_clock::duration timeout) override;

private:
  bool isUdpMsgAvailable() const;

private:
  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_write_;
  boost::asio::ip::udp::socket socket_read_;
  boost::asio::ip::udp::endpoint udp_write_endpoint_;
  boost::asio::ip::udp::endpoint udp_read_endpoint_;
};

}  // namespace psen_scan

#endif  // PSEN_SCAN_UDP_INTERFACE_H
