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

#include "psen_scan/psen_scan_udp_interface.h"
#include "psen_scan/udp_read_timeout_exception.h"
#include <boost/chrono/chrono_io.hpp>
#include <chrono>
#include <thread>

using boost::asio::ip::udp;

namespace psen_scan
{
static const uint64_t TIMEOUT_LOOP_SLEEP_DURATION_MS = 5;

PSENscanUDPInterface::PSENscanUDPInterface(const std::string& scanner_ip,
                                           const uint32_t& host_udp_port,
                                           const unsigned short scanner_port_write,
                                           const unsigned short scanner_port_read)
  : socket_write_(io_service_, udp::endpoint(udp::v4(), host_udp_port + 1))
  , socket_read_(io_service_, udp::endpoint(udp::v4(), host_udp_port))
  , udp_write_endpoint_(boost::asio::ip::address_v4::from_string(scanner_ip), scanner_port_write)
  , udp_read_endpoint_(boost::asio::ip::address_v4::from_string(scanner_ip), scanner_port_read)
{
}

PSENscanUDPInterface::~PSENscanUDPInterface()
{
  close();
}

void PSENscanUDPInterface::open()
{
  try
  {
    socket_write_.connect(udp_write_endpoint_);
    socket_read_.connect(udp_read_endpoint_);
  }
  // LCOV_EXCL_START
  catch (const boost::system::system_error& ex)
  {
    throw ScannerOpenFailed(ex.what());
  }
  // LCOV_EXCL_STOP
}

void PSENscanUDPInterface::close()
{
  try
  {
    socket_write_.close();
    socket_read_.close();
  }
  // LCOV_EXCL_START
  catch (const boost::system::system_error& ex)
  {
    throw ScannerCloseFailed(ex.what());
  }
  // LCOV_EXCL_STOP
}

void PSENscanUDPInterface::write(const boost::asio::mutable_buffers_1& buffer)
{
  try
  {
    socket_write_.send(buffer);
  }
  // LCOV_EXCL_START
  catch (const boost::system::system_error& ex)
  {
    throw ScannerWriteFailed(ex.what());
  }
  // LCOV_EXCL_STOP
}

bool PSENscanUDPInterface::isUdpMsgAvailable() const
{
  return socket_read_.available() > 0u;
}

std::size_t PSENscanUDPInterface::read(boost::asio::mutable_buffers_1& buffer,
                                       const std::chrono::steady_clock::duration timeout)
{
  const auto start_time{ std::chrono::steady_clock::now() };
  while (!isUdpMsgAvailable())
  {
    if ((std::chrono::steady_clock::now() - start_time) > timeout)
    {
      throw ScannerReadTimeout("Timeout while waiting for new UDP message");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(TIMEOUT_LOOP_SLEEP_DURATION_MS));
  }

  std::size_t bytes_read{ 0 };
  try
  {
    bytes_read = socket_read_.receive(buffer);
  }
  // LCOV_EXCL_START
  catch (const boost::system::system_error& ex)
  {
    throw ScannerReadFailed(ex.what());
  }
  // LCOV_EXCL_STOP

  return bytes_read;
}

}  // namespace psen_scan
