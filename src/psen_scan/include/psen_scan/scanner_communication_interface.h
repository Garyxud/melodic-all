// Copyright (c) 2020 Pilz GmbH & Co. KG
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

#ifndef SCANNER_COMMUNICATION_INTERFACE_H
#define SCANNER_COMMUNICATION_INTERFACE_H

#include <stdexcept>
#include <chrono>

#include <boost/asio.hpp>

namespace psen_scan
{
class ScannerOpenFailed : public std::runtime_error
{
public:
  ScannerOpenFailed(const std::string error_desc) : std::runtime_error(error_desc)
  {
  }
};

class ScannerWriteFailed : public std::runtime_error
{
public:
  ScannerWriteFailed(const std::string error_desc) : std::runtime_error(error_desc)
  {
  }
};

class ScannerReadTimeout : public std::runtime_error
{
public:
  ScannerReadTimeout(const std::string error_desc) : std::runtime_error(error_desc)
  {
  }
};

class ScannerReadFailed : public std::runtime_error
{
public:
  ScannerReadFailed(const std::string error_desc) : std::runtime_error(error_desc)
  {
  }
};

class ScannerCloseFailed : public std::runtime_error
{
public:
  ScannerCloseFailed(const std::string error_desc) : std::runtime_error(error_desc)
  {
  }
};

// LCOV_EXCL_START
/**
 * @brief Abstract base class for the communication interface with the PSENscan scanner.
 */
class ScannerCommunicationInterface
{
public:
  virtual ~ScannerCommunicationInterface() = default;

public:
  //! @brief Opens the connection to the scanner device.
  virtual void open() = 0;
  //! @brief Closes the connection to the scanner device.
  virtual void close() = 0;

  //! @brief Sends data to the scanner device.
  //! @param buffer Data which have to be send to the scanner device.
  virtual void write(const boost::asio::mutable_buffers_1& buffer) = 0;

  //! @brief Receive data from the scanner.
  //! @param buffer Buffer which contains the data received from the scanner device after the function call is done.
  //! @param timeout Defines how long to wait for new messages from the scanner device.
  //! @returns the number of bytes received from the scanner device.
  //! @throws UDPReadTimeoutException inidcating a timeout while reading data from the scanner device.
  virtual std::size_t read(boost::asio::mutable_buffers_1& buffer,
                           const std::chrono::steady_clock::duration timeout) = 0;
};
// LCOV_EXCL_STOP

}  // namespace psen_scan

#endif  // SCANNER_COMMUNICATION_INTERFACE_H
