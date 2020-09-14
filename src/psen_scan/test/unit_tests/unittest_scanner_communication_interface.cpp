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

#include <string>
#include <memory>
#include <chrono>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <gtest/gtest.h>

#include <psen_scan/scanner_communication_interface.h>
#include <psen_scan/psen_scan_udp_interface.h>
#include <psen_scan/udp_read_timeout_exception.h>

using namespace psen_scan;

constexpr uint32_t UDP_PORT{ 55000 };
const std::string IP_ADDR{ "127.0.0.1" };

TEST(ScannerCommunicationIntefaceTests, testFailingWriteOperation)
{
  PSENscanUDPInterface comm_interface(IP_ADDR, UDP_PORT);

  boost::array<char, 10> write_buf = { "Hello!" };
  ASSERT_THROW(comm_interface.write(boost::asio::buffer(write_buf)), ScannerWriteFailed);
}

TEST(ScannerCommunicationIntefaceTests, testReadTimeout)
{
  PSENscanUDPInterface comm_interface(IP_ADDR, UDP_PORT + 1);
  comm_interface.open();

  constexpr auto read_timeout{ std::chrono::seconds(3) };
  boost::array<char, 10> read_array;
  auto read_buf{ boost::asio::buffer(read_array) };
  ASSERT_THROW(comm_interface.read(read_buf, read_timeout), ScannerReadTimeout);
}

TEST(ScannerCommunicationIntefaceTests, testScannerOpenFailedForCompleteCoverage)
{
  const std::string except_str{ "DummyText" };
  std::unique_ptr<ScannerOpenFailed> e(new ScannerOpenFailed(except_str));
  EXPECT_EQ(except_str, e->what());
}

TEST(ScannerCommunicationIntefaceTests, testScannerWriteFailedForCompleteCoverage)
{
  const std::string except_str{ "DummyText" };
  std::unique_ptr<ScannerWriteFailed> e(new ScannerWriteFailed(except_str));
  EXPECT_EQ(except_str, e->what());
}

TEST(ScannerCommunicationIntefaceTests, testScannerScannerReadTimeoutForCompleteCoverage)
{
  const std::string except_str{ "DummyText" };
  std::unique_ptr<ScannerReadTimeout> e(new ScannerReadTimeout(except_str));
  EXPECT_EQ(except_str, e->what());
}

TEST(ScannerCommunicationIntefaceTests, testScannerReadFailedForCompleteCoverage)
{
  const std::string except_str{ "DummyText" };
  std::unique_ptr<ScannerReadFailed> e(new ScannerReadFailed(except_str));
  EXPECT_EQ(except_str, e->what());
}

TEST(ScannerCommunicationIntefaceTests, testScannerCloseFailedForCompleteCoverage)
{
  const std::string except_str{ "DummyText" };
  std::unique_ptr<ScannerCloseFailed> e(new ScannerCloseFailed(except_str));
  EXPECT_EQ(except_str, e->what());
}
