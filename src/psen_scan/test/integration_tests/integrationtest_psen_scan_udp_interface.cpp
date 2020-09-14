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

#include <chrono>
#include <string>
#include <thread>
#include <future>

#include <boost/asio.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <pilz_testutils/async_test.h>

#include <psen_scan/psen_scan_udp_interface.h>
#include "psen_scan/mock_udp_server.h"
#include "psen_scan/udp_read_timeout_exception.h"
#include "psen_scan/fetch_monitoring_frame_exception.h"

using namespace psen_scan;
using namespace ::testing;
using boost::asio::ip::udp;

namespace psen_scan_test
{
static const std::string MSG_RECEIVED{ "msg_received" };

static const std::string IP_ADDRESS{ "127.0.0.1" };

static constexpr std::chrono::seconds READ_TIMEOUT{ std::chrono::seconds(1) };
static constexpr int WRITE_TIMEOUT_MS{ 2000 };

static constexpr uint32_t UDP_PORT{ 45000 };

class ScannerCommunicationInterfaceTests : public testing::Test, public testing::AsyncTest
{
public:
  void SetUp() override;

protected:
  std::future<std::size_t> startAsyncReadOperation(boost::asio::mutable_buffers_1& read_buf);

protected:
  MockUDPServer mock_udp_server_;
  PSENscanUDPInterface scanner_interface_{ IP_ADDRESS, UDP_PORT };
};

void ScannerCommunicationInterfaceTests::SetUp()
{
  scanner_interface_.open();
}

std::future<std::size_t>
ScannerCommunicationInterfaceTests::startAsyncReadOperation(boost::asio::mutable_buffers_1& read_buf)
{
  return std::async(std::launch::async,
                    [this, &read_buf]() { return scanner_interface_.read(read_buf, READ_TIMEOUT); });
}

TEST_F(ScannerCommunicationInterfaceTests, testScannerWriteOperation)
{
  EXPECT_CALL(mock_udp_server_, receivedUdpMsg()).WillOnce(ACTION_OPEN_BARRIER_VOID(MSG_RECEIVED));
  mock_udp_server_.startIOService();

  mock_udp_server_.asyncReceive();
  boost::array<char, 10> write_buf = { "Hello!" };
  scanner_interface_.write(boost::asio::buffer(write_buf));

  BARRIER(MSG_RECEIVED);
}

TEST_F(ScannerCommunicationInterfaceTests, testScannerReadOperation)
{
  constexpr std::size_t expected_length{ 15 };
  boost::array<char, expected_length> read_array;
  auto read_buffer{ boost::asio::buffer(read_array) };

  std::future<std::size_t> read_future{ startAsyncReadOperation(read_buffer) };

  const udp::endpoint send_endpoint(boost::asio::ip::address_v4::from_string(IP_ADDRESS), UDP_PORT);
  boost::array<char, expected_length> send_array = { "Hello answer" };

  mock_udp_server_.asyncSend<expected_length>(send_endpoint, send_array);
  ASSERT_EQ(std::future_status::ready, read_future.wait_for(READ_TIMEOUT)) << "Timeout while waiting for read()";
  ASSERT_EQ(expected_length, read_future.get()) << "Received message length incorrect";
  ASSERT_EQ(send_array, read_array) << "Send and received message are not equal";
}

}  // namespace psen_scan_test
