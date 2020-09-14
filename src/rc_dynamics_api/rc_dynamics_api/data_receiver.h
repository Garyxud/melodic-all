/*
 * This file is part of the rc_dynamics_api package.
 *
 * Copyright (c) 2017 Roboception GmbH
 * All rights reserved
 *
 * Author: Christian Emmerich
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RC_DYNAMICS_API_DATASTREAM_H
#define RC_DYNAMICS_API_DATASTREAM_H

#include <memory>
#include <sstream>

#ifdef WIN32
#include <winsock2.h>
#else
#include <netinet/in.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <arpa/inet.h>
#endif

#include <string.h>
#include <functional>

#include "net_utils.h"
#include "socket_exception.h"

#include "roboception/msgs/frame.pb.h"
#include "roboception/msgs/dynamics.pb.h"
#include "roboception/msgs/imu.pb.h"

namespace rc
{
namespace dynamics
{
/**
 * A simple receiver object for handling data streamed by rc_visard's
 * rc_dynamics module.
 */
class DataReceiver : public std::enable_shared_from_this<DataReceiver>
{
public:
  using Ptr = std::shared_ptr<DataReceiver>;

  /**
   * Creates a data receiver bound to the user-given IP address and port
   * number.
   *
   * For binding to an arbitrary port, the given port number might be 0. In
   * this case, the actually chosen port number is returned.
   *
   * @param ip_address IP address for receiving data
   * @param port port number for receiving data
   * @return
   */
  static Ptr create(const std::string& ip_address, unsigned int& port)
  {
    return Ptr(new DataReceiver(ip_address, port));
  }

  virtual ~DataReceiver()
  {
#ifdef WIN32
    closesocket(_sockfd);
#else
    close(_sockfd);
#endif
  }

  /**
   * Returns Ip address for which the receiver was created
   */
  std::string getIpAddress() const {
    return ip_;
  }

  /**
   * Returns port  for which the receiver was created
   */
  unsigned int getPort() const {
    return port_;
  }

  /**
   * Sets a user-specified timeout for the receivePose() method.
   *
   * @param ms timeout in milliseconds
   */
  virtual void setTimeout(unsigned int ms)
  {
#ifdef WIN32
    DWORD timeout = ms;
    if (setsockopt(_sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout)) < 0)
    {
      throw SocketException("Error while setting receive timeout!", errno);
    }
#else
    struct timeval _recvtimeout;
    _recvtimeout.tv_sec = ms / 1000;
    _recvtimeout.tv_usec = (ms % 1000) * 1000;
    if (setsockopt(_sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&_recvtimeout, sizeof(struct timeval)) < 0)
    {
      throw SocketException("Error while setting receive timeout!", errno);
    }
#endif
  }

  /**
   * Receives the next message from data stream (template-parameter version)
   *
   * This method blocks until the next message is received and returns it as
   * specified by the template parameter PbMsgType, or when it runs into
   * user-specified timeout (see setTimeout(...)).
   *
   * NOTE: The specified PbMsgType *must match* the type with which the
   * received data was serialized during sending. Otherwise it will result in
   * undefined behaviour!
   *
   * @return the next rc_dynamics data stream message as PbMsgType, or NULL if timeout
   */
  template <class PbMsgType>
  std::shared_ptr<PbMsgType> receive()
  {
// receive msg from socket; blocking call (timeout)
#ifdef WIN32
    int msg_size = recvfrom(_sockfd, _buffer, sizeof(_buffer), 0, NULL, NULL);

    if (msg_size < 0)
    {
      int e = WSAGetLastError();
      if (e == WSAETIMEDOUT)
      {
        // timeouts are allowed to happen, then return NULL pointer
        return nullptr;
      }
      else
      {
        throw SocketException("Error during socket recvfrom!", e);
      }
    }
#else
    int msg_size = TEMP_FAILURE_RETRY(recvfrom(_sockfd, _buffer, sizeof(_buffer), 0, NULL, NULL));

    if (msg_size < 0)
    {
      int e = errno;
      if (e == EAGAIN || e == EWOULDBLOCK)
      {
        // timeouts are allowed to happen, then return NULL pointer
        return nullptr;
      }
      else
      {
        throw SocketException("Error during socket recvfrom!", e);
      }
    }
#endif

    // parse msgs as probobuf
    auto pb_msg = std::shared_ptr<PbMsgType>(new PbMsgType());
    pb_msg->ParseFromArray(_buffer, msg_size);
    return pb_msg;
  }

  /**
   * Receives the next message from data stream (string-parameter version)
   *
   * This method blocks until the next message is available and returns it -
   * de-serialized as specified by the pb_msg_type parameter - as a pb::Message
   * base class pointer, or until it runs into user-specified timeout (see
   * setTimeout(...)).
   *
   * NOTE: The specified PbMsgType *must match* the type with which the
   * received data was serialized during sending. Otherwise it will result in
   * undefined behaviour!
   *
   * @return the next rc_dynamics data stream message as a pb::Message base class pointer, or NULL if timeout
   */
  virtual std::shared_ptr<::google::protobuf::Message> receive(const std::string& pb_msg_type)
  {
    auto found = _recv_func_map.find(pb_msg_type);
    if (found == _recv_func_map.end())
    {
      std::stringstream msg;
      msg << "Unsupported protobuf message type '" << pb_msg_type << "'. Only the following types are supported: ";
      for (auto const& p : _recv_func_map)
        msg << p.first << " ";
      throw std::invalid_argument(msg.str());
    }
    return _recv_func_map[pb_msg_type]();
  }

protected:
  DataReceiver(const std::string& ip_address, unsigned int& port) : ip_(ip_address), port_(port)
  {
    // check if given string is a valid IP address
    if (!rc::isValidIPAddress(ip_address))
    {
      throw std::invalid_argument("Given IP address is not a valid address: " + ip_address);
    }

    // open socket for UDP listening
    _sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#ifdef WIN32
    if (_sockfd == INVALID_SOCKET)
#else
    if (_sockfd < 0)
#endif
    {
      throw SocketException("Error while creating socket!", errno);
    }

    // bind socket to IP address and port number
    struct sockaddr_in myaddr;
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = inet_addr(ip_address.c_str());  // set IP addrs
    myaddr.sin_port = htons(static_cast<u_short>(port));
    if (bind(_sockfd, (sockaddr*)&myaddr, sizeof(sockaddr)) < 0)
    {
      throw SocketException("Error while binding socket!", errno);
    }

    // if socket was bound to arbitrary port, we need to figure out to which
    // port number
    if (port == 0)
    {
#ifdef WIN32
      int len = sizeof(myaddr);
#else
      socklen_t len = sizeof(myaddr);
#endif

      if (getsockname(_sockfd, (struct sockaddr*)&myaddr, &len) < 0)
      {
#ifdef WIN32
        closesocket(_sockfd);
#else
        close(_sockfd);
#endif

        throw SocketException("Error while getting socket name!", errno);
      }
      port_ = port = ntohs(myaddr.sin_port);
    }

    // register all known protobuf message types
    _recv_func_map[roboception::msgs::Frame::descriptor()->name()] =
        std::bind(&DataReceiver::receive<roboception::msgs::Frame>, this);
    _recv_func_map[roboception::msgs::Imu::descriptor()->name()] =
        std::bind(&DataReceiver::receive<roboception::msgs::Imu>, this);
    _recv_func_map[roboception::msgs::Dynamics::descriptor()->name()] =
        std::bind(&DataReceiver::receive<roboception::msgs::Dynamics>, this);
  }

#ifdef WIN32
  SOCKET _sockfd;
#else
  int _sockfd;
#endif

  char _buffer[512];

  typedef std::map<std::string, std::function<std::shared_ptr<::google::protobuf::Message>()>> map_type;
  map_type _recv_func_map;

  std::string ip_;
  unsigned int port_;
};
}
}

#endif  // RC_DYNAMICS_API_DATASTREAM_H
