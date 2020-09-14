// -*- C++ -*-
/*!
 * @file  Routing.h
 * @brief Network routing information handling functions
 * @date  $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2010
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef COIL_ROUTING_H
#define COIL_ROUTING_H

#include <string>
 
namespace coil
{
  /*!
   * @if jp
   * @brief 宛先アドレスから利用されるエンドポイントアドレスを得る
   * 
   * 宛先アドレスを与えると、その際に利用されるエンドポイントのアドレス
   * が返される。宛先アドレス dest_addr には、IPアドレスまたはFQDNホス
   * ト名を与えることができる。宛先アドレスが到
   * 達可能であり、利用するエンドポイントが得られた場合 true、宛先アド
   * レスに到達できない場合は false が返される。
   *
   * @param dest_addr 宛先アドレスまたはホスト名
   * @param dest_if 宛先と通信を行う際に使用されるインターフェース名
   * @return 成功 true, 失敗 false
   * 
   * @else
   * @brief Getting network interface name from destination address
   *
   * This operation returns IP address of a endpoint to be used to
   * communicate with the given destination address. IP address and
   * FQDN hostname are available for the destination address
   * dest_addr. If a destination address are reachable and an endpoint
   * IP address is available, this operation returns true, and
   * otherwise false.
   *
   * @param dest_addr a destination address or host name
   * @param endpoint a IP address of the endpoint to be used to communicate
   *                 with the destination address
   * @return successful: true, failed: false
   *
   * @endif
   */
  bool dest_to_endpoint(std::string dest_addr, std::string& endpoint);

}; //namespace coil
#endif // COIL_ROUTING_H
