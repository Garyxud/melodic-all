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

#ifndef RC_DEVICE_NET_UTILS_H
#define RC_DEVICE_NET_UTILS_H

#include <string>
#include <stdint.h>

namespace rc
{
/**
 * Converts a string-represented ip into uint (e.g. for subnet masking)
 *  taken from: https://www.stev.org/post/ccheckanipaddressisinaipmask
 * @param ip
 * @return
 */
uint32_t ipToUInt(const std::string& ip);

/**
 * Checks if a given ip is in range of a network defined by ip/subnet
 *  taken from: https://www.stev.org/post/ccheckanipaddressisinaipmask
 * @param ip
 * @param network
 * @param mask
 * @return
 */
bool isIPInRange(const std::string& ip, const std::string& network, const std::string& mask);

/**
 * Convenience function to scan this host's (multiple) network interface(s) for
 * a valid IP address.
 * Users may give a hint either be specifying the preferred network interface
 * to be used, or the IP address of another host that should be reachable from
 * the returned IP address.
 *
 * @param this_hosts_ip IP address to be used as stream destination (only valid if returned true)
 * @param other_hosts_ip rc_visard's IP address, e.g. "192.168.0.20"
 * @param network_interface name, e.g. eth0, wlan0, ...
 * @return true if valid IP address was found among network interfaces
 */
bool getThisHostsIP(std::string& this_hosts_ip, const std::string& other_hosts_ip,
                    const std::string& network_interface = "");

/**
 * Checks if given string is a valid IP address
 * @param ip IP address to be checked
 * @return true if valid
 */
bool isValidIPAddress(const std::string& ip);
}

#endif  // RC_DEVICE_NET_UTILS_H
