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

#include "net_utils.h"

#include <string.h>

#ifdef WIN32
#include <winsock2.h>
#include <iphlpapi.h>
#include <mstcpip.h>
#pragma comment(lib, "IPHLPAPI.lib")
#pragma comment(lib, "ntdll.lib")
#else
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

namespace rc
{
using namespace std;

uint32_t ipToUInt(const std::string& ip)
{
  int a, b, c, d;
  uint32_t addr = 0;

  if (sscanf(ip.c_str(), "%d.%d.%d.%d", &a, &b, &c, &d) != 4)
  {
    return 0;
  }

  addr = a << 24;
  addr |= b << 16;
  addr |= c << 8;
  addr |= d;
  return addr;
}

bool isIPInRange(const std::string& ip, const std::string& network, const std::string& mask)
{
  uint32_t ip_addr = ipToUInt(ip);
  uint32_t network_addr = ipToUInt(network);
  uint32_t mask_addr = ipToUInt(mask);

  uint32_t net_lower = (network_addr & mask_addr);
  uint32_t net_upper = (net_lower | (~mask_addr));

  if (ip_addr >= net_lower && ip_addr <= net_upper)
  {
    return true;
  }

  return false;
}

#ifdef WIN32

bool getThisHostsIP(string& this_hosts_ip, const string& other_hosts_ip, const string& network_interface)
{
  this_hosts_ip = "";

  // convert string IP to integer representation

  DWORD dwOtherHostsIP = 0;
  if (other_hosts_ip.size() > 0)
  {
    dwOtherHostsIP = htonl(ipToUInt(other_hosts_ip));
  }

  // get index of network interface with given interface name

  DWORD ifindex = 0xffff;

  if (network_interface.size() > 0)
  {
    PIP_ADAPTER_ADDRESSES addr = 0;
    ULONG addr_size = 0;

    for (int i = 0; i < 3; i++)
    {
      if (addr_size > 0)
      {
        addr = reinterpret_cast<PIP_ADAPTER_ADDRESSES>(malloc(addr_size));
      }

      if (addr == 0)
      {
        addr_size = 0;
      }

      ULONG ret = GetAdaptersAddresses(AF_INET, 0, 0, addr, &addr_size);

      if (ret == ERROR_SUCCESS)
      {
        break;
      }

      free(addr);
      addr = 0;
    }

    PIP_ADAPTER_ADDRESSES p = addr;

    std::wstring wNetworkInterface(network_interface.begin(), network_interface.end());

    while (p != 0)
    {
      if (network_interface.compare(p->AdapterName) == 0 || wNetworkInterface.compare(p->FriendlyName) == 0)
      {
        ifindex = p->IfIndex;
        break;
      }

      p = p->Next;
    }

    free(addr);

    if (ifindex == 0xffff)
    {
      return false;
    }
  }

  // get table with IPv4 to network interface mappings

  PMIB_IPADDRTABLE table = 0;
  ULONG table_size = 0;

  for (int i = 0; i < 5; i++)
  {
    int result = GetIpAddrTable(table, &table_size, false);

    if (result == NO_ERROR)
    {
      break;
    }
    else if (result == ERROR_INSUFFICIENT_BUFFER)
    {
      free(table);
      table = static_cast<PMIB_IPADDRTABLE>(malloc(table_size));
    }
  }

  if (table == 0)
  {
    return false;
  }

  // got through table of mappings

  bool found_valid = false;
  for (unsigned int i = 0; i < table->dwNumEntries; i++)
  {
    PMIB_IPADDRROW row = &table->table[i];

    // filter out loopback device

    if (row->dwAddr == htonl(INADDR_LOOPBACK))
    {
      continue;
    }

    // if network interface name is given, then filter by this interface

    if (ifindex == 0xffff || ifindex == row->dwIndex)
    {
      // find network interface that can reach the specified other hosts IP

      if ((row->dwAddr & row->dwMask) == (dwOtherHostsIP & row->dwMask))
      {
        IN_ADDR addr;
        char tmp[80];
        addr.S_un.S_addr = row->dwAddr;
        RtlIpv4AddressToStringA(&addr, tmp);
        this_hosts_ip = string(tmp);

        found_valid = true;

        break;
      }
    }
  }

  // free resources

  free(table);

  return found_valid;
}

bool isValidIPAddress(const std::string& ip)
{
  LPCTSTR tp = 0;
  IN_ADDR addr;

  return RtlIpv4StringToAddressA(ip.c_str(), TRUE, &tp, &addr) == 0;
}

#else

bool getThisHostsIP(string& this_hosts_ip, const string& other_hosts_ip, const string& network_interface)
{
  // scan all network interfaces (for the desired one)
  struct ifaddrs* if_addr_struct = NULL;
  struct ifaddrs* ifa = NULL;
  void* tmp_addr_ptr = NULL;
  getifaddrs(&if_addr_struct);
  bool found_valid = false;
  char address_buffer[INET_ADDRSTRLEN], netmask_buffer[INET_ADDRSTRLEN];
  for (ifa = if_addr_struct; ifa != NULL; ifa = ifa->ifa_next)
  {
    // check if any valid IP4 address

    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET)
      continue;

    tmp_addr_ptr = &((struct sockaddr_in*)ifa->ifa_addr)->sin_addr;
    inet_ntop(AF_INET, tmp_addr_ptr, address_buffer, INET_ADDRSTRLEN);

    // if network interface name is given, then filter by this interface

    if (network_interface.size() == 0 || strcmp(network_interface.c_str(), ifa->ifa_name) == 0)
    {
      // find network interface that can reach the specified other hosts IP

      tmp_addr_ptr = &((struct sockaddr_in*)ifa->ifa_netmask)->sin_addr;
      inet_ntop(AF_INET, tmp_addr_ptr, netmask_buffer, INET_ADDRSTRLEN);
      if (isIPInRange(address_buffer, other_hosts_ip, netmask_buffer))
      {
        found_valid = true;
        break;
      }
    }
  }

  if (found_valid)
  {
    this_hosts_ip = string(address_buffer);
  }

  return found_valid;
}

bool isValidIPAddress(const std::string& ip)
{
  // use inet_pton to check if given string is a valid IP address
  static struct sockaddr_in sa;
  return TEMP_FAILURE_RETRY(inet_pton(AF_INET, ip.c_str(), &(sa.sin_addr))) == 1;
}

#endif
}
