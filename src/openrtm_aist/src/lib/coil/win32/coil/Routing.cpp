// -*- C++ -*-
/*!
 * @file  Routing.cpp
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

#ifndef NTDDI_VERSION
#define NTDDI_VERSION 0x05000000
#define WINVER _WIN32_WINNT
#ifdef _WIN32_WINNT
#undef _WIN32_WINNT
#endif // _WIN32_WINNT
#define _WIN32_WINNT 0x0500
#define _WIN32_WINDOWS _WIN32_WINNT
#define _WIN32_IE 0x0501
#endif // NTDDI_VERSION

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>

#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "ws2_32.lib")

#include <coil/Routing.h>
#include <coil/stringutil.h>
#include <coil/config_coil.h>

#define MALLOC(x) HeapAlloc(GetProcessHeap(), 0, (x))
#define FREE(x) HeapFree(GetProcessHeap(), 0, (x))

namespace coil
{
  // Winsock initializer
  class Winsock
  {
  public:
    Winsock() {
      WSADATA wsaData;
      int iResult;
      iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    }
    virtual ~Winsock()
    {
      WSACleanup();
    }
  };

  /*!
   * @if jp
   * @brief 宛先アドレスから利用されるエンドポイントアドレスを得る
   * @else
   * @brief Getting network interface name from destination address
   * @endif
   */
  bool dest_to_endpoint(std::string dest_addr, std::string& endpoint)
  {
    Winsock winsock;
    {
      struct hostent* hp;
      hp = ::gethostbyname(dest_addr.c_str());
      if (hp == 0) { return false; }

      int i(0);
      while (hp->h_addr_list[i] != 0)
        {
          if(hp->h_addrtype == AF_INET)
            {
              struct sockaddr_in addr;
              memset((char*)&addr, 0, sizeof(addr));
              memcpy((char*)&addr.sin_addr, hp->h_addr_list[i], hp->h_length);
              dest_addr = inet_ntoa(addr.sin_addr);
              break;
            }
          ++i;
        }
    }
    
    UINT ipaddress(inet_addr(dest_addr.c_str()));
    if (ipaddress == INADDR_NONE) { return false; }
    
    DWORD bestifindex;
    if (NO_ERROR != GetBestInterface(ipaddress, &bestifindex)) { return false; }
        
    PMIB_IPADDRTABLE ipaddr_table;
    ipaddr_table = (MIB_IPADDRTABLE *) MALLOC(sizeof (MIB_IPADDRTABLE));
    if (ipaddr_table == 0) { return false; }

    // Make an initial call to GetIpAddrTable to get the
    // necessary size into the size variable
    DWORD size(0);
    if (GetIpAddrTable(ipaddr_table, &size, 0) == ERROR_INSUFFICIENT_BUFFER)
      {
        FREE(ipaddr_table);
        ipaddr_table = (MIB_IPADDRTABLE *) MALLOC(size);
      }
    if (ipaddr_table == 0) { return false; }
    if (GetIpAddrTable(ipaddr_table, &size, 0) != NO_ERROR) { return false; }
    
    for (int i(0); i < (int) ipaddr_table->dwNumEntries; ++i)
      {
        if (bestifindex == ipaddr_table->table[i].dwIndex)
          {
            IN_ADDR inipaddr;
            inipaddr.S_un.S_addr = (u_long) ipaddr_table->table[i].dwAddr;
            endpoint = inet_ntoa(inipaddr);
            return true;
          }
      }
    return false;
  }
  
  
}; // namespace coil
