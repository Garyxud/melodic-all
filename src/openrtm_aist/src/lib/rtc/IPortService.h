// -*- C++ -*-
/*!
 * @file  IPortService.h
 * @brief IPortService interface class
 * @date  $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
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

#ifndef RTC_LOCAL_IPORTSERVICE_H
#define RTC_LOCAL_IPORTSERVICE_H

#include <vector>
#include <rtc/IRTC.h>

namespace RTC
{
namespace Local
{
  
  struct PortInterfaceProfile
  {
    char* instance_name;
    char* type_name;
    PortPolarity polarity;
  };
  
  typedef std::vector<PortInterfaceProfile*> PortInterfaceProfileList;
  class IPortService;
  typedef std::vector<IPortService*> PortServiceList;
  
  struct ConnectorProfile
  {
    char* name;
    UniqueIdentifier connector_id;
    PortServiceList ports;
    NVList properties;
  };
  
  typedef std::vector<ConnectorProfile*> ConnectorProfileList;
  class RTObject;
  
  struct PortProfile
  {
    char* name;
    PortInterfaceProfileList interfaces;
    IPortService* port_ref;
    ConnectorProfileList connector_profiles;
    RTObject* owner;
    NVList properties;
  };
  
  typedef std::vector<PortProfile*> PortProfileList;
  
  /*!
   * @if jp
   * @class IPortService
   * @brief IPortService インターフェースクラス
   * @else
   * @class IPortService
   * @brief IPortService itnerface class
   * @endif
   */
  class IPortService
  {
  public:
    virtual ~IPortService() {};
    virtual PortProfile& get_port_profile() const = 0;
    virtual ConnectorProfileList& get_connector_profiles() const = 0;
    virtual ConnectorProfile&
    get_connector_profile(const UniqueIdentifier connector_id) const = 0;
    virtual ReturnCode_t connect(ConnectorProfile& connector_profile) = 0;
    virtual ReturnCode_t disconnect(const UniqueIdentifier connector_id) = 0;
    virtual ReturnCode_t
    notify_connect(ConnectorProfile& connector_profile) = 0;
    virtual ReturnCode_t
    notify_disconnect(const UniqueIdentifier connector_id) = 0;
    virtual ReturnCode_t disconnect_all() = 0;
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IPORTSERVICE_H

