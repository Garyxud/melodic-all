// -*- C++ -*-
/*!
 * @file  IRTObject.h
 * @brief IRTObject interface class
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

#ifndef RTC_LOCAL_IRTOBJECT_H
#define RTC_LOCAL_IRTOBJECT_H

#include <vector>
#include <rtc/IRTC.h>
#include <rtc/ILightweightRTObject.h>

namespace RTC
{
namespace Local
{
  class PortProfile;
  typedef std::vector<PortProfile*> PortProfileList;
  
  class IPortService;
  typedef std::vector<IPortService*> PortServiceList;
  
  class IRTObject;
  
  struct ComponentProfile
  {
    char* instance_name;
    char* type_name;
    char* description;
    char* version;
    char* vendor;
    char* category;
    PortProfileList port_profiles;
    IRTObject* parent;
    NVList properties;
  };
  
  /*!
   * @if jp
   * @class IRTObject
   * @brief IRTObject インターフェースクラス
   * @else
   * @class IRTObject
   * @brief IRTObject interface class
   * @endif
   */
  class IRTObject
    : public virtual ILightweightRTObject
  //        public SDOPackage::SDO
  {
  public:
    virtual ~IRTObject() {};
    virtual const ComponentProfile& get_component_profile() = 0;
    virtual PortServiceList& get_ports() = 0;
  };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IRTOBJECT_H
