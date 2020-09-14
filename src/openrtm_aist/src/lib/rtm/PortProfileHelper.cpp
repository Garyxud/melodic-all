// -*- C++ -*-
/*!
 * @file PortProfileHelper.cpp
 * @brief RTC's PortProfile helper class
 * @date $Date: 2006-11-27 09:48:13 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#include "rtm/PortProfileHelper.h"

namespace RTC
{
  PortProfileHelper::PortProfileHelper()
  {
    ;
  }


  PortProfileHelper::~PortProfileHelper()
  {
    ;
  }


  /*!
   * @if jp
   * @brief PortProfile を設定する
   * @else
   * @brief Set PortProfile
   * @endif
   */
  void PortProfileHelper::setPortProfile(const PortProfile& profile)
  {
    Guard guard(m_mutex);

    m_name         = profile.name;
    m_ifProfiles   = profile.interfaces;
    m_portRef      = RTC::PortService::_duplicate(profile.port_ref);
    m_connProfiles = profile.connector_profiles;
    m_owner        = RTC::RTObject::_duplicate(profile.owner);
    m_properties   = profile.properties;

    return;
  }


  /*!
   * @if jp
   * @brief PortProfile を取得する
   * @else
   * @brief Get PortProfile
   * @endif
   */
  PortProfile* PortProfileHelper::getPortProfile()
  {
    Guard guard(m_mutex);

    PortProfile_var profile(new PortProfile());
    profile->name               = CORBA::string_dup(m_name.c_str());
    profile->interfaces         = m_ifProfiles;
    profile->port_ref           = m_portRef;
    profile->connector_profiles = m_connProfiles;
    profile->owner              = m_owner;
    profile->properties         = m_properties;

    return profile._retn();
  }


  /*!
   * @if jp
   * @brief PortProfile.name を設定する
   * @else
   * @brief Set PortProfile.name
   * @endif
   */
  void PortProfileHelper::setName(const char* name)
  {
    Guard guard(m_mutex);

    m_name = name;
  }


  /*!
   * @if jp
   * @brief PortProfile.name を取得する
   * @else
   * @brief Get PortProfile.name
   * @endif
   */
  const char* PortProfileHelper::getName() const
  {
    Guard guard(m_mutex);

    return m_name.c_str();
  }
  

  /*!
   * @if jp
   * @brief PortInterfaceProfile を追加する
   * @else
   * @brief Append PortInterfaceProfile to the PortProfile
   * @endif
   */
  void
  PortProfileHelper::appendPortInterfaceProfile(PortInterfaceProfile if_prof)
  {
    Guard guard(m_mutex);

    m_ifProfiles.push_back(if_prof);
    return;
  }


  /*!
   * @if jp
   * @brief PortInterfaceProfileList を取得する
   * @else
   * @brief Get PortInterfaceProfileList
   * @endif
   */
  const PortInterfaceProfileList&
  PortProfileHelper::getPortInterfaceProfiles() const
  {
    Guard guard(m_mutex);

    return m_ifProfiles;
  }


  /*!
   * @if jp
   * @brief PortInterfaceProfile を取得する
   * @else
   * @brief Get PortInterfaceProfile
   * @endif
   */
  const PortInterfaceProfile
  PortProfileHelper::getPortInterfaceProfile(const char* instance_name) const
  {
    Guard guard(m_mutex);

    return m_ifProfiles.find(if_name(instance_name));
  }


  /*!
   * @if jp
   * @brief PortInterfaceProfile を削除する
   * @else
   * @brief Erase PortInterfaceProfile from the PortProfile
   * @endif
   */
  void PortProfileHelper::erasePortInterfaceProfile(const char* instance_name)
  {
    Guard guard(m_mutex);

    m_ifProfiles.erase_if(if_name(instance_name));
    return;
  }


  /*!
   * @if jp
   * @brief Port のオブジェクト参照をセットする
   * @else
   * @brief Set Port's object reference
   * @endif
   */
  void PortProfileHelper::setPortRef(PortService_ptr port)
  {
    Guard guard(m_mutex);

    m_portRef = RTC::PortService::_duplicate(port);
  }


  /*!
   * @if jp
   * @brief Port のオブジェクト参照を取得する
   * @else
   * @brief Get Port's object reference
   * @endif
   */
  PortService_ptr PortProfileHelper::getPortRef() const
  {
    Guard guard(m_mutex);

    return m_portRef;
  }


  /*!
   * @if jp
   * @brief ConnectorProfile を追加する
   * @else
   * @brief Append ConnectorProfile
   * @endif
   */
  void PortProfileHelper::appendConnectorProfile(ConnectorProfile conn_profile)
  {
    Guard guard(m_mutex);

    m_connProfiles.push_back(conn_profile);
  }


  /*!
   * @if jp
   * @brief ConnectorProfileList を取得する
   * @else
   * @brief Get ConnectorProfileList
   * @endif
   */
  const ConnectorProfileList PortProfileHelper::getConnectorProfiles() const
  {
    Guard guard(m_mutex);

    return m_connProfiles;
  }



  /*!
   * @if jp
   * @brief ConnectorProfile を取得する
   * @else
   * @brief Get ConnectorProfile
   * @endif
   */
  const ConnectorProfile
  PortProfileHelper::getConnectorProfile(const char* name) const
  {
    Guard guard(m_mutex);

    return m_connProfiles.find(conn_name(name));
  }


  /*!
   * @if jp
   * @brief ConnectorProfile を取得する
   * @else
   * @brief Get ConnectorProfile
   * @endif
   */
  const ConnectorProfile
  PortProfileHelper::getConnectorProfileById(const char* id) const
  {
    Guard guard(m_mutex);

    return m_connProfiles.find(conn_id(id));
  }


  /*!
   * @if jp
   * @brief ConnectorProfile を削除する
   * @param naem ConnectorProfile の名前
   * @else
   * @brief Erase ConnectorProfile
   * @endif
   */
  void PortProfileHelper::eraseConnectorProfile(const char* name)
  {
    Guard guard(m_mutex);

    m_connProfiles.erase_if(conn_name(name));
    return;
  }


  /*!
   * @if jp
   * @brief ConnectorProfile を削除する
   * @else
   * @brief Erase ConnectorProfile
   * @endif
   */
  void PortProfileHelper::eraseConnectorProfileById(const char* id)
  {
    Guard guard(m_mutex);

    m_connProfiles.erase_if(conn_id(id));
    return;
  }


  /*!
   * @if jp
   * @brief PortProfile の owner を設定する
   * @else
   * @brief Set owner's object reference to the PortProfile
   * @endif
   */
  void PortProfileHelper::setOwner(RTObject_ptr owner)
  {
    Guard guard(m_mutex);

    m_owner = RTC::RTObject::_duplicate(owner);
  }


  /*!
   * @if jp
   * @brief PortProfile の owner を取得する
   * @else
   * @endif
   */
  RTObject_ptr PortProfileHelper::getOwner() const
  {
    Guard guard(m_mutex);

    return m_owner;
  }


  /*!
   * @if jp
   * @brief PortProfile の properties を設定する
   * @else
   * @brief Set properties to the PortProfile
   * @endif
   */
  void PortProfileHelper::setProperties(NVList& prop)
  {
    Guard guard(m_mutex);

    m_properties = prop;
  }


  /*!
   * @if jp
   * @brief PortProfile の properties を取得する
   * @else
   * @brief Get properties of the PortProfile
   * @endif
   */
  const NVList& PortProfileHelper::getProperties() const
  {
    Guard guard(m_mutex);

    return m_properties;
  }

};
