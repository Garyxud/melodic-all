// -*- C++ -*-
/*!
 * @file InPortBase.cpp
 * @brief Base class of InPort
 * @date $Date: 2007-12-31 03:08:02 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2008
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

#include <algorithm>
#include <iterator>

#include <rtm/CORBA_SeqUtil.h>
#include <rtm/NVUtil.h>
#include <rtm/InPortBase.h>
#include <rtm/CdrBufferBase.h>
#include <rtm/InPortProvider.h>
#include <rtm/OutPortConsumer.h>
#include <rtm/InPortPushConnector.h>
#include <rtm/InPortPullConnector.h>

namespace RTC
{

  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  InPortBase::InPortBase(const char* name, const char* data_type)
    : PortBase(name), m_singlebuffer(true), m_thebuffer(0), m_littleEndian(true)
  {
    RTC_DEBUG(("Port name: %s", name));

    // PortProfile::properties を設定
    RTC_DEBUG(("setting port.port_type: DataIntPort"));
    addProperty("port.port_type", "DataInPort");

    RTC_DEBUG(("setting dataport.data_type: %s", data_type));
    addProperty("dataport.data_type", data_type);

    addProperty("dataport.subscription_type", "Any");
  }
  
  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  InPortBase::~InPortBase()
  {
    RTC_TRACE(("~InPortBase()"));
    
    if (m_connectors.size() != 0)
      {
        RTC_ERROR(("connector.size should be 0 in InPortBase's dtor."));
        for (int i(0), len(m_connectors.size()); i < len; ++i)
          {
            m_connectors[i]->disconnect();
            delete m_connectors[i];
          }
      }
    
    if (m_thebuffer != 0)
      {
        CdrBufferFactory::instance().deleteObject(m_thebuffer);
        if (!m_singlebuffer)
          {
            RTC_ERROR(("Although singlebuffer flag is true, the buffer != 0"));
          }
      }
    
  }

  /*!
   * @if jp
   * @brief プロパティの初期化
   * @else
   * @brief Initializing properties
   * @endif
   */
  void InPortBase::init(coil::Properties& prop)
  {
    RTC_TRACE(("init()"));
    RTC_PARANOID(("given properties:"));
    RTC_DEBUG_STR((prop));

    // merge properties to PortProfile.properties
    m_properties << prop;
    NVList nv;
    NVUtil::copyFromProperties(nv, m_properties);
    CORBA_SeqUtil::push_back_list(m_profile.properties, nv);
    RTC_PARANOID(("updated properties:"));
    RTC_DEBUG_STR((m_properties));

    if (m_singlebuffer)
      {
        RTC_DEBUG(("single buffer mode."));
        m_thebuffer = CdrBufferFactory::instance().createObject("ring_buffer");
        if (m_thebuffer == 0)
          {
            RTC_ERROR(("default buffer creation failed"));
          }
      }
    else
      {
        RTC_DEBUG(("multi buffer mode."));
      }

    initProviders();
    initConsumers();
    int num(-1);
    if (!coil::stringTo(num, 
                     m_properties.getProperty("connection_limit","-1").c_str()))
      {
        RTC_ERROR(("invalid connection_limit value: %s", 
                   m_properties.getProperty("connection_limit").c_str()));
      }

    setConnectionLimit(num);
  }
  
  /*!
   * @if jp
   * @brief プロパティを取得する
   * @else
   * @brief Get properties
   * @endif
   */
  coil::Properties& InPortBase::properties()
  {
    RTC_TRACE(("properties()"));
    
    return m_properties;
  }

  /*!
   * @if jp
   * @brief Connector を取得
   * @else
   * @brief Connector list
   * @endif
   */
  const std::vector<InPortConnector*>& InPortBase::connectors()
  {
    RTC_TRACE(("connectors(): size = %d", m_connectors.size()));
    return m_connectors;
  }

  /*!
   * @if jp
   * @brief ConnectorProfile を取得
   * @else
   * @brief ConnectorProfile list
   * @endif
   */
  ConnectorInfoList InPortBase::getConnectorProfiles()
  {
    RTC_TRACE(("getConnectorProfiles(): size = %d", m_connectors.size()));
    ConnectorInfoList profs;
    for (int i(0), len(m_connectors.size()); i < len; ++i)
      {
        profs.push_back(m_connectors[i]->profile());
      }
    return profs;
  }

  /*!
   * @if jp
   * @brief ConnectorId を取得
   * @else
   * @brief ConnectorId list
   * @endif
   */
  coil::vstring InPortBase::getConnectorIds()
  {
    coil::vstring ids;
    for (int i(0), len(m_connectors.size()); i < len; ++i)
      {
        ids.push_back(m_connectors[i]->id());
      }
    RTC_TRACE(("getConnectorIds(): %s", coil::flatten(ids).c_str()));
    return ids;
  }

  /*!
   * @if jp
   * @brief Connectorの名前を取得
   * @else
   * @brief Connector name list
   * @endif
   */
  coil::vstring InPortBase::getConnectorNames()
  {
    coil::vstring names;
    for (int i(0), len(m_connectors.size()); i < len; ++i)
      {
        names.push_back(m_connectors[i]->name());
      }
    RTC_TRACE(("getConnectorNames(): %s", coil::flatten(names).c_str()));
    return names;
  }

  /*!
   * @if jp
   * @brief ConnectorをIDで取得
   * @else
   * @brief Getting Connector by ID
   * @endif
   */
  InPortConnector* InPortBase::getConnectorById(const char* id)
  {
    RTC_TRACE(("getConnectorById(id = %s)", id));

    std::string sid(id);
    for (int i(0), len(m_connectors.size()); i < len; ++i)
      {
        if (sid  == m_connectors[i]->id())
          {
            return m_connectors[i];
          }
      }
    RTC_WARN(("ConnectorProfile with the id(%s) not found.", id));
    return 0;
  }

  /*!
   * @if jp
   * @brief Connectorを名前で取得
   * @else
   * @brief Getting ConnectorProfile by name
   * @endif
   */
  InPortConnector* InPortBase::getConnectorByName(const char* name)
  {
    RTC_TRACE(("getConnectorByName(name = %s)", name));

    std::string sname(name);
    for (int i(0), len(m_connectors.size()); i < len; ++i)
      {
        if (sname  == m_connectors[i]->name())
          {
            return m_connectors[i];
          }
      }
    RTC_WARN(("ConnectorProfile with the name(%s) not found.", name));
    return 0;
  }

  /*!
   * @if jp
   * @brief ConnectorProfileをIDで取得
   * @else
   * @brief Getting ConnectorProfile by name
   * @endif
   */
  bool InPortBase::getConnectorProfileById(const char* id,
                                            ConnectorInfo& prof)
  {
    RTC_TRACE(("getConnectorProfileById(id = %s)", id));
    InPortConnector* conn(getConnectorById(id));
    if (conn == 0)
      {
        return false;
      }
    prof = conn->profile();
    return true;
  }

  /*!
   * @if jp
   * @brief ConnectorProfileを名前で取得
   * @else
   * @brief Getting ConnectorProfile by name
   * @endif
   */
  bool InPortBase::getConnectorProfileByName(const char* name,
                                              ConnectorInfo& prof)
  {
    RTC_TRACE(("getConnectorProfileByName(name = %s)", name));
    InPortConnector* conn(getConnectorByName(name));
    if (conn == 0)
      {
        return false;
      }
    prof = conn->profile();
    return true;
  }


  /*!
   * @if jp
   * @brief InPortを activates する
   * @else
   * @brief Activate all Port interfaces
   * @endif
   */
  void InPortBase::activateInterfaces()
  {
    RTC_TRACE(("activateInterfaces()"));
    
    for (int i(0), len(m_connectors.size()); i < len; ++i)
      {
        m_connectors[i]->activate();
        RTC_DEBUG(("activate connector: %s %s",
                   m_connectors[i]->name(),
                   m_connectors[i]->id()));
      }
  }

  /*!
   * @if jp
   * @brief 全ての Port のインターフェースを deactivates する
   * @else
   * @brief Deactivate all Port interfaces
   * @endif
   */
  void InPortBase::deactivateInterfaces()
  {
    RTC_TRACE(("deactivateInterfaces()"));

    for (int i(0), len(m_connectors.size()); i < len; ++i)
      {
        m_connectors[i]->deactivate();
        RTC_DEBUG(("deactivate connector: %s %s",
                   m_connectors[i]->name(),
                   m_connectors[i]->id()));
      }
  }

  /*!
   * @if jp
   * @brief ConnectorDataListener リスナを追加する
   *
   * バッファ書き込みまたは読み出しイベントに関連する各種リスナを設定する。
   *
   * @else
   * @brief Adding ConnectorDataListener type listener
   *
   * @endif
   */
  void InPortBase::
  addConnectorDataListener(ConnectorDataListenerType type,
                           ConnectorDataListener* listener,
                           bool autoclean)
  {
    if (type < CONNECTOR_DATA_LISTENER_NUM)
      {
        RTC_TRACE(("addConnectorDataListener(%s)",
                   ConnectorDataListener::toString(type)));
        m_listeners.connectorData_[type].addListener(listener, autoclean);
        return;
      }
    RTC_ERROR(("addConnectorDataListener(): Invalid listener type."));
    return;
  }

  void InPortBase::
  removeConnectorDataListener(ConnectorDataListenerType type,
                              ConnectorDataListener* listener)
  {
    if (type < CONNECTOR_DATA_LISTENER_NUM)
      {
        RTC_TRACE(("removeConnectorDataListener(%s)",
                   ConnectorDataListener::toString(type)));
        m_listeners.connectorData_[type].removeListener(listener);
        return;
      }
    RTC_ERROR(("removeConnectorDataListener(): Invalid listener type."));
    return;
  }
  
  /*!
   * @if jp
   * @brief ConnectorListener リスナを追加する
   *
   * @else
   * @brief Adding ConnectorListener type listener
   *
   * @endif
   */
  void InPortBase::addConnectorListener(ConnectorListenerType type,
                                         ConnectorListener* listener,
                                         bool autoclean)
  {
    if (type < CONNECTOR_LISTENER_NUM)
      {
        RTC_TRACE(("addConnectorListener(%s)",
                   ConnectorListener::toString(type)));
        m_listeners.connector_[type].addListener(listener, autoclean);
        return;
      }
    RTC_ERROR(("addConnectorListener(): Invalid listener type."));
    return;
  }
  
  void InPortBase::removeConnectorListener(ConnectorListenerType type,
                                            ConnectorListener* listener)
  {
    if (type < CONNECTOR_LISTENER_NUM)
      {
        RTC_TRACE(("removeConnectorListener(%s)",
                   ConnectorListener::toString(type)));
        m_listeners.connector_[type].removeListener(listener);
        return;
      }
    RTC_ERROR(("removeConnectorListener(): Invalid listener type."));
  }

  /*!
   * @if jp
   * @brief endian 設定がlittleか否か返す
   * @else
   * @brief return it whether endian setting is little
   * @endif
   */
  bool InPortBase::isLittleEndian()
  {
    return m_littleEndian;
  }

  /*!
   * @if jp
   * @brief [CORBA interface] Port の接続を行う
   * @else
   * @brief [CORBA interface] Connect the Port
   * @endif
   */
  ReturnCode_t InPortBase::connect(ConnectorProfile& connector_profile)
    throw (CORBA::SystemException)
  {
    RTC_TRACE(("InPortBase::connect()"));

    // endian infomation check
    CORBA::Long index(NVUtil::find_index(connector_profile.properties,
                                       "dataport.serializer.cdr.endian"));
    if (index < 0)
      {
        RTC_TRACE(("ConnectorProfile dataport.serializer.cdr.endian set."));
        // endian infomation set
        CORBA_SeqUtil::push_back(connector_profile.properties,
            NVUtil::newNV("dataport.serializer.cdr.endian", "little,big"));
      }
    return PortBase::connect(connector_profile);
  }
  
  //============================================================
  // protected interfaces
  //============================================================
  
  /*!
   * @if jp
   * @brief Interface情報を公開する
   * @else
   * @brief Publish interface information
   * @endif
   */
  ReturnCode_t InPortBase::publishInterfaces(ConnectorProfile& cprof)
  {    
    RTC_TRACE(("publishInterfaces()"));

    ReturnCode_t returnvalue = _publishInterfaces();
    if(returnvalue!=RTC::RTC_OK)
      {
        return returnvalue;
      }

    // prop: [port.outport].
    coil::Properties prop(m_properties);
    {
      coil::Properties conn_prop;
      NVUtil::copyToProperties(conn_prop, cprof.properties);
      prop << conn_prop.getNode("dataport"); // marge ConnectorProfile
      /*
       * marge ConnectorProfile for buffer property.
       * e.g.
       *  prof[buffer.write.full_policy]
       *       << cprof[dataport.inport.buffer.write.full_policy]
       */
      prop << conn_prop.getNode("dataport.inport");
    }
    RTC_DEBUG(("ConnectorProfile::properties are as follows."));
    RTC_DEBUG_STR((prop));

    /*
     * ここで, ConnectorProfile からの properties がマージされたため、
     * prop["dataflow_type"]: データフロータイプ
     * prop["interface_type"]: インターフェースタイプ
     * などがアクセス可能になる。
     */
    std::string dflow_type(prop["dataflow_type"]);
    coil::normalize(dflow_type);

    if (dflow_type == "push")
      {
        RTC_DEBUG(("dataflow_type = push .... create PushConnector"));

        // create InPortProvider
        InPortProvider* provider(createProvider(cprof, prop));
        if (provider == 0)
          {
            RTC_ERROR(("InPort provider creation failed."));
            return RTC::BAD_PARAMETER;
          }

        // create InPortPushConnector
        InPortConnector* connector(createConnector(cprof, prop, provider));
        if (connector == 0)
          {
            RTC_ERROR(("PushConnector creation failed."));
            return RTC::RTC_ERROR;
          }

        // connector set
        provider->setConnector(connector);

        RTC_DEBUG(("publishInterface() successfully finished."));
        return RTC::RTC_OK;
      }
    else if (dflow_type == "pull")
      {
        RTC_DEBUG(("dataflow_type = pull .... do nothing"));
        return RTC::RTC_OK;
      }

    RTC_ERROR(("unsupported dataflow_type: %s", dflow_type.c_str()));
    return RTC::BAD_PARAMETER;
  }
  
  /*!
   * @if jp
   * @brief Interfaceに接続する
   * @else
   * @brief Subscribe to the interface
   * @endif
   */
  ReturnCode_t InPortBase::subscribeInterfaces(const ConnectorProfile& cprof)
  {
    RTC_TRACE(("subscribeInterfaces()"));

    // prop: [port.outport].
    coil::Properties prop(m_properties);
    {
      coil::Properties conn_prop;
      NVUtil::copyToProperties(conn_prop, cprof.properties);
      prop << conn_prop.getNode("dataport"); // marge ConnectorProfile
      /*
       * marge ConnectorProfile for buffer property.
       * e.g.
       *  prof[buffer.write.full_policy]
       *       << cprof[dataport.inport.buffer.write.full_policy]
       */
      prop << conn_prop.getNode("dataport.inport");
    }
    RTC_DEBUG(("ConnectorProfile::properties are as follows."));
    RTC_DEBUG_STR((prop));

    bool littleEndian; // true: little, false: big
    if (!checkEndian(prop, littleEndian))
      {
        RTC_ERROR(("unsupported endian"));
        return RTC::UNSUPPORTED;
      }
    RTC_TRACE(("endian: %s", littleEndian ? "little" : "big"));

    /*
     * ここで, ConnectorProfile からの properties がマージされたため、
     * prop["dataflow_type"]: データフロータイプ
     * prop["interface_type"]: インターフェースタイプ
     * などがアクセス可能になる。
     */
    std::string dflow_type(prop["dataflow_type"]);
    coil::normalize(dflow_type);

    if (dflow_type == "push")
      {
        RTC_DEBUG(("dataflow_type is push."));

        // setting endian type
        InPortConnector* conn(getConnectorById(cprof.connector_id));
        if (conn == 0)
          {
            RTC_ERROR(("specified connector not found: %s",
                       (const char*)cprof.connector_id));
            return RTC::RTC_ERROR;
          }
        conn->setEndian(littleEndian);

        RTC_DEBUG(("subscribeInterfaces() successfully finished."));
        return RTC::RTC_OK;
      }
    else if (dflow_type == "pull")
      {
        RTC_DEBUG(("dataflow_type is pull."));

        // create OutPortConsumer
        OutPortConsumer* consumer(createConsumer(cprof, prop));
        if (consumer == 0)
          {
            return RTC::BAD_PARAMETER;
          }

        // create InPortPullConnector
        InPortConnector* connector(createConnector(cprof, prop, consumer));
        if (connector == 0)
          {
            return RTC::RTC_ERROR;
          }

        RTC_DEBUG(("subscribeInterfaces() successfully finished."));
        return RTC::RTC_OK;
      }

    RTC_ERROR(("unsupported dataflow_type: %s", dflow_type.c_str()));
    return RTC::BAD_PARAMETER;
  }
  
  /*!
   * @if jp
   * @brief Interfaceへの接続を解除する
   * @else
   * @brief Disconnect the interface connection
   * @endif
   */
  void
  InPortBase::unsubscribeInterfaces(const ConnectorProfile& connector_profile)
  {
    RTC_TRACE(("unsubscribeInterfaces()"));

    std::string id(connector_profile.connector_id);
    RTC_PARANOID(("connector_id: %s", id.c_str()));

    ConnectorList::iterator it(m_connectors.begin());

    while (it != m_connectors.end())
      {
        if (id == (*it)->id())
          {
            // Connector's dtor must call disconnect()
// RtORB's bug? This causes double delete and segmeentation fault.
#ifndef ORB_IS_RTORB
            delete *it;
#endif
            m_connectors.erase(it);
            RTC_TRACE(("delete connector: %s", id.c_str()));
            return;
          }
        ++it;
      }
    RTC_ERROR(("specified connector not found: %s", id.c_str()));
    return;
  }

  /*!
   * @if jp
   * @brief InPort provider の初期化
   * @else
   * @brief InPort provider initialization
   * @endif
   */
  void InPortBase::initProviders()
  {
    RTC_TRACE(("initProviders()"));

    // create InPort providers
    InPortProviderFactory& factory(InPortProviderFactory::instance());
    coil::vstring provider_types(factory.getIdentifiers());
    RTC_DEBUG(("available providers: %s",
               coil::flatten(provider_types).c_str()));

#ifndef RTC_NO_DATAPORTIF_ACTIVATION_OPTION
    if (m_properties.hasKey("provider_types") &&
        coil::normalize(m_properties["provider_types"]) != "all")
      {
        RTC_DEBUG(("allowed providers: %s",
                   m_properties["provider_types"].c_str()));

        coil::vstring temp_types(provider_types);
        provider_types.clear();
        coil::vstring
          active_types(coil::split(m_properties["provider_types"], ","));

        std::sort(temp_types.begin(), temp_types.end());
        std::sort(active_types.begin(), active_types.end());
        std::set_intersection(temp_types.begin(), temp_types.end(),
                              active_types.begin(), active_types.end(),
                              std::back_inserter(provider_types));
      }
#endif
    
    // InPortProvider supports "push" dataflow type
    if (provider_types.size() > 0)
      {
        RTC_DEBUG(("dataflow_type push is supported"));
        appendProperty("dataport.dataflow_type", "push");
        appendProperty("dataport.interface_type",
                       coil::flatten(provider_types).c_str());
      }

    m_providerTypes = provider_types;
  }

  /*!
   * @if jp
   * @brief OutPort consumer の初期化
   * @else
   * @brief OutPort consumer initialization
   * @endif
   */
  void InPortBase::initConsumers()
  {
    RTC_TRACE(("initConsumers()"));

    // create OuPort consumers
    OutPortConsumerFactory& factory(OutPortConsumerFactory::instance());
    coil::vstring consumer_types(factory.getIdentifiers());
    RTC_DEBUG(("available consumers: %s",
               coil::flatten(consumer_types).c_str()));

#ifndef RTC_NO_DATAPORTIF_ACTIVATION_OPTION
    if (m_properties.hasKey("consumer_types") &&
        coil::normalize(m_properties["consumer_types"]) != "all")
      {
        RTC_DEBUG(("allowed consumers: %s",
                   m_properties["consumer_types"].c_str()));

        coil::vstring temp_types(consumer_types);
        consumer_types.clear();
        coil::vstring
          active_types(coil::split(m_properties["consumer_types"], ","));

        std::sort(temp_types.begin(), temp_types.end());
        std::sort(active_types.begin(), active_types.end());
        std::set_intersection(temp_types.begin(), temp_types.end(),
                              active_types.begin(), active_types.end(),
                              std::back_inserter(consumer_types));
      }
#endif

    // OutPortConsumer supports "pull" dataflow type
    if (consumer_types.size() > 0)
      {
        RTC_PARANOID(("dataflow_type pull is supported"));
        appendProperty("dataport.dataflow_type", "pull");
        appendProperty("dataport.interface_type",
                       coil::flatten(consumer_types).c_str());
      }

    m_consumerTypes = consumer_types;
  }

  /*!
   * @if jp
   * @brief シリアライザのエンディアンをチェックする
   * @else
   * @brief Checking endian flag of serializer
   * @endif
   */
  bool InPortBase::checkEndian(const coil::Properties& prop,
                               bool& littleEndian)
  {
        // old version check
    if(prop.hasKey("serializer") == NULL)
      {
        littleEndian = true;
        return true;
      }

    // endian type check
    std::string endian_type(prop.getProperty("serializer.cdr.endian", ""));
    RTC_DEBUG(("endian_type: %s", endian_type.c_str()));
    coil::normalize(endian_type);
    std::vector<std::string> endian(coil::split(endian_type, ","));

    if(endian.empty()) { return false; }
    if(endian[0] == "little")
      {
        littleEndian = true;
        return true;
      }
    else if(endian[0] == "big")
      {
        littleEndian = false;
        return true;
      }
    return false;
  }



  /*!
   * @if jp
   * @brief InPort provider の生成
   * @else
   * @brief InPort provider creation
   * @endif
   */
  InPortProvider*
  InPortBase::createProvider(ConnectorProfile& cprof, coil::Properties& prop)
  {
    if (!prop["interface_type"].empty() &&
        !coil::includes((coil::vstring)m_providerTypes, prop["interface_type"]))
      {
        RTC_ERROR(("no provider found"));
        RTC_DEBUG(("interface_type:  %s", prop["interface_type"].c_str()));
        RTC_DEBUG(("interface_types: %s",
                   coil::flatten(m_providerTypes).c_str()));
        return 0;
      }
    
    RTC_DEBUG(("interface_type: %s", prop["interface_type"].c_str()));
    InPortProvider* provider;
    provider = InPortProviderFactory::
      instance().createObject(prop["interface_type"].c_str());
    
    if (provider != 0)
      {
        RTC_DEBUG(("provider created"));
        provider->init(prop.getNode("provider"));

#ifndef ORB_IS_RTORB
        if (!provider->publishInterface(cprof.properties))
          {
            RTC_ERROR(("publishing interface information error"));
            InPortProviderFactory::instance().deleteObject(provider);
            return 0;
          }
#else // ORB_IS_RTORB
        // RtORB's copy ctor's bug?
        ::SDOPackage::NVList_ptr prop_ref(cprof.properties);
        if (!provider->publishInterface(*prop_ref))
          {
            RTC_ERROR(("publishing interface information error"));
            InPortProviderFactory::instance().deleteObject(provider);
            return 0;
          }
#endif // ORB_IS_RTORB
        return provider;
      }

    RTC_ERROR(("provider creation failed"));
    return 0;
  }

  /*!
   * @if jp
   * @brief OutPort consumer の生成
   * @else
   * @brief InPort provider creation
   * @endif
   */
  OutPortConsumer*
  InPortBase::createConsumer(const ConnectorProfile& cprof,
                             coil::Properties& prop)
  {
    if (!prop["interface_type"].empty() &&
        !coil::includes((coil::vstring)m_consumerTypes, prop["interface_type"]))
      {
        RTC_ERROR(("no consumer found"));
        RTC_DEBUG(("interface_type:  %s", prop["interface_type"].c_str()));
        RTC_DEBUG(("interface_types: %s",
                   coil::flatten(m_consumerTypes).c_str()));
        return 0;
      }
    
    RTC_DEBUG(("interface_type: %s", prop["interface_type"].c_str()));
    OutPortConsumer* consumer;
    consumer = OutPortConsumerFactory::
      instance().createObject(prop["interface_type"].c_str());
    
    if (consumer != 0)
      {
        RTC_DEBUG(("consumer created"));
        consumer->init(prop.getNode("consumer"));

        if (!consumer->subscribeInterface(cprof.properties))
          {
            RTC_ERROR(("interface subscription failed."));
            OutPortConsumerFactory::instance().deleteObject(consumer);
            return 0;
          }

        return consumer;
      }

    RTC_ERROR(("consumer creation failed"));
    return 0; 
  }
  
  /*!
   * @if jp
   * @brief InPortPushConnector の生成
   * @else
   * @brief InPortPushConnector creation
   * @endif
   */
  InPortConnector*
  InPortBase::createConnector(ConnectorProfile& cprof, coil::Properties& prop,
                              InPortProvider* provider)
  {
#ifndef ORB_IS_RTORB
    ConnectorInfo profile(cprof.name,
                          cprof.connector_id,
                          CORBA_SeqUtil::refToVstring(cprof.ports),
                          prop); 
#else // ORB_IS_RTORB
    ConnectorInfo profile(cprof.name,
                          cprof.connector_id,
                          CORBA_SeqUtil::
                          refToVstring(RTC::PortServiceList(cprof.ports)),
                          prop); 
#endif // ORB_IS_RTORB

    InPortConnector* connector(0);
    try
      {
        if (m_singlebuffer)
          {
            connector = new InPortPushConnector(profile, provider,
                                                m_listeners,
                                                m_thebuffer);
          }
        else
          {
            connector = new InPortPushConnector(profile, 
                                                provider,
                                                m_listeners);
          }

        if (connector == 0)
          {
            RTC_ERROR(("old compiler? new returned 0;"));
            return 0;
          }
        RTC_TRACE(("InPortPushConnector created"));

        m_connectors.push_back(connector);
        RTC_PARANOID(("connector push backed: %d", m_connectors.size()));
        return connector;
      }
    catch (std::bad_alloc& e)
      {
        RTC_ERROR(("InPortPushConnector creation failed"));
        return 0;
      }
    RTC_FATAL(("never comes here: createConnector()"));
    return 0;
  }

  /*!
   * @if jp
   * @brief InPortPullConnector の生成
   * @else
   * @brief InPortPullConnector creation
   * @endif
   */
  InPortConnector*
  InPortBase::createConnector(const ConnectorProfile& cprof,
                              coil::Properties& prop,
                              OutPortConsumer* consumer)
  {
#ifndef ORB_IS_RTORB
    ConnectorInfo profile(cprof.name,
                          cprof.connector_id,
                          CORBA_SeqUtil::refToVstring(cprof.ports),
                          prop); 
#else // ORB_IS_RTORB
    ConnectorInfo profile(cprof.name,
                          cprof.connector_id,
                          CORBA_SeqUtil::
                          refToVstring(RTC::PortServiceList(cprof.ports)),
                          prop); 
#endif // ORB_IS_RTORB

    InPortConnector* connector(0);
    try
      {
        if (m_singlebuffer)
          {
            connector = new InPortPullConnector(profile, consumer,
                                                m_listeners,
                                                m_thebuffer);
          }
        else
          {
            connector = new InPortPullConnector(profile, consumer, m_listeners);
          }

        if (connector == 0)
          {
            RTC_ERROR(("old compiler? new returned 0;"));
            return 0;
          }
        RTC_TRACE(("InPortPushConnector created"));

        // endian type set
        connector->setEndian(m_littleEndian);
        m_connectors.push_back(connector);
        RTC_PARANOID(("connector push backed: %d", m_connectors.size()));
        return connector;
      }
    catch (std::bad_alloc& e)
      {
        RTC_ERROR(("InPortPullConnector creation failed"));
        return 0;
      }
    RTC_FATAL(("never comes here: createConnector()"));
    return 0;
  }

};
