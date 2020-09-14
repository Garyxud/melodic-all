// -*- C++ -*-
/*!
 * @file OutPortPullConnector.cpp
 * @brief OutPortPull type connector class
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2009
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

#include <rtm/OutPortPullConnector.h>
#include <rtm/OutPortProvider.h>

namespace RTC
{
  /*!
   * @if jp
   * @brief コンストラクタ
   * @elsek
   * @brief Constructor
   * @endif
   */
  OutPortPullConnector::OutPortPullConnector(ConnectorInfo info,
                                             OutPortProvider* provider,
                                             ConnectorListeners& listeners,
                                             CdrBufferBase* buffer)
    : OutPortConnector(info),
      m_provider(provider),
      m_listeners(listeners),
      m_buffer(buffer)
  {
    // create buffer
    if (m_buffer == 0)
      {
        m_buffer = createBuffer(info);
      }

    if (m_provider == 0 || m_buffer == 0) { throw std::bad_alloc(); }

    m_buffer->init(info.properties.getNode("buffer"));
    m_provider->setBuffer(m_buffer);
    m_provider->setConnector(this);
    //    m_provider->init(m_profile /* , m_listeners */);
    m_provider->setListener(info, &m_listeners);

    onConnect();
  }

  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  OutPortPullConnector::~OutPortPullConnector()
  {
    onDisconnect();
    disconnect();
  }

  /*!
   * @if jp
   * @brief read 関数
   * @else
   * @brief Destructor
   * @endif
   */
  ConnectorBase::ReturnCode
  OutPortPullConnector::write(const cdrMemoryStream& data)
  {
    m_buffer->write(data);
    return PORT_OK;
  }

  /*!
   * @if jp
   * @brief 接続解除関数
   * @else
   * @brief Disconnect connection
   * @endif
   */
  ConnectorBase::ReturnCode OutPortPullConnector::disconnect()
  {
    RTC_TRACE(("disconnect()"));
    // delete provider
    if (m_provider != 0)
      {
        OutPortProviderFactory& cfactory(OutPortProviderFactory::instance());
        cfactory.deleteObject(m_provider);
      }
    m_provider = 0;

    // delete buffer
    if (m_buffer != 0)
      {
        CdrBufferFactory& bfactory(CdrBufferFactory::instance());
        bfactory.deleteObject(m_buffer);
      }
    m_buffer = 0;

    return PORT_OK;
  }

  /*!
   * @if jp
   * @brief Buffer を取得する
   *
   * Connector が保持している Buffer を返す
   *
   * @else
   * @brief Getting Buffer
   *
   * This operation returns this connector's buffer
   *
   * @endif
   */
  CdrBufferBase* OutPortPullConnector::getBuffer()
  {
    return m_buffer;
  }
  
  /*!
   * @if jp
   * @brief Bufferの生成
   * @else
   * @brief create buffer
   * @endif
   */
  CdrBufferBase* OutPortPullConnector::createBuffer(ConnectorInfo& info)
  {
    std::string buf_type;
    buf_type = info.properties.getProperty("buffer_type",
                                              "ring_buffer");
    return CdrBufferFactory::instance().createObject(buf_type);
  }

  /*!
   * @if jp
   * @brief 接続確立時にコールバックを呼ぶ
   * @else
   * @brief Invoke callback when connection is established
   * @endif
   */
  void OutPortPullConnector::onConnect()
  {
    m_listeners.connector_[ON_CONNECT].notify(m_profile);
  }

  /*!
   * @if jp
   * @brief 接続切断時にコールバックを呼ぶ
   * @else
   * @brief Invoke callback when connection is destroied
   * @endif
   */
  void OutPortPullConnector::onDisconnect()
  {
    m_listeners.connector_[ON_DISCONNECT].notify(m_profile);
  }
};

