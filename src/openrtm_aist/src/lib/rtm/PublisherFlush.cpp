// -*- C++ -*-
/*!
 * @file  PublisherFlush.cpp
 * @brief PublisherFlush class
 * @date  $Date: 2007-12-31 03:08:06 $
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

#include <coil/Properties.h>
#include <rtm/RTC.h>
#include <rtm/PublisherBase.h>
#include <rtm/PublisherFlush.h>
#include <rtm/InPortConsumer.h>
#include <rtm/ConnectorListener.h>

namespace RTC
{
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  PublisherFlush::PublisherFlush()
    : rtclog("PublisherFlush"),
      m_consumer(0), m_listeners(0), m_active(false),
      m_retcode(PORT_OK)
  {
  }

  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  PublisherFlush::~PublisherFlush()
  {
    RTC_TRACE(("~PublisherFlush()"));
    // "consumer" should be deleted in the Connector
    m_consumer = 0;
  }
  
  /*!
   * @if jp
   * @brief 初期化
   * @else
   * @brief initialization
   * @endif
   */
  PublisherBase::ReturnCode PublisherFlush::init(coil::Properties& prop)
  {
    RTC_TRACE(("init()"));
    return PORT_OK;
  }

  /*!
   * @if jp
   * @brief InPortコンシューマのセット
   * @else
   * @brief Store InPort consumer
   * @endif
   */
  PublisherBase::ReturnCode
  PublisherFlush::setConsumer(InPortConsumer* consumer)
  {
    RTC_TRACE(("setConsumer()"));
    
    if (consumer == 0)
      {
        return INVALID_ARGS;
      }
    m_consumer = consumer;
    return PORT_OK;
  }

  /*!
   * @if jp
   * @brief バッファのセット
   * @else
   * @brief Setting buffer pointer
   * @endif
   */
  PublisherBase::ReturnCode PublisherFlush::setBuffer(CdrBufferBase* buffer)
  {
    RTC_TRACE(("setBuffer()"));

    return PORT_OK;
  }

  /*!
   * @if jp
   * @brief リスナのセット
   * @else
   * @brief Setting buffer pointer
   * @endif
   */ 
  ::RTC::DataPortStatus::Enum
  PublisherFlush::setListener(ConnectorInfo& info,
                              RTC::ConnectorListeners* listeners)
  {
    RTC_TRACE(("setListeners()"));

    if (listeners == 0)
      {
        RTC_ERROR(("setListeners(listeners == 0): invalid argument"));
        return INVALID_ARGS;
      }

    m_profile = info;
    m_listeners = listeners;

    return PORT_OK;
  }

  /*!
   * @if jp
   * @brief データを書き込む
   * @else
   * @brief Write data 
   * @endif
   */
  PublisherBase::ReturnCode PublisherFlush::write(const cdrMemoryStream& data,
                                                  unsigned long sec,
                                                  unsigned long usec)
  {
    RTC_PARANOID(("write()"));

    if (m_consumer == 0) { return PRECONDITION_NOT_MET; }
    if (m_listeners == 0) { return PRECONDITION_NOT_MET; }

    if (m_retcode == CONNECTION_LOST)
      {
        RTC_DEBUG(("write(): connection lost."));
        return m_retcode;
      }

    onSend(data);
    ReturnCode ret(m_consumer->put(data));
    // consumer::put() returns
    //  {PORT_OK, PORT_ERROR, SEND_FULL, SEND_TIMEOUT, UNKNOWN_ERROR}

    switch (ret)
      {
      case PORT_OK:
        onReceived(data);
        return ret;
      case PORT_ERROR:
        onReceiverError(data);
        return ret;
      case SEND_FULL:
        onReceiverFull(data);
        return ret;
      case SEND_TIMEOUT:
        onReceiverTimeout(data);
        return ret;
      case CONNECTION_LOST:
        onReceiverTimeout(data);
        return ret;
      case UNKNOWN_ERROR:
        onReceiverError(data);
        return ret;
      default:
        onReceiverError(data);
        return ret;
      }
    return ret;
  }

  /*!
   * @if jp
   * @brief アクティブ化確認
   * @else
   * @brief Confirm to activate
   * @endif
   */
  bool PublisherFlush::isActive()
  {
    return m_active;
  }

  /*!
   * @if jp
   * @brief アクティブ化
   * @else
   * @brief activation
   * @endif
   */
  PublisherBase::ReturnCode PublisherFlush::activate()
  {
    m_active = true;
    return PORT_OK;
  }

  /*!
   * @if jp
   * @brief 非アクティブ化
   * @else
   * @brief deactivation
   * @endif
   */
  PublisherBase::ReturnCode PublisherFlush::deactivate()
  {
    m_active = false;
    return PORT_OK;
  }

}; // namespace RTC


extern "C"
{
  void PublisherFlushInit()
  {
    ::RTC::PublisherFactory::
      instance().addFactory("flush",
                            ::coil::Creator< ::RTC::PublisherBase,
                                             ::RTC::PublisherFlush>,
                            ::coil::Destructor< ::RTC::PublisherBase,
                                                ::RTC::PublisherFlush>);
  }
};

