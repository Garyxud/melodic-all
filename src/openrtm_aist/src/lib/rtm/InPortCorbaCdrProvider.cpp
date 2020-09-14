// -*- C++ -*-
/*!
 * @file  InPortCorbaCdrProvider.cpp
 * @brief InPortCorbaCdrProvider class
 * @date  $Date: 2008-01-14 07:49:59 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2009-2010
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id: InPortCorbaCdrProvider.cpp 1244 2009-03-13 07:25:42Z n-ando $
 *
 */

#include <rtm/InPortCorbaCdrProvider.h>

#ifdef WIN32
#pragma warning( disable : 4290 )
#endif

namespace RTC
{
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  InPortCorbaCdrProvider::InPortCorbaCdrProvider(void)
   : m_buffer(0) 
  {
    // PortProfile setting
    setInterfaceType("corba_cdr");
    
    // ConnectorProfile setting
    m_objref = this->_this();
    
    // set InPort's reference
    CORBA::ORB_ptr orb = ::RTC::Manager::instance().getORB();
    CORBA::String_var ior = orb->object_to_string(m_objref.in());
    CORBA_SeqUtil::
      push_back(m_properties,
                NVUtil::newNV("dataport.corba_cdr.inport_ior", ior.in()));
    CORBA_SeqUtil::
      push_back(m_properties,
                NVUtil::newNV("dataport.corba_cdr.inport_ref", m_objref));
  }
  
  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  InPortCorbaCdrProvider::~InPortCorbaCdrProvider(void)
  {
    try
      {
        PortableServer::ObjectId_var oid;
        oid = _default_POA()->servant_to_id(this);
        _default_POA()->deactivate_object(oid);
      }
    catch (PortableServer::POA::ServantNotActive &e)
      {
        RTC_ERROR(("%s", e._name()));
      }
    catch (PortableServer::POA::WrongPolicy &e)
      {
        RTC_ERROR(("%s", e._name()));
      }
    catch (...)
      {
        // never throws exception
        RTC_ERROR(("Unknown exception caught."));
      }
  }

  void InPortCorbaCdrProvider::init(coil::Properties& prop)
  {
  }

  /*!
   * @if jp
   * @brief バッファをセットする
   * @else
   * @brief Setting outside buffer's pointer
   * @endif
   */
  void InPortCorbaCdrProvider::
  setBuffer(BufferBase<cdrMemoryStream>* buffer)
  {
    m_buffer = buffer;
  }

  /*!
   * @if jp
   * @brief リスナを設定する
   * @else
   * @brief Set the listener
   * @endif
   */
  void InPortCorbaCdrProvider::setListener(ConnectorInfo& info,
                                           ConnectorListeners* listeners)
  {
    m_profile = info;
    m_listeners = listeners;
  }

  /*!
   * @if jp
   * @brief Connectorを設定する。
   * @else
   * @brief set Connector
   * @endif
   */
  void InPortCorbaCdrProvider::setConnector(InPortConnector* connector)
  {
    m_connector = connector;
  }

  /*!
   * @if jp
   * @brief バッファにデータを書き込む
   * @else
   * @brief Write data into the buffer
   * @endif
   */
  ::OpenRTM::PortStatus
  InPortCorbaCdrProvider::put(const ::OpenRTM::CdrData& data)
    throw (CORBA::SystemException)
  {
    RTC_PARANOID(("InPortCorbaCdrProvider::put()"));

    if (m_buffer == 0)
      {
        cdrMemoryStream cdr;
        cdr.put_octet_array(&(data[0]), data.length());
        onReceiverError(cdr);
        return ::OpenRTM::PORT_ERROR;
      }

    RTC_PARANOID(("received data size: %d", data.length()))
    cdrMemoryStream cdr;
    // set endian type
    bool endian_type = m_connector->isLittleEndian();
    RTC_TRACE(("connector endian: %s", endian_type ? "little":"big"));
    cdr.setByteSwapFlag(endian_type);
    cdr.put_octet_array(&(data[0]), data.length());

    RTC_PARANOID(("converted CDR data size: %d", cdr.bufSize()));
    onReceived(cdr);
    BufferStatus::Enum ret = m_buffer->write(cdr);

    return convertReturn(ret, cdr);
  }

  /*!
   * @if jp
   * @brief リターンコード変換
   * @else
   * @brief Return codes conversion
   * @endif
   */
  ::OpenRTM::PortStatus
  InPortCorbaCdrProvider::convertReturn(BufferStatus::Enum status,
                                        const cdrMemoryStream& data)
  {
    switch(status)
      {
      case BufferStatus::BUFFER_OK:
        onBufferWrite(data);
        return ::OpenRTM::PORT_OK;
        break;

      case BufferStatus::BUFFER_ERROR:
        onReceiverError(data);
        return ::OpenRTM::PORT_ERROR;
        break;

      case BufferStatus::BUFFER_FULL:
        onBufferFull(data);
        onReceiverFull(data);
        return ::OpenRTM::BUFFER_FULL;
        break;

      case BufferStatus::BUFFER_EMPTY:
        // never come here
        return ::OpenRTM::BUFFER_EMPTY;
        break;

      case BufferStatus::PRECONDITION_NOT_MET:
        onReceiverError(data);
        return ::OpenRTM::PORT_ERROR;
        break;

      case BufferStatus::TIMEOUT:
        onBufferWriteTimeout(data);
        onReceiverTimeout(data);
        return ::OpenRTM::BUFFER_TIMEOUT;
        break;

      default:
        return ::OpenRTM::UNKNOWN_ERROR;
      }

    onReceiverError(data);
    return ::OpenRTM::UNKNOWN_ERROR;
  }

};     // namespace RTC


extern "C"
{
  /*!
   * @if jp
   * @brief モジュール初期化関数
   * @else
   * @brief Module initialization
   * @endif
   */
  void InPortCorbaCdrProviderInit(void)
  {
    RTC::InPortProviderFactory& factory(RTC::InPortProviderFactory::instance());
    factory.addFactory("corba_cdr",
                       ::coil::Creator< ::RTC::InPortProvider,
                                        ::RTC::InPortCorbaCdrProvider>,
                       ::coil::Destructor< ::RTC::InPortProvider,
                                           ::RTC::InPortCorbaCdrProvider>);
  }
};
