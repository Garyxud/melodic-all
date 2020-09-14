// -*- C++ -*-
/*!
 * @file  OutPortCorbaCdrProvider.cpp
 * @brief OutPortCorbaCdrProvider class
 * @date  $Date: 2008-01-14 07:52:40 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2010
 *     Noriaki Ando
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id: OutPortCorbaCdrProvider.cpp 1244 2009-03-13 07:25:42Z n-ando $
 *
 */

#include <rtm/OutPortCorbaCdrProvider.h>

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
  OutPortCorbaCdrProvider::OutPortCorbaCdrProvider(void)
   : m_buffer(0) 
  {
    // PortProfile setting
    setInterfaceType("corba_cdr");
    
    // ConnectorProfile setting
    m_objref = this->_this();
    
    // set outPort's reference
    CORBA::ORB_ptr orb = ::RTC::Manager::instance().getORB();
    CORBA::String_var ior = orb->object_to_string(m_objref.in());
#ifndef ORB_IS_RTORB
    CORBA_SeqUtil::
      push_back(m_properties,
                NVUtil::newNV("dataport.corba_cdr.outport_ior", ior));
#else // ORB_IS_RTORB
    CORBA_SeqUtil::
      push_back(m_properties,
                NVUtil::newNV("dataport.corba_cdr.outport_ior", ior.in()));
#endif // ORB_IS_RTORB
    CORBA_SeqUtil::
      push_back(m_properties,
                NVUtil::newNV("dataport.corba_cdr.outport_ref", m_objref));
  }
  
  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  OutPortCorbaCdrProvider::~OutPortCorbaCdrProvider(void)
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
  
  /*!
   * @if jp
   * @brief 設定初期化
   * @else
   * @brief Initializing configuration
   * @endif
   */
  void OutPortCorbaCdrProvider::init(coil::Properties& prop)
  {
  }

  /*!
   * @if jp
   * @brief バッファをセットする
   * @else
   * @brief Setting outside buffer's pointer
   * @endif
   */
  void OutPortCorbaCdrProvider::setBuffer(CdrBufferBase* buffer)
  {
    m_buffer = buffer;
  }

  /*!
   * @if jp
   * @brief リスナを設定する。
   * @else
   * @brief Set the listener. 
   * @endif
   */
  void OutPortCorbaCdrProvider::setListener(ConnectorInfo& info,
                                            ConnectorListeners* listeners)
  {
    m_profile = info;
    m_listeners = listeners;
  }

  /*!
   * @if jp
   * @brief コネクタをセットする
   * @else
   * @brief Setting outside connector's pointer
   * @endif
   */
  void OutPortCorbaCdrProvider::setConnector(OutPortConnector* connector)
  {
    m_connector = connector;
  }

  /*!
   * @if jp
   * @brief [CORBA interface] バッファからデータを取得する
   * @else
   * @brief [CORBA interface] Get data from the buffer
   * @endif
   */
  ::OpenRTM::PortStatus
  OutPortCorbaCdrProvider::get(::OpenRTM::CdrData_out data)
    throw (CORBA::SystemException)
  {
    RTC_PARANOID(("OutPortCorbaCdrProvider::get()"));
    // at least the output "data" area should be allocated
    data = new ::OpenRTM::CdrData();

    if (m_buffer == 0)
      {
        onSenderError();
        return ::OpenRTM::UNKNOWN_ERROR;
      }

    cdrMemoryStream cdr;
    CdrBufferBase::ReturnCode ret(m_buffer->read(cdr));
    
    if (ret == CdrBufferBase::BUFFER_OK)
      {
        CORBA::ULong len((CORBA::ULong)cdr.bufSize());
        RTC_PARANOID(("converted CDR data size: %d", len));

	if (len == (CORBA::ULong)0) {
	  RTC_ERROR(("buffer is empty."));
	  return ::OpenRTM::BUFFER_EMPTY;
	}
#ifndef ORB_IS_RTORB
        data->length(len);
        cdr.get_octet_array(&((*data)[0]), len);
#else
        data->length(len);
        cdr.get_octet_array((char *)&((*data)[0]), (int)len);
#endif // ORB_IS_RTORB
      }

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
  OutPortCorbaCdrProvider::convertReturn(BufferStatus::Enum status,
                                        const cdrMemoryStream& data)
  {
    switch(status)
      {
      case BufferStatus::BUFFER_OK:
        onBufferRead(data);
        onSend(data);
        return ::OpenRTM::PORT_OK;
        break;

      case BufferStatus::BUFFER_ERROR:
        onSenderError();
        return ::OpenRTM::PORT_ERROR;
        break;

      case BufferStatus::BUFFER_FULL:
        // never come here
        return ::OpenRTM::BUFFER_FULL;
        break;

      case BufferStatus::BUFFER_EMPTY:
        onBufferEmpty();
        onSenderEmpty();
        return ::OpenRTM::BUFFER_EMPTY;
        break;

      case BufferStatus::PRECONDITION_NOT_MET:
        onSenderError();
        return ::OpenRTM::PORT_ERROR;
        break;

      case BufferStatus::TIMEOUT:
        onBufferReadTimeout();
        onSenderTimeout();
        return ::OpenRTM::BUFFER_TIMEOUT;
        break;

      default:
        return ::OpenRTM::UNKNOWN_ERROR;
      }

    onSenderError();
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
  void OutPortCorbaCdrProviderInit(void)
  {
    RTC::OutPortProviderFactory&
      factory(RTC::OutPortProviderFactory::instance());
    factory.addFactory("corba_cdr",
                       ::coil::Creator< ::RTC::OutPortProvider,
                                        ::RTC::OutPortCorbaCdrProvider>,
                       ::coil::Destructor< ::RTC::OutPortProvider,
                                           ::RTC::OutPortCorbaCdrProvider>);
  }
};
