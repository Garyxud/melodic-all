// -*- C++ -*-
/*!
 * @file  DataFlowComponentBase.h
 * @brief DataFlowComponentBase class
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

#ifndef DataFlowComponentBase_h
#define DataFlowComponentBase_h

#include <memory>
#include <rtc/IRTC.h>
#include <rtc/IDataFlowComponent.h>
 
namespace RTC
{
namespace Local
{
  /*!
   * @if jp
   * @class DataFlowComponentBase
   * @brief DataFlowComponentBase епеще╣
   * @else
   * @class DataFlowComponentBase
   * @brief DataFlowComponentBase class
   * @endif
   */
  class DataFlowComponentBase
    : public virtual IDataFlowComponent
  {
  public:
    typedef RTC::Local::ReturnCode_t ReturnCode_t;
    typedef RTC::Local::ExecutionContextHandle_t ExecutionContextHandle_t;
    typedef RTC::Local::ComponentProfile ComponentProfile;
    typedef RTC::Local::PortServiceList PortServiceList;
    typedef RTC::Local::ExecutionContextList ExecutionContextList;
    typedef RTC::Local::IExecutionContext IExecutionContext;

    DataFlowComponentBase();
    virtual ~DataFlowComponentBase();

    //------------------------------------------------------------
    // RTObject
    //------------------------------------------------------------
    virtual bool is_alive(IExecutionContext& ec);
    virtual ReturnCode_t initialize();
    virtual ReturnCode_t finalize();
    virtual ReturnCode_t exit();
    virtual const ComponentProfile& get_component_profile();
    virtual PortServiceList& get_ports();
    virtual ExecutionContextHandle_t attach_context(IExecutionContext& ec);
    virtual ReturnCode_t detach_context(ExecutionContextHandle_t ec_handle);
    virtual IExecutionContext& get_context(ExecutionContextHandle_t ec_handle);
    virtual ExecutionContextList& get_owned_contexts() const;
    virtual ExecutionContextList& get_participating_contexts() const;

    //------------------------------------------------------------
    // ComponentAction
    //------------------------------------------------------------
    virtual ReturnCode_t on_initialize();
    virtual ReturnCode_t on_finalize();
    virtual ReturnCode_t on_startup(ExecutionContextHandle_t ec_handle);
    virtual ReturnCode_t on_shutdown(ExecutionContextHandle_t ec_handle);
    virtual ReturnCode_t on_activated(ExecutionContextHandle_t ec_handle);
    virtual ReturnCode_t on_deactivated(ExecutionContextHandle_t ec_handle);
    virtual ReturnCode_t on_aborting(ExecutionContextHandle_t ec_handle);
    virtual ReturnCode_t on_error(ExecutionContextHandle_t ec_handle);
    virtual ReturnCode_t on_reset(ExecutionContextHandle_t ec_handle);

    //------------------------------------------------------------
    // DataFlowComponentAction
    //------------------------------------------------------------
    virtual ReturnCode_t on_execute(ExecutionContextHandle_t ec_handle);
    virtual ReturnCode_t on_state_update(ExecutionContextHandle_t ec_handle);
    virtual ReturnCode_t on_rate_changed(ExecutionContextHandle_t ec_handle);


  protected:
  
  private:
    class DataFlowComponentImpl;
    std::auto_ptr<DataFlowComponentImpl> pimpl;
  };
};     // namespace Local
};     // namespace RTC
#endif // DataFlowComponentBase_h

