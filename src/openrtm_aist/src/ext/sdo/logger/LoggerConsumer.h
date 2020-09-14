// -*- C++ -*-
/*!
 * @file LoggerConsumer.h
 * @brief Component observer SDO service consumer implementation
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2011
 *     Noriaki Ando
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id: SdoConfiguration.cpp 1971 2010-06-03 08:46:40Z n-ando $
 *
 */


#ifndef RTC_LOGGERCONSUMER_H
#define RTC_LOGGERCONSUMER_H

#include <coil/Mutex.h>
#include <coil/Factory.h>
#include <coil/stringutil.h>
#include <rtm/SdoServiceConsumerBase.h>
#include <rtm/CorbaConsumer.h>
#include <rtm/idl/SDOPackageStub.h>

#include "LoggerStub.h"

namespace RTC
{

  /*!
   * @if jp
   * @else
   * @endif
   */
  class LoggerConsumer
    : public SdoServiceConsumerBase
  {
  public:
    /*!
     * @if jp
     * @brief ctor of LoggerConsumer
     * @else
     * @brief ctor of LoggerConsumer
     * @endif
     */
    LoggerConsumer();

    /*!
     * @if jp
     * @brief dtor
     * @else
     * @brief dtor
     * @endif
     */
    virtual ~LoggerConsumer();

    /*!
     * @if jp
     * @brief èâä˙âª
     * @else
     * @brief Initialization
     * @endif
     */
    virtual bool init(RTObject_impl& rtobj,
                      const SDOPackage::ServiceProfile& profile);

    /*!
     * @if jp
     * @brief çƒèâä˙âª
     * @else
     * @brief Re-initialization
     * @endif
     */
    virtual bool reinit(const SDOPackage::ServiceProfile& profile);

    /*!
     * @if jp
     * @brief ServiceProfile ÇéÊìæÇ∑ÇÈ
     * @else
     * @brief getting ServiceProfile
     * @endif
     */
    virtual const SDOPackage::ServiceProfile& getProfile() const;
    
    /*!
     * @if jp
     * @brief èIóπèàóù
     * @else
     * @brief Finalization
     * @endif
     */
    virtual void finalize();

  protected:

    RTC::RTObject_impl* m_rtobj;
    SDOPackage::ServiceProfile m_profile;
    CorbaConsumer<OpenRTM::Logger> m_logger;
  };

}; // namespace RTC

extern "C"
{
  DLL_EXPORT void LoggerConsumerInit();
};

#endif // RTC_LOGGERCONSUMER_H


