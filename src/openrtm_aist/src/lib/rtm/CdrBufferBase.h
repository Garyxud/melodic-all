// -*- C++ -*-
/*!
 * @file  InPortConsumer.h
 * @brief InPortConsumer class
 * @date  $Date: 2007-12-31 03:08:03 $
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
 * $Id: InPortConsumer.h 1225 2009-02-28 02:30:25Z n-ando $
 *
 */

#ifndef RTC_CDRBUFFERBASE_H
#define RTC_CDRBUFFERBASE_H

#include <coil/Factory.h>

#include <rtm/RTC.h>
#include <rtm/BufferBase.h>

namespace RTC
{
  typedef BufferBase<cdrMemoryStream> CdrBufferBase;
  typedef ::coil::GlobalFactory<CdrBufferBase> CdrBufferFactory;
};     // namespace RTC
#endif // RTC_CDRBUFFERBASE_H
