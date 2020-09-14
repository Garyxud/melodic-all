// -*- C++ -*-
/*!
 * @file  CdrRingBuffer.h
 * @brief RingBuffer for CDR
 * @date  $Date: 2007-12-31 03:08:03 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2009
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

#ifndef RTC_CDRRINGBUFFER_H
#define RTC_CDRRINGBUFFER_H

#include <rtm/RingBuffer.h>
#include <rtm/CdrBufferBase.h>

namespace RTC
{
  typedef RingBuffer<cdrMemoryStream> CdrRingBuffer;
}; // namespace RTC

extern "C"
{
  void CdrRingBufferInit();
};
#endif // RTC_CDRRINGBUFFER_H
