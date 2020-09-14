// -*- C++ -*-
/*!
 * @file  IFsm.h
 * @brief IFsm interface class
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

#ifndef RTC_LOCAL_IFSM_H
#define RTC_LOCAL_IFSM_H
 
namespace RTC
{
namespace Local
{
    /*!
     * @if jp
     * @class IFsm
     * @brief IFsm インターフェースクラス
     * @else
     * @class IFsm
     * @brief IFsm interface class
     * @endif
     */
    class IFsm
    {
    public:
      virtual ~IFsm() {};
    };
};     // namespace Local
};     // namespace RTC
#endif // RTC_LOCAL_IFSM_H
  
  
