// -*- C++ -*-
/*!
 * @file  ExecutionContextAdapter.h
 * @brief ExecutionContextAdapter
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

#ifndef DOIL_CORBA_SEQTYPECONVERSION_H
#define DOIL_CORBA_SEQTYPECONVERSION_H

#include <coil/stringutil.h>
#include <doil/corba/CORBA.h>
#include <string>
#include <vector>

//-------------------------------------------------------------
// local -> corba
template <typename LocalSeq, typename LocalElement,
          typename CORBASeq, typename CORBAElement>
bool local_to_corba_seq(const LocalSeq& _from, CORBASeq& _to)
{
  long int len(_from.size());
  _to.length((::CORBA::Long)len);
  for (long int i(0); i < len; ++i)
    {
      if (!local_to_corba(_from[i], _to[i])) return false;
    }
  return true;
}


template <typename CORBASeq, typename CORBAType,
          typename LocalSeq, typename LocalType>
bool corba_to_local_seq(const CORBASeq& _from, LocalSeq& _to)
{
  ::CORBA::Long len(_from.length());
  _to.resize((long)len);
  for (::CORBA::Long i(0); i < len; ++i)
    {
      if (!corba_to_local(_from[i], _to[i]))
        return false;
    }
  return true;
}

#endif // DOIL_CORBA_SEQTYPECONVERSION_H
