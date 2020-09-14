// -*- C++ -*-
/*!
 * @file  Signal_win32.h
 * @brief SignalAction class
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


#ifndef COIL_SIGNAL_H
#define COIL_SIGNAL_H
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

namespace coil
{
#define _SIG_WORDS      4
#define _SIG_MAXSIG     NSIG

  typedef void (*SignalHandler)(int);
  typedef struct __sigset {
	  unsigned int __bits[_SIG_WORDS];
  } sigset_t;	

  /*!
   * @if jp
   *
   * @class SignalAction
   * @brief SignalAction クラス
   *
   * @else
   *
   * @class SignalAction
   * @brief SignalAction class
   *
   * @endif
   */
  class SignalAction
  {
  public:
    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * コンストラクタ。
     *
     * @else
     *
     * @brief Constructor
     *
     * Constructor
     *
     * @endif
     */
    SignalAction();

    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * コンストラクタ。
     *
     * @param handle シグナルハンドラ
     * @param signum シグナル番号
     * @param mask マスク
     * @param flags フラグ
     *
     * @else
     *
     * @brief Constructor
     *
     * Constructor
     *
     * @param handle Signal handler.
     * @param signum Signal number.
     * @param mask Signal mask.
     * @param flags Signal flag.
     *
     * @endif
     */
    SignalAction(SignalHandler handle, int signum, sigset_t *mask = 0,
                 int flags = 0);
    
    /*!
     * @if jp
     *
     * @brief デストラクタ
     *
     * デストラクタ。
     *
     * @else
     *
     * @brief Destructor
     *
     * Destructor
     *
     * @endif
     */
    ~SignalAction();

  private:
    SignalAction(const SignalAction&);
    SignalAction& operator=(const SignalAction &);
    SignalHandler m_handle;
    int m_signum;
    sigset_t* m_mask;
    int m_flags;

  };
};
#endif // COIL_SIGNAL_H
