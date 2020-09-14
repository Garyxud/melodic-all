// -*- C++ -*-
/*!
 * @file  Mutex_posix.h
 * @brief coil POSIX mutex class
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

#ifndef COIL_MUTEX_H
#define COIL_MUTEX_H

#include <pthread.h>

namespace coil
{
  /*!
   * @if jp
   *
   * @class Mutex
   * @brief Mutex クラス
   *
   * @else
   *
   * @class Mutex
   * @brief Mutex class
   *
   * @endif
   */
  class Mutex
  {
  public:
    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * コンストラクタ。
     *
     * @param name オブジェクト名
     *
     * @else
     *
     * @brief Constructor
     *
     * Constructor
     *
     * @param name Object name
     *
     * @endif
     */
    Mutex(const char * const name = 0)
    {
      ::pthread_mutex_init(&mutex_, 0);
    }

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
    ~Mutex()
    {
      ::pthread_mutex_destroy(&mutex_);
    }

    /*!
     * @if jp
     *
     * @brief 排他制御のロック
     *
     * 排他制御のロックを行う。
     *
     * @else
     *
     * @brief Mutual exclusion lock
     *
     * Lock the Mutual exclusion.
     *
     * @endif
     */
    inline void lock()
    {
      ::pthread_mutex_lock(&mutex_);
    }

    /*!
     * @if jp
     *
     * @brief 排他制御のノンブロッキングロック
     *
     * 排他制御のロックをノンブロッキングで行う。
     *
     * @else
     *
     * @brief Mutual exclusion non-blocking lock
     *
     * Lock the Mutual exclusion by non-blocking.
     *
     * @endif
     */
    inline bool trylock()
    {
      return ::pthread_mutex_trylock(&mutex_);
    }

    /*!
     * @if jp
     *
     * @brief 排他制御のロック解除
     *
     * 排他制御のロック解除を行う。
     *
     * @else
     *
     * @brief Mutual exclusion unlock
     *
     * Unlock the Mutual exclusion.
     *
     * @endif
     */
    inline void unlock()
    {
      ::pthread_mutex_unlock(&mutex_);
    }

    /*!
     * @if jp
     *
     * @brief 排他制御オブジェクト
     *
     * @else
     *
     * @brief Mutual exclusion object
     *
     * @endif
     */
    pthread_mutex_t mutex_;

  private:
    Mutex(const Mutex&);
    Mutex& operator=(const Mutex &);
  };
};
#endif // COIL_MUTEX_H
