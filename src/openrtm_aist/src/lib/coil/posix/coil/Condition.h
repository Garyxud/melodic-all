// -*- C++ -*-
/*!
 * @file  Condition_posix.h
 * @brief Condition variable for POSIX
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

#ifndef COIL_CONDITION_H
#define COIL_CONDITION_H

#include <pthread.h>
#include <algorithm>
#include <ctime>
#include <sys/time.h>

namespace coil
{
  /*!
   * @if jp
   *
   * @class Condition
   * @brief Condition テンプレートクラス
   *
   * @else
   *
   * @class Condition
   * @brief Condition template class
   *
   * @endif
   */
  template <class M>
  class Condition
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
    Condition(M& mutex)
      : m_mutex(mutex)
    {
      ::pthread_cond_init(&m_cond, 0);
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
    ~Condition()
    {
      ::pthread_cond_destroy(&m_cond);
    }

    /*!
     * @if jp
     *
     * @brief スレッド実行の再開
     *
     * 待機しているスレッド実行を再開させる。
     *
     * @else
     *
     * @brief Resume of the thread practice
     *
     * Let the practice of a waiting thread resume.
     *
     * @endif
     */
    inline void signal()
    {
      ::pthread_cond_signal(&m_cond);
    }

    /*!
     * @if jp
     *
     * @brief 全スレッド実行の再開
     *
     * 待機している全てのスレッド実行を再開させる。
     *
     * @else
     *
     * @brief Resume of all the thread practice
     *
     * Let all waiting thread practice resume.
     *
     * @endif
     */
    inline void broadcast()
    {
      ::pthread_cond_broadcast(&m_cond);
    }

    /*!
     * @if jp
     *
     * @brief スレッド実行の待機
     *
     * 条件変数が送信されるまでスレッドの実行を停止する。
     *
     * @return true: 成功, false: 失敗
     *
     * @else
     *
     * @brief Wait of the thread practice
     *
     * Stop the practice of the thread till a condition variable is transmitted.
     *
     * @return true: successful, false: failed
     *
     * @endif
     */
    bool wait()
    {
      return 0 == ::pthread_cond_wait(&m_cond, &m_mutex.mutex_);
    }

    /*!
     * @if jp
     *
     * @brief 設定時間のスレッド実行待機
     *
     * 設定された時間、スレッドの実行を停止する。
     *
     * @param second 秒単位の時間
     * @param nano_second ナノ秒単位の時間
     *
     * @return true: 成功, false: 失敗
     *
     * @else
     *
     * @brief Thread practice wait of set time
     *
     * In set time, stop the practice of the thread.
     *
     * @param second Time of the seconds.
     * @param nano_second time of the nanoseconds.
     *
     * @return true: successful, false: failed
     *
     * @endif
     */
    bool wait(long second, long nano_second = 0)
    {
      struct timeval tv;
      timespec abstime;

      ::gettimeofday(&tv, NULL);
      abstime.tv_sec  = tv.tv_sec + second;
      abstime.tv_nsec = tv.tv_usec * 1000 + nano_second;
      if (abstime.tv_nsec >= 1000000000) {
        abstime.tv_nsec -= 1000000000;
        abstime.tv_sec ++;
      }
      return 0 == ::pthread_cond_timedwait(&m_cond, &m_mutex.mutex_, &abstime);
    }

  private:
    Condition(const M&);
    Condition& operator=(const M &);
    pthread_cond_t m_cond;
    M& m_mutex;
  };
};
#endif // COIL_CONDITION_H
