// -*- C++ -*-
/*!
 * @file RTPreemptEC.h
 * @brief RTPreemptEC class
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2010
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef OPENRTM_RTPREEMPTEC_H
#define OPENRTM_RTPREEMPTEC_H

#include <rtm/RTC.h>
#include <rtm/Manager.h>
#include <rtm/PeriodicExecutionContext.h>

namespace OpenRTM
{
  /*!
   * @if jp
   * @class RTPreemptEC
   * @brief RTPreemptEC クラス
   *
   * RT-Preempt kernel を利用した、リアルタイム実行コンテキストクラス。
   * この実行コンテキストは、RT-Preempt Patch を適用した Linux kernel、
   * 又は、これが組み込まれた Linux kernel によるリアルタイムスケジュー
   * リング機能を利用した実行コンテキストである。
   *
   * この実行コンテキストを利用するには、rtc.conf に下記のように記述する。
   * 
   * <pre>
   * exec_cxt.periodic.type: RTPreemptEC
   * exec_cxt.periodic.rate: 1000
   * exec_cxt.priority: 50
   * manager.modules.load_path: <RTPreemptRC.so がある場所へのパス>
   * manager.modules.preload: RTPreemptEC.so
   * </pre>
   *
   * さらに、実行時には root 権限が必要となるので、root としてコンポー
   * ネントを実行する必要がある。
   *
   * このECに特有なオプションは以下のとおりである。
   *
   * - exec_cxt.periodic.priority: (default: 49) <br>
   * - exec_cxt.periodic.rtpreempt.priority: (default: 49)<br>
   *      スレッドの実行優先度 1 (最低) から 99 (最高)<br>
   *      Linux sched_setscheduler(2) を参照のこと。<br>
   *
   * - exec_cxt.periodic.rtpreempt.sched_policy:  (default: fifo)<br>
   *      スケジューリングのポリシ。<br>
   *      rr: ラウンドロビン, fifo: FIFO 型 (default: fifo)<br>
   *      Linux sched_setscheduler(2) を参照のこと。<br>
   *
   * - exec_cxt.periodic.rtpreempt.wait_offset: (default: -10000)<br>
   *      ウェイト時間のオフセット。[ns] 単位で指定する。 <br>
   *      1周期あたり数十 us 程度の定常的な遅れが発生する場合があるので、
   *      この値を調整することで、より正確な周期で実行させることができる。
   *
   * 注意事項: このECを比較的速い周期 (数十ms以上) で実行させる場合は、
   * ログレベルを DEBUG よりも下 (logger.log_level: NORMAL 等) に設定し
   * て実行する必要がある。また、logger.enable: NO に設定されていても、
   * logger.log_level: PARANOID に設定されている場合には、onExecute()
   * の実行に時間がかかり、デッドラインを守れなくなる可能性があるので注
   * 意が必要である。
   *
   * @since 1.0.1
   *
   * @else
   * @class RTPreemptEC
   * @brief RTPreemptEC クラス
   *
   * This class is real-time ExecutionContext which utilizes RT-Prempt
   * kernel.  This ExecutionContext is a real-time ExecutionContext
   * which utilizes real-time scheduler functionality of "RT-Preempt"
   * Linux kernel (patched or originally embedded).
   *
   * Give the following configurations in your rtc.conf to use this EC.
   * <pre>
   * exec_cxt.periodic.type: RTPreemptEC
   * exec_cxt.periodic.rate: 1000
   * exec_cxt.priority: 50
   * manager.modules.load_path: <path to RTPreemptRC.so>
   * manager.modules.preload: RTPreemptEC.so
   * </pre>
   *
   * Since this functionality requires root authority, your RTC which
   * uses this EC have to be executed as root.
   *
   * The following RTPreemptEC specific options are available.
   *
   * - exec_cxt.periodic.priority: (default: 49)<br>
   * - exec_cxt.periodic.rtpreempt.priority: (default: 49)<br>
   *      Execution priority of threads from 1 (lowest) to 99 (highest)<br>
   *      See Linux sched_setscheduler(2).
   *
   * - exec_cxt.periodic.rtpreempt.sched_policy:  (default: fifo)<br>
   *      Scheduling policy.<br>
   *      rr: round-robin, fifo: FIFO type scheduling (default: fifo)<br>
   *      See Linux sched_setscheduler(2).
   *
   * - exec_cxt.periodic.rtpreempt.wait_offset: (default: -10000)<br>
   *      Offset time of wait. It can be given [ns] unit. <br> 
   *      Adjusting this value, If dozens micro seconds delay in one
   *      execution cycle constantly, more accurate periodic execution
   *      can be achieved.
   *
   * NOTICE: When performing comparatively quick cycle (tens of ms or
   * more) using this EC, log-level have to be lower than DEBUG
   * (logger.log_level: NORMAL or etc.). Moreover, even if
   * "logger.enable=No" is set, if "logger.log_level=PRANOID" is set,
   * execution of onExecute() takes longer time and it may cause
   * deadline-miss. So caution is needed.
   *
   * @since 1.0.1
   *
   * @endif
   */
  class RTPreemptEC
    : public virtual ::RTC::PeriodicExecutionContext
  {
  public:
    /*!
     * @if jp
     * @brief デフォルトコンストラクタ
     *
     * デフォルトコンストラクタ
     *
     * @else
     * @brief Default Constructor
     *
     * Default Constructor
     *
     * @endif
     */
    RTPreemptEC();

    /*!
     * @if jp
     * @brief デストラクタ
     *
     * デストラクタ
     *
     * @else
     * @brief Destructor
     *
     * Destructor
     *
     * @endif
     */
    virtual ~RTPreemptEC();

    /*!
     * @if jp
     * @brief ExecutionContext 用のスレッド実行関数
     *
     * ExecutionContext 用のスレッド実行関数。
     * 登録されたコンポーネントの処理を呼び出す。
     *
     * @return 実行結果
     *
     * @else
     * @brief Thread execution function for ExecutionContext
     *
     * Thread execution function for ExecutionContext.
     * Invoke the registered components operation.
     *
     * @return The execution result
     *
     * @endif
     */ 
    virtual int svc(void);

    /*!
     * @if jp
     * @brief あるキーを持つプロパティを取得する
     *
     * @param ExecutionContext 用のスレッド実行関数。
     * 登録されたコンポーネントの処理を呼び出す。
     *
     * @return 実行結果
     *
     * @else
     * @brief Thread execution function for ExecutionContext
     *
     * Thread execution function for ExecutionContext.
     * Invoke the registered components operation.
     *
     * @return The execution result
     *
     * @endif
     */ 
    template <class T>
    void getProperty(coil::Properties& prop, const char* key, T& value)
    {
    if (prop.findNode(key) != 0)
      {
        T tmp;
        if (coil::stringTo(tmp, prop[key].c_str()))
          {
            value = tmp;
          }
      }
    }

  private:
    int m_priority;
    int m_policy;
    int m_waitoffset;
  };
};

extern "C"
{
  /*!
   * @if jp
   * @brief ECFactoryへの登録のための初期化関数
   * @else
   * @brief Initialization function to register to ECFactory
   * @endif
   */
  void DLL_EXPORT RTPreemptECInit(RTC::Manager* manager);
};

#endif // OPENRTM_RTPREEMPTEC_H

