#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file  PublisherPeriodic.py
# @brief PublisherPeriodic class
# @date  $Date: 2007/09/28 $
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2006-2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import threading
from omniORB import any

import OpenRTM_aist


##
# @if jp
# @class PublisherPeriodic
# @brief PublisherPeriodic クラス
#
# 周期的にデータを送信するための Publisher クラス。このクラスは、通
# 常 Connector 内にあって、バッファおよびコンシューマに関連付けられ
# る。一定周期ごとにバッファからデータを取り出しコンシューマに対して
# データを送出する。
#
# @else
# @class PublisherPeriodic
# @brief PublisherPeriodic class
#
# Publisher for periodic data transmitting. Usually this class
# object exists in a Connector object, and it is associated with a
# buffer and a consumer. This publisher periodically gets data from
# the buffer and publish it into the consumer.
#
# @endif
#
class PublisherPeriodic(OpenRTM_aist.PublisherBase):
  """
  """

  # Policy
  ALL  = 0
  FIFO = 1
  SKIP = 2
  NEW  = 3

  ##
  # @if jp
  # @brief コンストラクタ
  #
  # コンストラクタ
  # 送出処理の呼び出し間隔を、Propertyオブジェクトのdataport.push_rateメンバ
  # に設定しておく必要がある。送出間隔は、Hz単位の浮動小数文字列で指定。
  # たとえば、1000.0Hzの場合は、「1000.0」を設定。
  # 上記プロパティが未設定の場合は、「1000Hz」を設定。
  #
  # @param self
  # @param consumer データ送出を待つコンシューマ
  # @param property 本Publisherの駆動制御情報を設定したPropertyオブジェクト
  #
  # @else
  # @brief Constructor
  # @endif
  def __init__(self):
    self._rtcout = OpenRTM_aist.Manager.instance().getLogbuf("PublisherPeriodic")
    self._consumer   = None
    self._buffer     = None
    self._task       = None
    self._retcode    = self.PORT_OK
    self._retmutex   = threading.RLock()
    self._pushPolicy = self.NEW
    self._skipn      = 0
    self._active     = False
    self._readback   = False
    self._leftskip   = 0
    self._profile    = None
    self._listeners  = None

    return

  ##
  # @if jp
  # @brief デストラクタ
  #
  # デストラクタ
  #
  # @param self
  #
  # @else
  # @brief Destructor
  # @endif
  def __del__(self):
    self._rtcout.RTC_TRACE("~PublisherPeriodic()")
    if self._task:
      self._task.resume()
      self._task.finalize()
      self._rtcout.RTC_PARANOID("task finalized.")

      OpenRTM_aist.PeriodicTaskFactory.instance().deleteObject(self._task)
      del self._task
      self._rtcout.RTC_PARANOID("task deleted.")

    # "consumer" should be deleted in the Connector
    self._consumer = None
    # "buffer"   should be deleted in the Connector
    self._buffer = None
    return

  ##
  # @if jp
  # @brief PushPolicy の設定
  # @else
  # @brief Setting PushPolicy
  # @endif
  #
  #void PublisherNew::setPushPolicy(const coil::Properties& prop)
  def setPushPolicy(self, prop):
    push_policy = prop.getProperty("publisher.push_policy","new")
    self._rtcout.RTC_DEBUG("push_policy: %s", push_policy)

    push_policy = OpenRTM_aist.normalize([push_policy])

    if push_policy == "all":
      self._pushPolicy = self.ALL

    elif push_policy == "fifo":
      self._pushPolicy = self.FIFO

    elif push_policy == "skip":
      self._pushPolicy = self.SKIP

    elif push_policy == "new":
      self._pushPolicy = self.NEW

    else:
      self._rtcout.RTC_ERROR("invalid push_policy value: %s", push_policy)
      self._pushPolicy = self.NEW
  
    skip_count = prop.getProperty("publisher.skip_count","0")
    self._rtcout.RTC_DEBUG("skip_count: %s", skip_count)

    skipn = [self._skipn]
    ret = OpenRTM_aist.stringTo(skipn, skip_count)
    if ret:
      self._skipn = skipn[0]
    else:
      self._rtcout.RTC_ERROR("invalid skip_count value: %s", skip_count)
      self._skipn = 0

    if self._skipn < 0:
      self._rtcout.RTC_ERROR("invalid skip_count value: %d", self._skipn)
      self._skipn = 0

    return

  ##
  # @if jp
  # @brief Task の設定
  # @else
  # @brief Setting Task
  # @endif
  #
  #bool PublisherNew::createTask(const coil::Properties& prop)
  def createTask(self, prop):
    factory = OpenRTM_aist.PeriodicTaskFactory.instance()

    th = factory.getIdentifiers()
    self._rtcout.RTC_DEBUG("available task types: %s", OpenRTM_aist.flatten(th))

    self._task = factory.createObject(prop.getProperty("thread_type", "default"))
    if not self._task:
      self._rtcout.RTC_ERROR("Task creation failed: %s",
                             prop.getProperty("thread_type", "default"))
      return self.INVALID_ARGS

    self._rtcout.RTC_PARANOID("Task creation succeeded.")

    # setting task function
    self._task.setTask(self.svc)

    # Task execution rate
    rate = prop.getProperty("publisher.push_rate")

    if rate != "":
      hz = float(rate)
      if hz == 0:
        hz = 1000.0
      self._rtcout.RTC_DEBUG("Task period %f [Hz]", hz)
    else:
      hz = 1000.0

    self._task.setPeriod(1.0/hz)
    
    # Measurement setting
    mprop = prop.getNode("measurement")

    self._task.executionMeasure(OpenRTM_aist.toBool(mprop.getProperty("exec_time"),
                                                    "enable", "disable", True))
    
    ecount = [0]
    if OpenRTM_aist.stringTo(ecount, mprop.getProperty("exec_count")):
      self._task.executionMeasureCount(ecount[0])

    self._task.periodicMeasure(OpenRTM_aist.toBool(mprop.getProperty("period_time"),
                                                   "enable", "disable", True))

    pcount = [0]
    if OpenRTM_aist.stringTo(pcount, mprop.getProperty("period_count")):
      self._task.periodicMeasureCount(pcount[0])

    # Start task in suspended mode
    self._task.suspend()
    self._task.activate()
    self._task.suspend()

    return self.PORT_OK

  ##
  # @if jp
  # @brief 初期化
  #
  # このクラスのオブジェクトを使用するのに先立ち、必ずこの関数を呼び
  # 出す必要がある。引数には、このオブジェクトの各種設定情報を含む
  # Properties を与える。少なくとも、送出処理の呼び出し周期を単位
  # Hz の数値として Propertyオブジェクトの publisher.push_rate をキー
  # とする要素に設定する必要がある。周期 5ms すなわち、200Hzの場合、
  # 200.0 を設定する。 dataport.publisher.push_rate が未設定の場合、
  # false が返される。データをプッシュする際のポリシーとして
  # publisher.push_policy をキーとする値に、all, fifo, skip, new の
  # いずれかを与えることができる。
  # 
  # 以下のオプションを与えることができる。
  # 
  # - publisher.thread_type: スレッドのタイプ (文字列、デフォルト: default)
  # - publisher.push_rate: Publisherの送信周期 (数値)
  # - publisher.push_policy: Pushポリシー (all, fifo, skip, new)
  # - publisher.skip_count: 上記ポリシが skip のときのスキップ数
  # - measurement.exec_time: タスク実行時間計測 (enable/disable)
  # - measurement.exec_count: タスク関数実行時間計測周期 (数値, 回数)
  # - measurement.period_time: タスク周期時間計測 (enable/disable)
  # - measurement.period_count: タスク周期時間計測周期 (数値, 回数)
  #
  # @param property 本Publisherの駆動制御情報を設定したPropertyオブジェクト
  # @return ReturnCode PORT_OK 正常終了
  #                    INVALID_ARGS Properties が不正な値を含む
  #
  # @else
  # @brief Initialization
  #
  # This function have to be called before using this class object.
  # Properties object that includes certain configuration
  # information should be given as an argument.  At least, a
  # numerical value of unit of Hz with the key of
  # "dataport.publisher.push_rate" has to be set to the Properties
  # object of argument.  The value is the invocation cycle of data
  # sending process.  In case of 5 ms period or 200 Hz, the value
  # should be set as 200.0. False will be returned, if there is no
  # value with the key of "dataport.publisher.push_rate".
  #
  # The following options are available.
  # 
  # - publisher.thread_type: Thread type (string, default: default)
  # - publisher.push_rate: Publisher sending period (numberical)
  # - publisher.push_policy: Push policy (all, fifo, skip, new)
  # - publisher.skip_count: The number of skip count in the "skip" policy
  # - measurement.exec_time: Task execution time measurement (enable/disable)
  # - measurement.exec_count: Task execution time measurement count
  #                           (numerical, number of times)
  # - measurement.period_time: Task period time measurement (enable/disable)
  # - measurement.period_count: Task period time measurement count 
  #                             (number, count)
  #
  # @param property Property objects that includes the control information
  #                 of this Publisher
  # @return ReturnCode PORT_OK normal return
  #                    INVALID_ARGS Properties with invalid values.
  # @endif
  #
  # PublisherBase::ReturnCode PublisherPeriodic::init(coil::Properties& prop)
  def init(self, prop):
    self._rtcout.RTC_TRACE("init()")
    self.setPushPolicy(prop)
    return self.createTask(prop)
  
  ##
  # @if jp
  # @brief InPortコンシューマのセット
  #
  # この関数では、この Publisher に関連付けられるコンシューマをセットする。
  # コンシューマオブジェクトがヌルポインタの場合、INVALID_ARGSが返される。
  # それ以外の場合は、PORT_OK が返される。
  #
  # @param consumer Consumer へのポインタ
  # @return ReturnCode PORT_OK 正常終了
  #                    INVALID_ARGS 引数に不正な値が含まれている
  #
  # @else
  # @brief Store InPort consumer
  #
  # This operation sets a consumer that is associated with this
  # object. If the consumer object is NULL, INVALID_ARGS will be
  # returned.
  #
  # @param consumer A pointer to a consumer object.
  # @return ReturnCode PORT_OK normal return
  #                    INVALID_ARGS given argument has invalid value
  #
  # @endif
  #
  # PublisherBase::ReturnCode
  # PublisherPeriodic::setConsumer(InPortConsumer* consumer)
  def setConsumer(self, consumer):
    self._rtcout.RTC_TRACE("setConsumer()")

    if not consumer:
      self._rtcout.RTC_ERROR("setConsumer(consumer = 0): invalid argument.")
      return self.INVALID_ARGS

    self._consumer = consumer
    return self.PORT_OK
  
  ##
  # @if jp
  # @brief バッファのセット
  #
  # この関数では、この Publisher に関連付けられるバッファをセットする。
  # バッファオブジェクトがヌルポインタの場合、INVALID_ARGSが返される。
  # それ以外の場合は、PORT_OK が返される。
  #
  # @param buffer CDR buffer へのポインタ
  # @return ReturnCode PORT_OK 正常終了
  #                    INVALID_ARGS 引数に不正な値が含まれている
  #
  # @else
  # @brief Setting buffer pointer
  #
  # This operation sets a buffer that is associated with this
  # object. If the buffer object is NULL, INVALID_ARGS will be
  # returned.
  #
  # @param buffer A pointer to a CDR buffer object.
  # @return ReturnCode PORT_OK normal return
  #                    INVALID_ARGS given argument has invalid value
  #
  # @endif
  #
  # PublisherBase::ReturnCode PublisherPeriodic::setBuffer(CdrBufferBase* buffer)
  def setBuffer(self, buffer):
    self._rtcout.RTC_TRACE("setBuffer()")
    
    if not buffer:
      self._rtcout.RTC_ERROR("setBuffer(buffer == 0): invalid argument")
      return self.INVALID_ARGS

    self._buffer = buffer
    return self.PORT_OK

  ##
  # @if jp
  # @brief リスナを設定する。
  #
  # Publisher に対してリスナオブジェクト ConnectorListeners を設定する。
  # 各種リスナオブジェクトを含む ConnectorListeners をセットすることで、
  # バッファの読み書き、データの送信時等にこれらのリスナをコールする。
  # ConnectorListeners オブジェクトの所有権はポートまたは RTObject が持ち
  # Publisher 削除時に ConnectorListeners は削除されることはない。
  # ConnectorListeners がヌルポインタの場合 INVALID_ARGS を返す。
  #
  # @param info ConnectorProfile をローカル化したオブジェクト ConnectorInfo
  # @param listeners リスナを多数保持する ConnectorListeners オブジェクト
  # @return PORT_OK      正常終了
  #         INVALID_ARGS 不正な引数
  # @else
  # @brief Set the listener. 
  #
  # This function sets ConnectorListeners listener object to the
  # Publisher. By setting ConnectorListeners containing various
  # listeners objects, these listeners are called at the time of
  # reading and writing of a buffer, and transmission of data
  # etc. Since the ownership of the ConnectorListeners object is
  # owned by Port or RTObject, the Publisher never deletes the
  # ConnectorListeners object. If the given ConnectorListeners'
  # pointer is NULL, this function returns INVALID_ARGS.
  #
  # @param info ConnectorInfo that is localized object of ConnectorProfile
  # @param listeners ConnectorListeners that holds various listeners
  # @return PORT_OK      Normal return
  #         INVALID_ARGS Invalid arguments
  # @endif
  #
  #PublisherBase::ReturnCode
  #PublisherPeriodic::setListener(ConnectorInfo& info,
  #                               ConnectorListeners* listeners)
  def setListener(self, info, listeners):
    self._rtcout.RTC_TRACE("setListeners()")

    if not listeners:
      self._rtcout.RTC_ERROR("setListeners(listeners == 0): invalid argument")
      return self.INVALID_ARGS

    self._profile = info
    self._listeners = listeners
    return self.PORT_OK

  ##
  # @if jp
  # @brief データを書き込む
  #
  # Publisher が保持するバッファに対してデータを書き込む。コンシュー
  # マ、バッファ、リスナ等が適切に設定されていない等、Publisher オブ
  # ジェクトが正しく初期化されていない場合、この関数を呼び出すとエラー
  # コード PRECONDITION_NOT_MET が返され、バッファへの書き込み等の操
  # 作は一切行われない。
  #
  # バッファへの書き込みと、InPortへのデータの送信は非同期的に行われ
  # るため、この関数は、InPortへのデータ送信の結果を示す、
  # CONNECTION_LOST, BUFFER_FULL などのリターンコードを返すことがあ
  # る。この場合、データのバッファへの書き込みは行われない。
  #
  # バッファへの書き込みに対して、バッファがフル状態、バッファのエ
  # ラー、バッファへの書き込みがタイムアウトした場合、バッファの事前
  # 条件が満たされない場合にはそれぞれ、エラーコード BUFFER_FULL,
  # BUFFER_ERROR, BUFFER_TIMEOUT, PRECONDITION_NOT_MET が返される。
  #
  # これら以外のエラーの場合、PORT_ERROR が返される。
  # 
  #
  # @param data 書き込むデータ 
  # @param sec タイムアウト時間
  # @param nsec タイムアウト時間
  #
  # @return PORT_OK             正常終了
  #         PRECONDITION_NO_MET consumer, buffer, listener等が適切に設定
  #                             されていない等、このオブジェクトの事前条件
  #                             を満たさない場合。
  #         CONNECTION_LOST     接続が切断されたことを検知した。
  #         BUFFER_FULL         バッファがフル状態である。
  #         BUFFER_ERROR        バッファに何らかのエラーが生じた場合。
  #         NOT_SUPPORTED       サポートされない操作が行われた。
  #         TIMEOUT             タイムアウトした。
  #
  # @else
  # @brief Write data 
  #
  # This function writes data into the buffer associated with this
  # Publisher.  If a Publisher object calls this function, without
  # initializing correctly such as a consumer, a buffer, listeners,
  # etc., error code PRECONDITION_NOT_MET will be returned and no
  # operation of the writing to a buffer etc. will be performed.
  #
  # Since writing into the buffer and sending data to InPort are
  # performed asynchronously, occasionally this function returns
  # return-codes such as CONNECTION_LOST and BUFFER_FULL that
  # indicate the result of sending data to InPort. In this case,
  # writing data into buffer will not be performed.
  #
  # When publisher writes data to the buffer, if the buffer is
  # filled, returns error, is returned with timeout and returns
  # precondition error, error codes BUFFER_FULL, BUFFER_ERROR,
  # BUFFER_TIMEOUT and PRECONDITION_NOT_MET will be returned
  # respectively.
  #
  # In other cases, PROT_ERROR will be returned.
  #
  # @param data Data to be wrote to the buffer
  # @param sec Timeout time in unit seconds
  # @param nsec Timeout time in unit nano-seconds
  # @return PORT_OK             Normal return
  #         PRECONDITION_NO_MET Precondition does not met. A consumer,
  #                             a buffer, listenes are not set properly.
  #         CONNECTION_LOST     detected that the connection has been lost
  #         BUFFER_FULL         The buffer is full status.
  #         BUFFER_ERROR        Some kind of error occurred in the buffer.
  #         NOT_SUPPORTED       Some kind of operation that is not supported
  #                             has been performed.
  #         TIMEOUT             Timeout occurred when writing to the buffer.
  #
  # @endif
  # 
  # PublisherBase::ReturnCode
  # PublisherPeriodic::write(const cdrMemoryStream& data,
  #                          unsigned long sec,
  #                          unsigned long usec)
  def write(self, data, sec, usec):
    self._rtcout.RTC_PARANOID("write()")

    if not self._consumer or not self._buffer or not self._listeners:
      return self.PRECONDITION_NOT_MET

    if self._retcode == self.CONNECTION_LOST:
      self._rtcout.RTC_DEBUG("write(): connection lost.")
      return self._retcode

    if self._retcode == self.SEND_FULL:
      self._rtcout.RTC_DEBUG("write(): InPort buffer is full.")
      self._buffer.write(data,sec,usec)
      return self.BUFFER_FULL

    self.onBufferWrite(data)
    ret = self._buffer.write(data, sec, usec)
    self._rtcout.RTC_DEBUG("%s = write()", OpenRTM_aist.DataPortStatus.toString(ret))
    self._task.resume()
    return self.convertReturn(ret, data)

  ##
  # @if jp
  #
  # @brief アクティブ化確認
  # 
  # Publisher はデータポートと同期して activate/deactivate される。
  # activate() / deactivate() 関数によって、アクティブ状態と非アクティ
  # ブ状態が切り替わる。この関数により、現在アクティブ状態か、非アク
  # ティブ状態かを確認することができる。
  #
  # @return 状態確認結果(アクティブ状態:true、非アクティブ状態:false)
  #
  # @else
  #
  # @brief If publisher is active state
  # 
  # A Publisher can be activated/deactivated synchronized with the
  # data port.  The active state and the non-active state are made
  # transition by the "activate()" and the "deactivate()" functions
  # respectively. This function confirms if the publisher is in
  # active state.
  #
  # @return Result of state confirmation
  #         (Active state:true, Inactive state:false)
  #
  # @endif
  #
  # bool PublisherPeriodic::isActive()
  def isActive(self):
    return self._active

  ##
  # @if jp
  # @brief アクティブ化する
  #
  # Publisher をアクティブ化する。この関数を呼び出すことにより、
  # Publisherが持つ、データを送信するスレッドが動作を開始する。初期
  # 化が行われていないなどにより、事前条件を満たさない場合、エラーコー
  # ド PRECONDITION_NOT_MET を返す。
  #
  # @return PORT_OK 正常終了
  #         PRECONDITION_NOT_MET 事前条件を満たさない
  #
  # @else
  # @brief activation
  #
  # This function activates the publisher. By calling this
  # function, this publisher starts the thread that pushes data to
  # InPort. If precondition such as initialization process and so
  # on is not met, the error code PRECONDITION_NOT_MET is returned.
  #
  # @return PORT_OK normal return
  #         PRECONDITION_NOT_MET precondition is not met
  #
  # @endif
  #
  # PublisherBase::ReturnCode PublisherPeriodic::activate()
  def activate(self):
    if not self._task or not self._buffer:
      return self.PRECONDITION_NOT_MET
    self._active = True
    self._task.resume()
    return self.PORT_OK

  ##
  # @if jp
  # @brief 非アクティブ化する
  #
  # Publisher を非アクティブ化する。この関数を呼び出すことにより、
  # Publisherが持つ、データを送信するスレッドが動作を停止する。初期
  # 化が行われていないなどにより、事前条件を満たさない場合、エラーコー
  # ド PRECONDITION_NOT_MET を返す。
  #
  # @return PORT_OK 正常終了
  #         PRECONDITION_NOT_MET 事前条件を満たさない
  #
  # @else
  # @brief deactivation
  #
  # This function deactivates the publisher. By calling this
  # function, this publisher stops the thread that pushes data to
  # InPort. If precondition such as initialization process and so
  # on is not met, the error code PRECONDITION_NOT_MET is returned.
  #
  # @return PORT_OK normal return
  #         PRECONDITION_NOT_MET precondition is not met
  #
  # @endif
  #
  # PublisherBase::ReturnCode PublisherPeriodic::deactivate()
  def deactivate(self):
    if not self._task:
      return self.PRECONDITION_NOT_MET
    self._active = False
    self._task.suspend()
    return self.PORT_OK

  ##
  # @if jp
  # @brief スレッド実行関数
  # @else
  # @brief Thread execution function
  # A task execution function to be executed by coil::PeriodicTask.
  # @endif
  #
  # int PublisherPeriodic::svc(void)
  def svc(self):
    guard = OpenRTM_aist.ScopedLock(self._retmutex)

    if self._pushPolicy == self.ALL:
      self._retcode = self.pushAll()
      return 0

    elif self._pushPolicy == self.FIFO:
      self._retcode = self.pushFifo()
      return 0

    elif self._pushPolicy == self.SKIP:
      self._retcode = self.pushSkip()
      return 0

    elif self._pushPolicy == self.NEW:
      self._retcode = self.pushNew()
      return 0

    else:
      self._retcode = self.pushNew()

    return 0
  
  ##
  # @brief push all policy
  #
  # PublisherBase::ReturnCode PublisherPeriodic::pushAll()
  def pushAll(self):
    self._rtcout.RTC_TRACE("pushAll()")

    if not self._buffer:
      return self.PRECONDITION_NOT_MET      

    if self.bufferIsEmpty():
      return self.BUFFER_EMPTY

    while self._buffer.readable() > 0:
      cdr = self._buffer.get()
      self.onBufferRead(cdr)

      self.onSend(cdr)
      ret = self._consumer.put(cdr)

      if ret != self.PORT_OK:
        self._rtcout.RTC_DEBUG("%s = consumer.put()", OpenRTM_aist.DataPortStatus.toString(ret))
        return self.invokeListener(ret, cdr)

      self.onReceived(cdr)
      self._buffer.advanceRptr()

    return self.PORT_OK


  ##
  # @brief push "fifo" policy
  #
  # PublisherBase::ReturnCode PublisherPeriodic::pushFifo()
  def pushFifo(self):
    self._rtcout.RTC_TRACE("pushFifo()")
    if not self._buffer:
      return self.PRECONDITION_NOT_MET      

    if self.bufferIsEmpty():
      return self.BUFFER_EMPTY

    cdr = self._buffer.get()
    self.onBufferRead(cdr)

    self.onSend(cdr)
    ret = self._consumer.put(cdr)

    if ret != self.PORT_OK:
      self._rtcout.RTC_DEBUG("%s = consumer.put()",OpenRTM_aist.DataPortStatus.toString(ret))
      return self.invokeListener(ret, cdr)

    self.onReceived(cdr)
    self._buffer.advanceRptr()
    
    return self.PORT_OK


  ##
  # @brief push "skip" policy
  #
  # PublisherBase::ReturnCode PublisherPeriodic::pushSkip()
  def pushSkip(self):
    self._rtcout.RTC_TRACE("pushSkip()")
    if not self._buffer:
      return self.PRECONDITION_NOT_MET      

    if self.bufferIsEmpty():
      return self.BUFFER_EMPTY

    ret = self.PORT_OK
    preskip  = self._buffer.readable() + self._leftskip
    loopcnt  = preskip / (self._skipn + 1)
    postskip = self._skipn - self._leftskip
    for i in range(loopcnt):
      self._buffer.advanceRptr(postskip)
      cdr = self._buffer.get()
      self.onBufferRead(cdr)

      self.onSend(cdr)
      ret = self._consumer.put(cdr)
      if ret != self.PORT_OK:
        self._buffer.advanceRptr(-postskip)
        self._rtcout.RTC_DEBUG("%s = consumer.put()",OpenRTM_aist.DataPortStatus.toString(ret))
        return self.invokeListener(ret, cdr)
      self.onReceived(cdr)
      postskip = self._skipn + 1

    self._buffer.advanceRptr(self._buffer.readable())
    self._leftskip = preskip % (self._skipn + 1)
    
    return ret


  ##
  # @brief push "new" policy
  #
  # PublisherBase::ReturnCode PublisherPeriodic::pushNew()
  def pushNew(self):
    self._rtcout.RTC_TRACE("pushNew()")
    if not self._buffer:
      return self.PRECONDITION_NOT_MET      

    if self.bufferIsEmpty():
      return self.BUFFER_EMPTY

    # In case of the periodic/push_new policy, the buffer should
    # allow readback. But, readback flag should be set as "true"
    # after written at least one datum into the buffer.
    self._readback = True

    self._buffer.advanceRptr(self._buffer.readable() - 1)
    
    cdr = self._buffer.get()
    self.onBufferRead(cdr)

    self.onSend(cdr)
    ret = self._consumer.put(cdr)
    
    if ret != self.PORT_OK:
      self._rtcout.RTC_DEBUG("%s = consumer.put()", OpenRTM_aist.DataPortStatus.toString(ret))
      return self.invokeListener(ret, cdr)

    self.onReceived(cdr)

    self._buffer.advanceRptr()
    return self.PORT_OK

  ##
  # @if jp
  # @brief BufferStatus から DataPortStatus への変換
  #
  # バッファからの戻り値を DataPortStatus::Enum 型へ変換する関数。そ
  # れぞれ、以下のように変換される。変換時にコールバックを呼ぶ場合、
  # コールバク関数も付記する。
  # 
  # - BUFFER_OK: PORT_OK
  #  - None
  # - BUFFER_ERROR: BUFFER_ERROR
  #  - None
  # - BUFFER_FULL: BUFFER_FULL
  #  - onBufferFull()
  # - NOT_SUPPORTED: PORT_ERROR
  #  - None
  # - TIMEOUT: BUFFER_TIMEOUT
  #  - onBufferWriteTimeout()
  # - PRECONDITION_NOT_MET: PRECONDITION_NOT_MET
  #  - None
  # - other: PORT_ERROR
  #  - None
  #
  # @param status BufferStatus
  # @param data cdrMemoryStream
  # @return DataPortStatu 型のリターンコード
  #
  # @else
  # @brief Convertion from BufferStatus to DataPortStatus
  # 
  # This function converts return value from the buffer to
  # DataPortStatus::Enum typed return value. The conversion rule is
  # as follows. Callback functions are also shown, if it exists.
  # 
  # - BUFFER_OK: PORT_OK
  #  - None
  # - BUFFER_ERROR: BUFFER_ERROR
  #  - None
  # - BUFFER_FULL: BUFFER_FULL
  #  - onBufferFull()
  # - NOT_SUPPORTED: PORT_ERROR
  #  - None
  # - TIMEOUT: BUFFER_TIMEOUT
  #  - onBufferWriteTimeout()
  # - PRECONDITION_NOT_MET: PRECONDITION_NOT_MET
  #  - None
  # - other: PORT_ERROR
  #  - None
  #
  # @param status BufferStatus
  # @param data cdrMemoryStream
  # @return DataPortStatus typed return code
  #
  # @endif
  #
  # PublisherBase::ReturnCodea
  # PublisherPeriodic::convertReturn(BufferStatus::Enum status,
  #                                  const cdrMemoryStream& data)
  def convertReturn(self, status, data):
    if status == OpenRTM_aist.BufferStatus.BUFFER_OK:
      return self.PORT_OK

    elif status == OpenRTM_aist.BufferStatus.BUFFER_ERROR:
      return self.BUFFER_ERROR

    elif status == OpenRTM_aist.BufferStatus.BUFFER_FULL:
      self.onBufferFull(data)
      return self.BUFFER_FULL

    elif status == OpenRTM_aist.BufferStatus.NOT_SUPPORTED:
      return self.PORT_ERROR

    elif status == OpenRTM_aist.BufferStatus.TIMEOUT:
      self.onBufferWriteTimeout(data)
      return self.BUFFER_TIMEOUT

    elif status == OpenRTM_aist.BufferStatus.PRECONDITION_NOT_MET:
      return self.PRECONDITION_NOT_MET

    else:
      return self.PORT_ERROR
    
    return self.PORT_ERROR

  ##
  # @if jp
  # @brief DataPortStatusに従ってリスナへ通知する関数を呼び出す。
  #
  # @param status DataPortStatus
  # @param data cdrMemoryStream
  # @return リターンコード
  #
  # @else
  # @brief Call listeners according to the DataPortStatus
  #
  # @param status DataPortStatus
  # @param data cdrMemoryStream
  # @return Return code
  #
  # @endif
  #
  # PublisherPeriodic::ReturnCode
  # PublisherPeriodic::invokeListener(DataPortStatus::Enum status,
  #                                   const cdrMemoryStream& data)
  def invokeListener(self, status, data):
    # ret:
    # PORT_OK, PORT_ERROR, SEND_FULL, SEND_TIMEOUT, CONNECTION_LOST,
    # UNKNOWN_ERROR
    if status == self.PORT_ERROR:
      self.onReceiverError(data)
      return self.PORT_ERROR
        
    elif status == self.SEND_FULL:
      self.onReceiverFull(data)
      return self.SEND_FULL
        
    elif status == self.SEND_TIMEOUT:
      self.onReceiverTimeout(data)
      return self.SEND_TIMEOUT
        
    elif status == self.CONNECTION_LOST:
      self.onReceiverError(data)
      return self.CONNECTION_LOST
        
    elif status == self.UNKNOWN_ERROR:
      self.onReceiverError(data)
      return self.UNKNOWN_ERROR
        
    else:
      self.onReceiverError(data)
      return self.PORT_ERROR

  ##
  # @if jp
  # @brief ON_BUFFER_WRITEのリスナへ通知する。 
  # @param data cdrMemoryStream
  # @else
  # @brief Notify an ON_BUFFER_WRITE event to listeners
  # @param data cdrMemoryStream
  # @endif
  #
  # inline void onBufferWrite(const cdrMemoryStream& data)
  def onBufferWrite(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE].notify(self._profile, data)
    return

  ##
  # @if jp
  # @brief ON_BUFFER_FULLリスナへイベントを通知する。 
  # @param data cdrMemoryStream
  # @else
  # @brief Notify an ON_BUFFER_FULL event to listeners
  # @param data cdrMemoryStream
  # @endif
  #
  # inline void onBufferFull(const cdrMemoryStream& data)
  def onBufferFull(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_FULL].notify(self._profile, data)
    return

  ##
  # @if jp
  # @brief ON_BUFFER_WRITE_TIMEOUTのリスナへ通知する。 
  # @param data cdrMemoryStream
  # @else
  # @brief Notify an ON_BUFFER_WRITE_TIMEOUT event to listeners
  # @param data cdrMemoryStream
  # @endif
  #
  # inline void onBufferWriteTimeout(const cdrMemoryStream& data)
  def onBufferWriteTimeout(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_WRITE_TIMEOUT].notify(self._profile, data)
    return

  ##
  # @if jp
  # @brief ON_BUFFER_READのリスナへ通知する。 
  # @param data cdrMemoryStream
  # @else
  # @brief Notify an ON_BUFFER_READ event to listeners
  # @param data cdrMemoryStream
  # @endif
  #
  #  inline void onBufferRead(const cdrMemoryStream& data)
  def onBufferRead(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_BUFFER_READ].notify(self._profile, data)
    return

  ##
  # @if jp
  # @brief ON_SENDのリスナへ通知する。 
  # @param data cdrMemoryStream
  # @else
  # @brief Notify an ON_SEND event to listners
  # @param data cdrMemoryStream
  # @endif
  #
  # inline void onSend(const cdrMemoryStream& data)
  def onSend(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_SEND].notify(self._profile, data)
    return

  ##
  # @if jp
  # @brief ON_RECEIVEDのリスナへ通知する。 
  # @param data cdrMemoryStream
  # @else
  # @brief Notify an ON_RECEIVED event to listeners
  # @param data cdrMemoryStream
  # @endif
  #
  # inline void onReceived(const cdrMemoryStream& data)
  def onReceived(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVED].notify(self._profile, data)
    return

  ##
  # @if jp
  # @brief ON_RECEIVER_FULLのリスナへ通知する。 
  # @param data cdrMemoryStream
  # @else
  # @brief Notify an ON_RECEIVER_FULL event to listeners
  # @param data cdrMemoryStream
  # @endif
  #
  # inline void onReceiverFull(const cdrMemoryStream& data)
  def onReceiverFull(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_FULL].notify(self._profile, data)
    return

  ##
  # @if jp
  # @brief ON_RECEIVER_TIMEOUTのリスナへ通知する。 
  # @param data cdrMemoryStream
  # @else
  # @brief Notify an ON_RECEIVER_TIMEOUT event to listeners
  # @param data cdrMemoryStream
  # @endif
  #
  # inline void onReceiverTimeout(const cdrMemoryStream& data)
  def onReceiverTimeout(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_TIMEOUT].notify(self._profile, data)
    return

  ##
  # @if jp
  # @brief ON_RECEIVER_ERRORのリスナへ通知する。 
  # @param data cdrMemoryStream
  # @else
  # @brief Notify an ON_RECEIVER_ERROR event to listeners
  # @param data cdrMemoryStream
  # @endif
  #
  # inline void onReceiverError(const cdrMemoryStream& data)
  def onReceiverError(self, data):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connectorData_[OpenRTM_aist.ConnectorDataListenerType.ON_RECEIVER_ERROR].notify(self._profile, data)
    return


  ##
  # @if jp
  # @brief ON_BUFFER_EMPTYのリスナへ通知する。 
  # @else
  # @brief Notify an ON_BUFFER_EMPTY event to listeners
  # @endif
  #
  #inline void onBufferEmpty()
  def onBufferEmpty(self):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_BUFFER_EMPTY].notify(self._profile)
    return

  ##
  # @if jp
  # @brief ON_SENDER_EMPTYのリスナへ通知する。 
  # @else
  # @brief Notify an ON_SENDER_EMPTY event to listeners
  # @endif
  #
  # inline void onSenderEmpty()
  def onSenderEmpty(self):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_SENDER_EMPTY].notify(self._profile)
    return

  ##
  # @if jp
  # @brief ON_SENDER_ERRORのリスナへ通知する。 
  # @else
  # @brief Notify an ON_SENDER_ERROR event to listeners
  # @endif
  #
  # inline void onSenderError()
  def onSenderError(self):
    if self._listeners is not None and self._profile is not None:
      self._listeners.connector_[OpenRTM_aist.ConnectorListenerType.ON_SENDER_ERROR].notify(self._profile)
    return


  ##
  # @if jp
  # @brief バッファが空かどうかをチェックする。x
  # @else
  # @brief Whether a buffer is empty.
  # @endif
  #
  # bool bufferIsEmpty()
  def bufferIsEmpty(self):
    if self._buffer and self._buffer.empty() and  not self._readback:
      self._rtcout.RTC_DEBUG("buffer empty")
      self.onBufferEmpty()
      self.onSenderEmpty()
      return True

    return False



def PublisherPeriodicInit():
  OpenRTM_aist.PublisherFactory.instance().addFactory("periodic",
                                                      OpenRTM_aist.PublisherPeriodic,
                                                      OpenRTM_aist.Delete)
