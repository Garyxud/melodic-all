#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file SystemLogger.py
# @brief RT component logger class
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2003-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.

import sys
import traceback
import time
import threading
import logging
import logging.handlers

logger = None


##
# @if jp
#
# @class Logg
#
# @brief ロガーフォーマットダミークラス
#
# ログフォーマット用ダミークラス。
#
# @else
#
# @endif
class Logger:
  """
  """

  SILENT    = 0  # ()
  FATAL     = 41 # (FATAL)
  ERROR     = 40 # (FATAL, ERROR)
  WARN      = 30 # (FATAL, ERROR, WARN)
  INFO      = 20 # (FATAL, ERROR, WARN, INFO)
  DEBUG     = 10 # (FATAL, ERROR, WARN, INFO, DEBUG)
  TRACE     = 9  # (FATAL, ERROR, WARN, INFO, DEBUG, TRACE)
  VERBOSE   = 8  # (FATAL, ERROR, WARN, INFO, DEBUG, TRACE, VERBOSE)
  PARANOID  = 7  # (FATAL, ERROR, WARN, INFO, DEBUG, TRACE, VERBOSE, PARA)


  ##
  # @if jp
  #
  # @brief ログレベル設定
  #
  # 与えられた文字列に対応したログレベルを設定する。
  #
  # @param self
  # @param lv ログレベル文字列
  #
  # @return 設定したログレベル
  #
  # @else
  #
  # @endif
  def strToLogLevel(self, lv):
    if lv == "SILENT":
      return Logger.SILENT
    elif lv == "FATAL":
      return Logger.FATAL
    elif lv == "ERROR":
      return Logger.ERROR
    elif lv == "WARN":
      return Logger.WARN
    elif lv == "INFO":
      return Logger.INFO
    elif lv == "DEBUG":
      return Logger.DEBUG
    elif lv == "TRACE":
      return Logger.TRACE
    elif lv == "VERBOSE":
      return Logger.VERBOSE
    elif lv == "PARANOID":
      return Logger.PARANOID
    else:
      return Logger.INFO



  ##
  # @if jp
  #
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  # @param (mode,file_name,address)
  #
  # @else
  #
  # @brief constructor.
  #
  # @endif
  def __init__(self, *args):
    self._mutex = threading.RLock()
    self._fhdlr = None


  def init(*args):
    global logger

    if logger is not None:
      return logger


    logger = Logger()
    mode = None
    fileName = None

    if len(args) == 0:
      return
    elif len(args) == 2:
      name = args[0]
      mode = args[1]
    elif len(args) == 3:
      name = args[0]
      mode = args[1]
      fileName = args[2]


    logging.PARANOID  = logging.DEBUG - 3
    logging.VERBOSE   = logging.DEBUG - 2
    logging.TRACE     = logging.DEBUG - 1
    logging.FATAL     = logging.ERROR + 1

    logging.addLevelName(logging.PARANOID,  "PARANOID")
    logging.addLevelName(logging.VERBOSE,   "VERBOSE")
    logging.addLevelName(logging.TRACE,     "TRACE")
    logging.addLevelName(logging.FATAL,     "FATAL")

    """
    logging.root.setLevel([logging.NOTSET,
                           logging.PARANOID,
                           logging.VERBOSE,
                           logging.TRACE,
                           logging.DEBUG,
                           logging.INFO,
                           logging.WARNING,
                           logging.ERROR,
                           logging.FATAL,
                           logging.CRITICAL])
    """

    
    formatter = logging.Formatter('%(asctime)s %(name)s %(levelname)s %(message)s')

    if mode is None or mode == "FILE":
      if fileName:
        logger._fhdlr = logging.FileHandler(fileName)
      else:
        logger._fhdlr = logging.FileHandler('rtcsystem.log')

      mhdlr = logging.handlers.MemoryHandler(1024,logging.NOTSET, logger._fhdlr)
      logger._fhdlr.setFormatter(formatter)
      logging.getLogger("").addHandler(mhdlr)
      logging.getLogger("").setLevel(logging.NOTSET)
      
    elif mode == "STDOUT":
      ch = logging.StreamHandler()
      ch.setLevel(logging.NOTSET)
      ch.setFormatter(formatter)
      logging.getLogger("").addHandler(ch)

    return logger

  init = staticmethod(init)



  ##
  # @if jp
  #
  # @brief デストラクタ
  #
  # デストラクタ。ファイルをクローズする。
  #
  # @param self
  #
  # @else
  #
  # @brief destractor.
  #
  # @endif
  def __del__(self):
    #if self._fhdlr is not None:
    #  self._fhdlr.close()
    #  self._fhdler = None
    pass

  ##
  # @if jp
  #
  # @brief printf フォーマット出力
  #
  # printfライクな書式でログ出力する。<br>
  # ※本実装では引数 fmt で与えられた文字をそのまま返す。
  #
  # @param self
  # @param fmt 書式文字列
  #
  # @return 書式付き文字列出力
  #
  # @else
  #
  # @brief Formatted output like printf
  #
  # @endif
  def printf(self, fmt):
    return fmt


  def addHandler(self, *args):
    mode = None
    fileName = None

    if len(args) == 0:
      return
    elif len(args) == 1:
      mode = args[0]
    elif len(args) == 2:
      mode = args[0]
      fileName = args[1]
    
    formatter = logging.Formatter('%(asctime)s %(name)s %(levelname)s %(message)s')

    if mode is None or mode == "FILE":
      if fileName:
        self._fhdlr = logging.FileHandler(fileName)
      else:
        self._fhdlr = logging.FileHandler('rtcsystem.log')

      mhdlr = logging.handlers.MemoryHandler(1024,logging.NOTSET, self._fhdlr)
      self._fhdlr.setFormatter(formatter)
      logging.getLogger("").addHandler(mhdlr)
      
    elif mode.lower() == "stdout":
      ch = logging.StreamHandler()
      ch.setLevel(logging.NOTSET)
      ch.setFormatter(formatter)
      logging.getLogger("").addHandler(ch)


  ##
  # @if jp
  #
  # @brief 例外情報出力
  #  例外情報を文字列で返す。
  #
  # @return 例外情報の文字列出力
  #
  # @else
  #
  # @brief Print exception information 
  # @return Return exception information string.
  #
  # @endif
  def print_exception():
    if sys.version_info[0:3] >= (2, 4, 0):
      return traceback.format_exc()
    else:
      _exc_list = traceback.format_exception(*sys.exc_info())
      _exc_str = "".join(_exc_list)
      return _exc_str
    
  print_exception = staticmethod(print_exception)



##
# @if jp
#
# @class Logg
#
# @brief ロガーフォーマットダミークラス
#
# ログフォーマット用ダミークラス。
#
# @else
#
# @endif
class LogStream:
  """
  """

  ##
  # @if jp
  #
  # @brief コンストラクタ
  #
  # コンストラクタ
  #
  # @param self
  # @param (mode,file_name,address)
  #
  # @else
  #
  # @brief constructor.
  #
  # @endif
  def __init__(self, *args):
    self._LogLock = False
    self._log_enable = False
    self._loggerObj = None
    name = ""

    if len(args) == 0:
      return
    elif len(args) > 0:
      name = args[0]

    self._loggerObj = Logger.init(*args)
    self._log_enable = True
    self.logger = logging.getLogger(name)


  def __del__(self):
    return

  def shutdown(self):
    logging.shutdown()
    return

  def addHandler(self, *args):
    if self._loggerObj is not None:
      self._loggerObj.addHandler(*args)


  ##
  # @if jp
  #
  # @brief ログレベル設定
  #
  # ログレベルを設定する。
  #
  # @param self
  # @param level ログレベル
  #
  # @else
  #
  # @endif
  def setLogLevel(self, level):
    if level == "INFO":
      self.logger.setLevel(logging.INFO)
    elif level == "FATAL":
      self.logger.setLevel(logging.FATAL)
    elif level == "ERROR":
      self.logger.setLevel(logging.ERROR)
    elif level == "WARN":
      self.logger.setLevel(logging.WARNING)
    elif level == "DEBUG":
      self.logger.setLevel(logging.DEBUG)
    elif level == "SILENT":
      self.logger.setLevel(logging.NOTSET)
    elif level == "TRACE":
      self.logger.setLevel(logging.TRACE)
    elif level == "VERBOSE":
      self.logger.setLevel(logging.VERBOSE)
    elif level == "PARANOID":
      self.logger.setLevel(logging.PARANOID)
    else:
      self.logger.setLevel(logging.INFO)


  ##
  # @if jp
  #
  # @brief ロックモード設定
  #
  # ログのロックモードを設定する。
  #
  # @param self
  # @param lock ログロックフラグ
  #
  # @else
  #
  # @endif
  def setLogLock(self, lock):
    if lock == 1:
      self._LogLock = True
    elif lock == 0:
      self._LogLock = False


  ##
  # @if jp
  #
  # @brief ロックモード有効化
  #
  # @param self
  #
  # ロックモードを有効にする。
  #
  # @else
  #
  # @endif
  def enableLogLock(self):
    self._LogLock = True


  ##
  # @if jp
  #
  # @brief ロックモード解除
  #
  # @param self
  #
  # ロックモードを無効にする。
  #
  # @else
  #
  # @endif
  def disableLogLock(self):
    self._LogLock = False


  ##
  # @if jp
  #
  # @brief ログロック取得
  # ロックモードが設定されている場合、ログのロックを取得する。
  #
  # @param self
  #
  # @else
  #
  # @endif
  def acquire(self):
    if self._LogLock:
      self.guard = OpenRTM_aist.ScopedLock(self._mutex)


  ##
  # @if jp
  #
  # @brief ログロック解放
  # ロックモードが設定されている場合に、ログのロックを解放する。
  #
  # @param self
  #
  # @else
  #
  # @endif
  def release(self):
    if self._LogLock:
      del self.guard


  ##
  # @if jp
  #
  # @brief 汎用ログ出力
  #
  # ログレベルおよび出力フォーマット文字列を引数としてとり，
  # 汎用ログを出力する。
  #
  # @param self
  # @param LV ログレベル
  # @param msg ログメッセージ
  # @param opt オプション(デフォルト値:None)
  #
  # @else
  #
  # @brief Log output macro
  #
  # @endif
  def RTC_LOG(self, LV, msg, opt=None):
    if self._log_enable:
      self.acquire()

      if opt is None:
        messages = msg
      else:
        try:
          messages = msg%(opt)
        except:
          print "RTC_LOG : argument error"
          return

      self.logger.log(LV,messages)

      self.release()


  ##
  # @if jp
  #
  # @brief FATALエラーログ出力
  #
  # FATALエラーレベルのログを出力する。<BR>ログレベルが
  # FATAL, ERROR, WARN, INFO, DEBUG, TRACE, VERBOSE, PARANOID
  # の場合にログ出力される。
  #
  # @param self
  # @param msg ログメッセージ
  # @param opt オプション(デフォルト値:None)
  #
  # @else
  #
  # @brief Error log output macro.
  #
  # @endif
  def RTC_FATAL(self, msg, opt=None):
    if self._log_enable:
      self.acquire()

      if opt is None:
        messages = msg
      else:
        try:
          messages = msg%(opt)
        except:
          print "RTC_FATAL : argument error"
          return

      self.logger.log(logging.FATAL,messages)

      self.release()


  ##
  # @if jp
  #
  # @brief エラーログ出力
  #
  # エラーレベルのログを出力する。<BR>ログレベルが
  # ERROR, WARN, INFO, DEBUG, TRACE, VERBOSE, PARANOID
  # の場合にログ出力される。
  #
  # @param self
  # @param msg ログメッセージ
  # @param opt オプション(デフォルト値:None)
  #
  # @else
  #
  # @brief Error log output macro.
  #
  # @endif
  def RTC_ERROR(self, msg, opt=None):
    if self._log_enable:
      self.acquire()

      if opt is None:
        messages = msg
      else:
        try:
          messages = msg%(opt)
        except:
          print "RTC_ERROR : argument error"
          return

      self.logger.error(messages)

      self.release()


  ##
  # @if jp
  #
  # @brief ワーニングログ出力
  #
  # ワーニングレベルのログを出力する。<BR>ログレベルが
  # ( WARN, INFO, DEBUG, TRACE, VERBOSE, PARANOID )
  # の場合にログ出力される。
  #
  # @param self
  # @param msg ログメッセージ
  # @param opt オプション(デフォルト値:None)
  #
  # @else
  #
  # @brief Warning log output macro.
  #
  # If logging levels are
  # ( WARN, INFO, DEBUG, TRACE, VERBOSE, PARANOID ),
  # message will be output to log.
  #
  # @endif
  def RTC_WARN(self, msg, opt=None):
    if self._log_enable:
      self.acquire()

      if opt is None:
        messages = msg
      else:
        try:
          messages = msg%(opt)
        except:
          print "RTC_WARN : argument error"
          return

      self.logger.warning(messages)

      self.release()


  ##
  # @if jp
  #
  # @brief インフォログ出力
  #
  # インフォレベルのログを出力する。<BR>ログレベルが
  # ( INFO, DEBUG, TRACE, VERBOSE, PARANOID )
  # の場合にログ出力される。
  #
  # @param self
  # @param msg ログメッセージ
  # @param opt オプション(デフォルト値:None)
  #
  # @else
  #
  # @brief Infomation level log output macro.
  #
  #  If logging levels are
  # ( INFO, DEBUG, TRACE, VERBOSE, PARANOID ),
  # message will be output to log.
  #
  # @endif
  def RTC_INFO(self, msg, opt=None):
    if self._log_enable:
      self.acquire()

      if opt is None:
        messages = msg
      else:
        try:
          messages = msg%(opt)
        except:
          print "RTC_INFO : argument error"
          return

      self.logger.info(messages)
    
      self.release()


  ##
  # @if jp
  #
  # @brief デバッグログ出力
  #
  # デバッグレベルのログを出力する。<BR>ログレベルが
  # ( DEBUG, TRACE, VERBOSE, PARANOID )
  # の場合にログ出力される。
  #
  # @param self
  # @param msg ログメッセージ
  # @param opt オプション(デフォルト値:None)
  #
  # @else
  #
  # @brief Debug level log output macro.
  #
  # If logging levels are
  # ( DEBUG, TRACE, VERBOSE, PARANOID ),
  # message will be output to log.
  #
  # @endif
  def RTC_DEBUG(self, msg, opt=None):
    if self._log_enable:
      self.acquire()

      if opt is None:
        messages = msg
      else:
        try:
          messages = msg%(opt)
        except:
          print "RTC_DEBUG : argument error"
          return
        
      self.logger.debug(messages)
      
      self.release()


  ##
  # @if jp
  #
  # @brief トレースログ出力
  #
  # トレースレベルのログを出力する。<BR>ログレベルが
  # ( TRACE, VERBOSE, PARANOID )
  # の場合にログ出力される。
  #
  # @param self
  # @param msg ログメッセージ
  # @param opt オプション(デフォルト値:None)
  #
  # @else
  #
  # @brief Trace level log output macro.
  #
  # If logging levels are
  # ( TRACE, VERBOSE, PARANOID ),
  # message will be output to log.
  #
  # @endif
  def RTC_TRACE(self, msg, opt=None):
    if self._log_enable:
      self.acquire()

      if opt is None:
        messages = msg

      else:
        try:
          messages = msg%(opt)
        except:
          print "RTC_TRACE : argument error"
          return

      self.logger.log(logging.TRACE,messages)
    
      self.release()


  ##
  # @if jp
  #
  # @brief ベルボーズログ出力
  #
  # ベルボーズレベルのログを出力する。<BR>ログレベルが
  # ( VERBOSE, PARANOID )
  # の場合にログ出力される。<br>
  # ※現状では未実装
  #
  # @param self
  # @param msg ログメッセージ
  # @param opt オプション(デフォルト値:None)
  #
  # @else
  #
  # @brief Verbose level log output macro.
  #
  # If logging levels are
  # ( VERBOSE, PARANOID ),
  # message will be output to log.
  #
  # @endif
  def RTC_VERBOSE(self, msg, opt=None):
    if self._log_enable:
      self.acquire()

      if opt is None:
        messages = msg
      else:
        try:
          messages = msg%(opt)
        except:
          print "RTC_VERBOSE : argument error"
          return

      self.logger.log(logging.VERBOSE,messages)
    
      self.release()



  ##
  # @if jp
  #
  # @brief パラノイドログ出力
  #
  # パラノイドレベルのログを出力する。<BR>ログレベルが
  # ( PARANOID )
  # の場合にログ出力される。<br>
  # ※現状では未実装
  #
  # @param self
  # @param msg ログメッセージ
  # @param opt オプション(デフォルト値:None)
  #
  # @else
  #
  # @brief Paranoid level log output macro.
  #
  # If logging levels are
  # ( PARANOID ),
  # message will be output to log.
  #
  # @endif
  def RTC_PARANOID(self, msg, opt=None):
    if self._log_enable:
      self.acquire()

      if opt is None:
        messages = msg
      else:
        try:
          messages = msg%(opt)
        except:
          print "RTC_PARANOID : argument error"
          return

      self.logger.log(logging.PARANOID,messages)
    
      self.release()


