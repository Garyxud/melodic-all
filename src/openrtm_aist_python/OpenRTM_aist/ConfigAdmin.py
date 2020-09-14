#!/usr/bin/env python
# -*- coding: euc-jp -*- 

##
# @file ConfigAdmin.py
# @brief Configuration Administration classes
# @date $Date: 2007/09/04$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
# 
# Copyright (C) 2007-2008
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.



import copy
import OpenRTM_aist


class OnUpdateCallback:
  def __init__(self):
    pass


  def __call__(self, config_set):
    pass



class OnUpdateParamCallback:
  def __init__(self):
    pass


  def __call__(self, config_set, config_param):
    pass



class OnSetConfigurationSetCallback:
  def __init__(self):
    pass


  def __call__(self, config_set):
    pass



class OnAddConfigurationAddCallback:
  def __init__(self):
    pass


  def __call__(self, config_set):
    pass



class OnRemoveConfigurationSetCallback:
  def __init__(self):
    pass


  def __call__(self, config_set):
    pass



class OnActivateSetCallback:
  def __init__(self):
    pass


  def __call__(self, config_id):
    pass



##
# @if jp
# @class Config
# @brief Config クラス
# 
# コンフィギュレーションパラメータの情報を保持するクラス。
#
# @since 0.4.0
#
# @else
# @class Config
# @brief Config class
# 
# Class to hold the configuration parameter information.
#
# @since 0.4.0
#
# @endif
class Config:
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
  # @param name コンフィギュレーションパラメータ名
  # @param var コンフィギュレーションパラメータ格納用変数
  # @param def_val 文字列形式のデフォルト値
  # @param trans 文字列形式変換関数(デフォルト値:None)
  # 
  # @else
  #
  # @brief Constructor
  # 
  # Constructor
  #
  # @param self 
  # @param name Configuration parameter name
  # @param var Configuration parameter variable
  # @param def_val Default value in string format
  # @param trans Function to transform into string format
  #
  # @endif
  def __init__(self, name, var, def_val, trans=None):
    self.name = name
    self.default_value = def_val
    self._var = var
    if trans:
      self._trans = trans
    else:
      self._trans = OpenRTM_aist.stringTo


  ##
  # @if jp
  # 
  # @brief バインドパラメータ値を更新
  # 
  # コンフィギュレーション設定値でコンフィギュレーションパラメータを更新する
  # 
  # @param self 
  # @param val パラメータ値の文字列表現
  # 
  # @return 更新処理結果(更新成功:true，更新失敗:false)
  # 
  # @else
  # 
  # @brief Update a bind parameter value
  # 
  # Update configuration paramater by the configuration value.
  #
  # @param self 
  # @param val The parameter values converted into character string format
  #
  # @return Update result (Successful:true, Failed:false)
  # 
  # @endif
  # virtual bool update(const char* val)
  def update(self, val):
    if self._trans(self._var, val):
      return True
    self._trans(self._var, self._default_value)
    return False



##
# @if jp
# @class ConfigAdmin
# @brief ConfigAdmin クラス
# 
# 各種コンフィギュレーション情報を管理するクラス。
# 用語を以下のように定義する。
#
# - コンフィギュレーション: コンポーネントの設定情報。
#
# - (コンフィギュレーション)パラメータ： key-value からなる設定情報。
#   coil::Properties 変数として扱われ、key、value 共に文字列として保
#   持される。key をコンフィギュレーションパラメータ名、value をコン
#   フィギュレーションパラメータ値と呼ぶ。
#
# - コンフィギュレーションセット： コンフィギュレーションパラメータ
#   のリストで、名前 (ID) によって区別される。IDをコンフィギュレーショ
#   ンセットIDと呼ぶ。
#
# - (コンフィギュレーション)パラメータ変数：コンフィギュレーションパ
#   ラメータをRTCのアクティビティ内で実際に利用する際に参照される変
#   数。パラメータごとに固有の型を持つ。
#
# - アクティブ(コンフィギュレーション)セット：現在有効なコンフィギュ
#   レーションセットのことであり、唯一つ存在する。原則として、アクティ
#   ブコンフィギュレーションセットのパラメータがコンフィギュレーショ
#   ンパラメータ変数に反映される。
#
# このクラスでは、コンフィギュレーションのための以下の2つの情報を保
# 持している。
#
# -# コンフィギュレーションセットのリスト
# -# パラメータ変数のリスト
#
# 基本的には、(1) のコンフィギュレーションセットのリストのうち一つを、
# (2) のパラメータ変数へ反映させる、のが本クラスの目的である。通常、
# パラメータ変数の変更操作は、コンフィギュレーションセットの変更とパ
# ラメータ変数への反映の2段階で行われる。
#
# コンフィギュレーションセットのリストの操作には、以下の関数を用いる。
#
# - getConfigurationSets()
# - getConfigurationSet()
# - setConfigurationSetValues()
# - getActiveConfigurationSet()
# - addConfigurationSet()
# - removeConfigurationSet()
# - activateConfigurationSet()
#
# これらの関数により、コンフィギュレーションセットの変更、追加、削除、
# 取得、アクティブ化を行う。これらの操作により変更されたコンフィギュ
# レーションセットを、RTCのアクティビティから使用するパラメータ変数
# に反映させるには、以下の update() 関数を用いる。
#
# - update(void)
# - update(const char* config_set)
# - update(const char* config_set, const char* config_param)
#
# コンフィギュレーション操作をフックするためにコールバックファンクタ
# を与えることができる。フックできる操作は以下の通り。
#
# - ON_UPDATE                   : update() コール時
# - ON_UPDATE_PARAM             : update(param) コール時
# - ON_SET_CONFIGURATIONSET     : setConfigurationSet() コール時
# - ON_ADD_CONFIGURATIONSET     : addConfigurationSet() コール時
# - ON_REMOVE_CONFIGURATIONSET  : removeConfigurationSet() コール時
# - ON_ACTIVATE_CONFIGURATIONSET: activateConfigurationSet() コール時
#
# @since 0.4.0
#
# @else
# @class ConfigAdmin
# @brief ConfigAdmin class
# 
# Class to manage various configuration information.
# Now terms for this class are defined as follows.
#
# - Configurations: The configuration information for the RTCs.
#
# - (Configuration) parameters: Configuration information that
#   consists of a key-value pair. The "key" and the "value" are
#   both stored as character string values in a coil::Properties
#   variable in this class. The "key" is called the "configuration
#   parameter name", and the "value" is called the "configuration
#   parameter value".
#
# - Configuration-sets: This is a list of configuration parameters,
#   and it is distinguished by name (ID). The ID is called
#   configuration-set ID.
#
# - (Configuration) parameter variables: The variables to be
#   referred when configuration parameters are actually used within
#   the activity of an RTC. Each variable has each type.
#
# - Active (configuration) set: This is the only configuration-set
#   that is currently active. The parameter values of the active
#    configuration-set are substituted into configuration variables
#   in principle.
#
# The following two configuration informations are stored in this class.
#
# -# A list of configuration-set
# -# A list of configuration parameter variables
#
# Basically, the purpose of this class is to set one of the
# configuration-set in the list of (1) into parameter variables of
# (2). Usually, configuration parameter variables manipulation is
# performed with two-phases of configuration-set setting and
# parameter variables setting.
#
# The configuration-set manipulations are performed by the
# following functions.
#
# - getConfigurationSets()
# - getConfigurationSet()
# - setConfigurationSetValues()
# - getActiveConfigurationSet()
# - addConfigurationSet()
# - removeConfigurationSet()
# - activateConfigurationSet()
#
# Modification, addition, deletion, acquisition and activation of
# configuration-set are performed by these functions. In order to
# reflect configuration-set, which is manipulated by these
# functions, on parameter variables that are used from RTC
# activities, the following update() functions are used .
#
# - update(void)
# - update(const char* config_set)
# - update(const char* config_set, const char* config_param)
#
# Callback functors can be given to hook configuration
# operation. Operations to be hooked are as follows.
#
# - ON_UPDATE                   : when update() is called
# - ON_UPDATE_PARAM             : when update(param) is called
# - ON_SET_CONFIGURATIONSET     : when setConfigurationSet() is called
# - ON_ADD_CONFIGURATIONSET     : when addConfigurationSet() is called
# - ON_REMOVE_CONFIGURATIONSET  : when removeConfigurationSet() is called
# - ON_ACTIVATE_CONFIGURATIONSET: when activateConfigurationSet() is called
#
# @since 0.4.0
#
# @endif
class ConfigAdmin:
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
  # @param configsets 設定対象プロパティ名
  # 
  # @else
  # 
  # Constructor
  #
  # @param self 
  # @param prop The target property name for setup
  # 
  # @endif
  # ConfigAdmin(coil::Properties& prop);
  def __init__(self, configsets):
    self._configsets = configsets
    self._activeId   = "default"
    self._active     = True
    self._changed    = False
    self._params     = []
    self._emptyconf  = OpenRTM_aist.Properties()
    self._newConfig  = []
    self._listeners  = OpenRTM_aist.ConfigurationListeners()

  ##
  # @if jp
  # 
  # @brief デストラクタ
  # 
  # デストラクタ。
  # 設定されているパラメータを削除する。
  # 
  # @param self 
  # 
  # @else
  # 
  # @brief Destructor
  # 
  # @param self 
  # 
  # @endif
  def __del__(self):
    del self._params


  ##
  # @if jp
  # 
  # @brief コンフィギュレーションパラメータの設定
  # 
  # コンフィギュレーションパラメータと変数をバインドする
  # 指定した名称のコンフィギュレーションパラメータが既に存在する場合は
  # falseを返す。
  # 
  # @param self 
  # @param param_name コンフィギュレーションパラメータ名
  # @param var コンフィギュレーションパラメータ格納用変数
  # @param def_val コンフィギュレーションパラメータデフォルト値
  # @param trans コンフィギュレーションパラメータ文字列変換用関数
  #             (デフォルト値:None)
  # 
  # @return 設定結果(設定成功:true，設定失敗:false)
  # 
  # @else
  # 
  # @brief Setup for configuration parameters
  # 
  # Bind configuration parameter to its variable.
  # Return false, if configuration parameter of specified name has already 
  # existed.
  #
  # @param self 
  # @param param_name Configuration parameter name
  # @param var Configuration parameter variable
  # @param def_val Default value of configuration parameter
  # @param trans Function to transform configuration parameter type into 
  #        string format
  #
  # @return Setup result (Successful:true, Failed:false)
  #
  # 
  # @endif
  #template <typename VarType>
  # bool bindParameter(const char* param_name, VarType& var,
  #                    const char* def_val,
  #                    bool (*trans)(VarType&, const char*) = coil::stringTo)
  def bindParameter(self, param_name, var, def_val, trans=None):
    if trans is None:
      trans = OpenRTM_aist.stringTo
    
    if self.isExist(param_name):
      return False

    if not trans(var, def_val):
      return False
    
    self._params.append(Config(param_name, var, def_val, trans))
    return True


  ##
  # void update(void);
  #
  # @if jp
  #
  # @brief コンフィギュレーションパラメータの更新
  #        (アクティブコンフィギュレーションセット)
  # 
  # コンフィギュレーションセットが更新されている場合に、現在アクティ
  # ブになっているコンフィギュレーションに設定した値で、コンフィギュ
  # レーションパラメータの値を更新する。この処理での更新は、アクティ
  # ブとなっているコンフィギュレーションセットが存在している場合、前
  # 回の更新からコンフィギュレーションセットの内容が更新されている場
  # 合のみ実行される。
  #
  # @else
  #
  # @brief Update the values of configuration parameters
  #        (Active configuration set)
  # 
  # When configuration set is updated, update the configuration
  # parameter value to the value that is set to the current active
  # configuration.  This update will be executed, only when an
  # active configuration set exists and the content of the
  # configuration set has been updated from the last update.
  #
  # @endif
  #
  # void update(const char* config_set);
  #
  # @if jp
  #
  # @brief コンフィギュレーションパラメータの更新(ID指定)
  # 
  # コンフィギュレーション変数の値を、指定したIDを持つコンフィギュレー
  # ションセットの値で更新する。これにより、アクティブなコンフィギュ
  # レーションセットは変更されない。したがって、アクティブコンフィギュ
  # レーションセットとパラメータ変数の間に矛盾が発生する可能性がある
  # ので注意が必要である。
  #
  # 指定したIDのコンフィギュレーションセットが存在しない場合は、何も
  # せずに終了する。
  #
  # @param config_set 設定対象のコンフィギュレーションセットID
  # 
  # @else
  #
  # @brief Update configuration parameter (By ID)
  # 
  # This operation updates configuration variables by the
  # configuration-set with specified ID. This operation does not
  # change current active configuration-set. Since this operation
  # causes inconsistency between current active configuration set
  # and actual values of configuration variables, user should
  # carefully use it.
  #
  # This operation ends without doing anything, if the
  # configuration-set does not exist.
  #
  # @param config_set The target configuration set's ID to setup
  #
  # @endif
  #
  # void update(const char* config_set, const char* config_param);
  #
  # @if jp
  #
  # @brief コンフィギュレーションパラメータの更新(名称指定)
  # 
  # 特定のコンフィギュレーション変数の値を、指定したIDを持つコンフィ
  # ギュレーションセットの値で更新する。これにより、アクティブなコン
  # フィギュレーションセットは変更されない。したがって、アクティブコ
  # ンフィギュレーションセットとパラメータ変数の間に矛盾が発生する可
  # 能性があるので注意が必要である。
  #
  # 指定したIDのコンフィギュレーションセットや、指定した名称のパラメー
  # タが存在しない場合は、何もせずに終了する。
  #
  # @param config_set コンフィギュレーションID
  # @param config_param コンフィギュレーションパラメータ名
  # 
  # @else
  #
  # @brief Update the values of configuration parameters (By name)
  # 
  # This operation updates a configuration variable by the
  # specified configuration parameter in the
  # configuration-set. This operation does not change current
  # active configuration-set. Since this operation causes
  # inconsistency between current active configuration set and
  # actual values of configuration variables, user should carefully
  # use it.
  #
  # This operation ends without doing anything, if the
  # configuration-set or the configuration parameter do not exist.
  #
  # @param config_set configuration-set ID.
  # @param config_param configuration parameter name.
  #
  # @endif
  #
  def update(self, config_set=None, config_param=None):
    # update(const char* config_set)
    if config_set and config_param is None:
      if self._configsets.hasKey(config_set) is None:
        return
      prop = self._configsets.getNode(config_set)
      for i in range(len(self._params)):
        if prop.hasKey(self._params[i].name):
          self._params[i].update(prop.getProperty(self._params[i].name))
          self.onUpdate(config_set)

    # update(const char* config_set, const char* config_param)
    if config_set and config_param:
      key = config_set
      key = key+"."+config_param
      for conf in self._params:
        if conf.name == config_param:
          conf.update(self._configsets.getProperty(key))
          self.onUpdateParam(config_set, config_param)
          return

    # update()
    if config_set is None and config_param is None:
      if self._changed and self._active:
        self.update(self._activeId)
        self._changed = False
      return


  ##
  # @if jp
  # 
  # @brief コンフィギュレーションパラメータの存在確認
  # 
  # 指定した名称を持つコンフィギュレーションパラメータが存在するか確認する。
  # 
  # @param self 
  # @param param_name コンフィギュレーションパラメータ名称。
  # 
  # @return 存在確認結果(パラメータあり:true，パラメータなし:false)
  # 
  # @else
  # 
  # @brief Check the existence of configuration parameters
  # 
  # Check the existence of configuration parameters of specified name.
  #
  # @param self 
  # @param name Configuration parameter name
  #
  # @return Result of existance confirmation 
  #         (Parameters exist:true, else:false)
  # 
  # @endif
  # bool isExist(const char* name);
  def isExist(self, param_name):
    if not self._params:
      return False
    
    for conf in self._params:
      if conf.name == param_name:
        return True

    return False


  ##
  # @if jp
  # 
  # @brief コンフィギュレーションパラメータの変更確認
  # 
  # コンフィギュレーションパラメータが変更されたか確認する。
  # 
  # @param self 
  # 
  # @return 変更確認結果(変更あり:true、変更なし:false)
  # 
  # @else
  # 
  # @brief Confirm to change configuration parameters
  # 
  # Confirm that configuration parameters have changed.
  #
  # @param self 
  #
  # @return Result of change confirmation
  #         (There is a change:true、No change:false)
  # 
  # @endif
  # bool isChanged(void) {return m_changed;}
  def isChanged(self):
    return self._changed


  ##
  # @if jp
  # 
  # @brief アクティブ・コンフィギュレーションセットIDの取得
  # 
  # 現在アクティブなコンフィギュレーションセットのIDを取得する。
  # 
  # @param self 
  # 
  # @return アクティブ・コンフィギュレーションセットID
  # 
  # @else
  # 
  # @brief Get ID of active configuration set
  # 
  # Get ID of the current active configuration set.
  #
  # @param self 
  #
  # @return The active configuration set ID
  # 
  # @endif
  # const char* getActiveId(void);
  def getActiveId(self):
    return self._activeId


  ##
  # @if jp
  # 
  # @brief コンフィギュレーションセットの存在確認
  # 
  # 指定したコンフィギュレーションセットが存在するか確認する。
  # 
  # @param self 
  # @param config_id 確認対象コンフィギュレーションセットID
  # 
  # @return 存在確認結果(指定したConfigSetあり:true、なし:false)
  # 
  # @else
  # 
  # @brief Check the existence of configuration set
  # 
  # Check the existence of specified configuration set.
  #
  # @param self 
  # @param config_id ID of target configuration set for confirmation
  # @return Result of existence confirmation 
  #         (Specified ConfigSet exists:true, else:false)
  # @endif
  # bool haveConfig(const char* config_id);
  def haveConfig(self, config_id):
    if self._configsets.hasKey(config_id) is None:
      return False
    else:
      return True


  ##
  # @if jp
  # 
  # @brief コンフィギュレーションセットのアクティブ化確認
  # 
  # コンフィギュレーションセットがアクティブ化されているか確認する。
  # 
  # @param self 
  # 
  # @return 状態確認結果(アクティブ状態:true、非アクティブ状態:false)
  # 
  # @else
  # 
  # @brief Confirm to activate configuration set
  # 
  # Confirm that configuration set has been activated.
  #
  # @param self 
  #
  # @return Result of state confirmation
  #         (Active state:true, Inactive state:false)
  # 
  # @endif
  # bool isActive(void);
  def isActive(self):
    return self._active


  ##
  # @if jp
  # 
  # @brief 全コンフィギュレーションセットの取得
  # 
  # 設定されている全コンフィギュレーションセットを取得する。
  # 
  # @param self 
  # 
  # @return 全コンフィギュレーションセット
  # 
  # @else
  # 
  # @brief Get all configuration sets
  # 
  # Get all specified configuration sets
  #
  # @param self 
  #
  # @return All configuration sets
  # 
  # @endif
  # const std::vector<coil::Properties*>& getConfigurationSets(void);
  def getConfigurationSets(self):
    return self._configsets.getLeaf()


  ##
  # @if jp
  # 
  # @brief 指定したIDのコンフィギュレーションセットの取得
  # 
  # IDで指定したコンフィギュレーションセットを取得する。
  # 指定したコンフィギュレーションセットが存在しない場合は、
  # 空のコンフィギュレーションセットを返す。
  # 
  # @param self 
  # @param config_id 取得対象コンフィギュレーションセットのID
  # 
  # @return コンフィギュレーションセット
  # 
  # @else
  # 
  # @brief Get a configuration set by specified ID
  # 
  # Get a configuration set that was specified by ID
  # Return empty configuration set, if a configuration set of
  # specified ID doesn't exist.
  #
  # @param self 
  # @param config_id ID of the target configuration set for getting
  #
  # @return The configuration set
  # 
  # @endif
  # const coil::Properties& getConfigurationSet(const char* config_id);
  def getConfigurationSet(self, config_id):
    prop = self._configsets.getNode(config_id)
    if prop is None:
      return self._emptyconf
    return prop


  ##
  # @if jp
  # 
  # @brief 指定したプロパティのコンフィギュレーションセットへの追加
  # 
  # 指定したプロパティをコンフィギュレーションセットへ追加する。
  # 
  # @param self 
  # @param config_set 追加するプロパティ
  # 
  # @return 追加処理実行結果(追加成功:true、追加失敗:false)
  # 
  # @else
  # 
  # @brief Add to configuration set from specified property
  # 
  # Add specified property to configuration set.
  #
  # @param self 
  # @param configuration_set Property to add
  #
  # @return Add result (Successful:true, Failed:false)
  # 
  # @endif
  # bool setConfigurationSetValues(const coil::Properties& config_set)
  def setConfigurationSetValues(self, config_set):
    if config_set.getName() == "" or config_set.getName() is None:
      return False

    if not self._configsets.hasKey(config_set.getName()):
      return False

    p = self._configsets.getNode(config_set.getName())
    if p is None:
      return False

    p.mergeProperties(config_set)
    self._changed = True
    self._active  = False
    self.onSetConfigurationSet(config_set)
    return True


  ##
  # @if jp
  # 
  # @brief アクティブ・コンフィギュレーションセットを取得
  # 
  # 現在アクティブとなっているコンフィギュレーションセットを取得する。
  # アクティブとなっているコンフィギュレーションセットが存在しない場合は、
  # 空のコンフィギュレーションセット を返す。
  # 
  # @param self 
  # 
  # @return アクティブ・コンフィギュレーションセット
  # 
  # @else
  # 
  # @brief Get the active configuration set
  # 
  # Get the current active configuration set.
  # Return empty configuration set, if an active configuration set 
  # doesn't exist.
  #
  # @param self 
  # @return The active configuration set
  # 
  # @endif
  # const coil::Properties& getActiveConfigurationSet(void);
  def getActiveConfigurationSet(self):
    p = self._configsets.getNode(self._activeId)
    if p is None:
      return self._emptyconf

    return p


  ##
  # @if jp
  # 
  # @brief コンフィギュレーションセットに設定値を追加
  # 
  # コンフィギュレーションセットに設定値を追加する。
  # 
  # @param self 
  # @param configset 追加するプロパティ
  # 
  # @return 追加処理結果(追加成功:true、追加失敗:false)
  # 
  # @else
  # 
  # @brief Add the configuration value to configuration set
  # 
  # Add the configuration value to configuration set
  #
  # @param self 
  # @param configuration_set Property to add
  #
  # @return Add Result (Successful:true, Failed:false)
  # 
  # @endif
  # bool addConfigurationSet(const coil::Properties& configuration_set);
  def addConfigurationSet(self, configset):
    if self._configsets.hasKey(configset.getName()):
      return False
    node = configset.getName()

    # Create node
    self._configsets.createNode(node)

    p = self._configsets.getNode(node)
    if p is None:
      return False

    p.mergeProperties(configset)
    self._newConfig.append(node)

    self._changed = True
    self._active  = False
    self.onAddConfigurationSet(configset)
    return True


  ##
  # @if jp
  #
  # @brief コンフィギュレーションセットの削除
  # 
  # 指定したIDのコンフィギュレーションセットを削除する。
  #
  # 指定したIDのコンフィギュレーションセットが存在しない場合は、
  # falseを返す。削除可能なコンフィギュレーションセットは、
  # addConfigruationSet() によって追加したコンフィギュレーションセッ
  # トのみであり、デフォルトコンフィギュレーションセット、コンポーネ
  # ント起動時にファイルから読み込まれるコンフィギュレーションセット
  # は削除することができない。
  #
  # また、指定したコンフィギュレーションセットが現在アクティブである
  # 場合には、いかなるコンフィギュレーションセットでも削除できない。
  #
  # この関数により実際にコンフィギュレーションセットが削除された場合、
  # setOnRemoveConfigurationSet() でセットされたコールバック関数が呼
  # び出される。
  #
  # @param self 
  # @param config_id 削除対象コンフィギュレーションセットのID
  #
  # @return 削除処理結果(削除成功:true、削除失敗:false)
  #
  # @else
  #
  # @brief Remove the configuration set
  # 
  # Remove the configuration set of specified ID Return empty
  # configuration set, if a configuration set of specified ID
  # doesn't exist.
  #
  # The configuration-sets that can be removed by this function are
  # only configuration-sets newly added by the
  # addConfigurationSet() function. The configuration that can be
  # removed by this function is only newly added configuration-set
  # by addConfigurationSet() function.  The "default"
  # configuration-set and configurationi-sets that is loaded from
  # configuration file cannot be removed.
  #
  # If the specified configuration is active currently, any
  # configurations are not deleted.
  #
  # Callback functions that are set by
  # addOnRemovedConfigurationSet() will be called if a
  # configuration-set is deleted actually by this function.
  #
  # @param self 
  # @param config_id ID of the target configuration set for remove
  #
  # @return Remove result (Successful:true, Failed:false)
  #
  # @endif
  #
  # bool removeConfigurationSet(const char* config_id);
  def removeConfigurationSet(self, config_id):
    if config_id == "default":
      return False
    if self._activeId == config_id:
      return False

    find_flg = False
    # removeable config-set is only config-sets newly added
    for (idx,conf) in enumerate(self._newConfig):
      if conf == config_id:
        find_flg = True
        break


    if not find_flg:
      return False

    p = self._configsets.getNode(config_id)
    if p:
      p.getRoot().removeNode(config_id)
      del p

    del self._newConfig[idx]

    self._changed = True
    self._active  = False
    self.onRemoveConfigurationSet(config_id)
    return True


  ##
  # @if jp
  # 
  # @brief コンフィギュレーションセットのアクティブ化
  # 
  # 指定したIDのコンフィギュレーションセットをアクティブ化する。
  # 指定したIDのコンフィギュレーションセットが存在しない場合は、
  # falseを返す。
  # 
  # @param self 
  # @param config_id 削除対象コンフィギュレーションセットのID
  # 
  # @return アクティブ処理結果(成功:true、失敗:false)
  # 
  # @else
  # 
  # @brief Activate the configuration set
  # 
  # Activate the configuration set of specified ID
  # Return empty configuration set, if a configuration set of
  # specified ID doesn't exist.
  #
  # @param self 
  # @param config_id ID of the target configuration set for remove
  #
  # @return Activate result (Remove success:true、Remove failure:false)
  # 
  # @endif
  # bool activateConfigurationSet(const char* config_id);
  def activateConfigurationSet(self, config_id):
    if config_id is None:
      return False

    # '_<conf_name>' is special configuration set name
    if config_id[0] == '_':
      return False

    if not self._configsets.hasKey(config_id):
      return False
    self._activeId = config_id
    self._active   = True
    self._changed  = True
    self.onActivateSet(config_id)
    return True


  #------------------------------------------------------------
  # obsolete functions
  #

  ##
  # @if jp
  #
  # @brief OnUpdate のコールバックの設定
  #
  # OnUpdate で呼ばれるコールバックのオブジェクトを設定する。
  # 
  # @param self 
  # @param cb OnUpdateCallback型のオブジェクト
  #
  # @else
  #
  # @brief Set callback that is called by OnUpdate. 
  # 
  # @param self 
  # @param cb OnUpdateCallback type object
  #
  # @endif
  #
  # void setOnUpdate(OnUpdateCallback* cb);
  def setOnUpdate(self, cb):
    print "setOnUpdate function is obsolete."
    print "Use addConfigurationSetNameListener instead."
    self._listeners.configsetname_[OpenRTM_aist.ConfigurationSetNameListenerType.ON_UPDATE_CONFIG_SET].addListener(cb, False)
    return


  ##
  # @if jp
  #
  # @brief OnUpdateParam のコールバックの設定
  #
  # OnUpdateParam で呼ばれるコールバックのオブジェクトを設定する。
  # 
  # @param self 
  # @param cb OnUpdateParamCallback型のオブジェクト
  #
  # @else
  #
  # @brief Set callback that is called by OnUpdateParam. 
  # 
  # @param self 
  # @param cb OnUpdateParamCallback type object
  #
  # @endif
  #
  # void setOnUpdateParam(OnUpdateParamCallback* cb);
  def setOnUpdateParam(self, cb):
    print "setOnUpdateParam function is obsolete."
    print "Use addConfigurationParamListener instead."
    self._listeners.configparam_[OpenRTM_aist.ConfigurationParamListenerType.ON_UPDATE_CONFIG_PARAM].addListener(cb, False)
    return


  ##
  # @if jp
  #
  # @brief OnSetConfigurationSet のコールバックの設定
  #
  # OnSetConfigurationSet で呼ばれるコールバックのオブジェクトを設定する。
  # 
  # @param self 
  # @param cb OnSetConfigurationSetCallback型のオブジェクト
  #
  # @else
  #
  # @brief Set callback that is called by OnSetConfiguration. 
  # 
  # @param self 
  # @param cb OnSetConfigurationSetCallback type object
  #
  # @endif
  #
  # void setOnSetConfigurationSet(OnSetConfigurationSetCallback* cb);
  def setOnSetConfigurationSet(self, cb):
    print "setOnSetConfigurationSet function is obsolete."
    print "Use addConfigurationSetListener instead."
    self._listeners.configset_[OpenRTM_aist.ConfigurationSetListenerType.ON_SET_CONFIG_SET].addListener(cb, False)
    return


  ##
  # @if jp
  #
  # @brief OnAddConfigurationSet のコールバックの設定
  #
  # OnAddConfigurationSet で呼ばれるコールバックのオブジェクトを設定する。
  # 
  # @param self 
  # @param cb OnAddConfigurationAddCallback型のオブジェクト
  #
  # @else
  #
  # @brief Set callback that is called by OnSetConfiguration. 
  # 
  # @param self 
  # @param cb OnSetConfigurationSetCallback type object
  #
  # @endif
  #
  # void setOnAddConfigurationSet(OnAddConfigurationAddCallback* cb);
  def setOnAddConfigurationSet(self, cb):
    print "setOnAddConfigurationSet function is obsolete."
    print "Use addConfigurationSetListener instead."
    self._listeners.configset_[OpenRTM_aist.ConfigurationSetListenerType.ON_ADD_CONFIG_SET].addListener(cb, False)
    return


  ##
  # @if jp
  #
  # @brief OnRemoveConfigurationSet のコールバックの設定
  #
  # OnRemoveConfiguration で呼ばれるコールバックのオブジェクトを設定する。
  # 
  # @param self 
  # @param cb OnRemoveConfigurationSetCallback型のオブジェクト
  #
  # @else
  #
  # @brief Set callback that is called by OnRemoveConfigurationSet. 
  # 
  # @param self 
  # @param cb OnRemoveConfigurationSetCallback type object
  #
  # @endif
  #
  # void setOnRemoveConfigurationSet(OnRemoveConfigurationSetCallback* cb);
  def setOnRemoveConfigurationSet(self, cb):
    print "setOnRemoveConfigurationSet function is obsolete."
    print "Use addConfigurationSetNameListener instead."
    self._listeners.configsetname_[OpenRTM_aist.ConfigurationSetNameListenerType.ON_REMOVE_CONFIG_SET].addListener(cb, False)
    return


  ##
  # @if jp
  #
  # @brief OnActivateSet のコールバックの設定
  #
  # OnActivateSet で呼ばれるコールバックのオブジェクトを設定する。
  # 
  # @param self 
  # @param cb OnActivateSetCallback型のオブジェクト
  #
  # @else
  #
  # @brief Set callback that is called by OnActivateSet. 
  # 
  # @param self 
  # @param cb OnActivateSetCallback type object
  #
  # @endif
  #
  # void setOnActivateSet(OnActivateSetCallback* cb);
  def setOnActivateSet(self, cb):
    print "setOnActivateSet function is obsolete."
    print "Use addConfigurationSetNameListener instead."
    self._listeners.configsetname_[OpenRTM_aist.ConfigurationSetNameListenerType.ON_ACTIVATE_CONFIG_SET].addListener(cb, False)
    return

  #
  # end of obsolete functions
  #------------------------------------------------------------

  ##
  # @if jp
  #
  # @brief ConfigurationParamListener を追加する
  #
  # update(const char* config_set, const char* config_param) が呼ばれた際に
  # コールされるリスナ ConfigurationParamListener を追加する。
  # type には現在のところ ON_UPDATE_CONFIG_PARAM のみが入る。
  #
  # @param type ConfigurationParamListenerType型の値。
  #             ON_UPDATE_CONFIG_PARAM がある。
  #
  # @param listener ConfigurationParamListener 型のリスナオブジェクト。
  # @param autoclean リスナオブジェクトを自動で削除するかどうかのフラグ
  # 
  # @else
  #
  # @brief Adding ConfigurationParamListener 
  # 
  # This function adds a listener object which is called when
  # update(const char* config_set, const char* config_param) is
  # called. In the type argument, currently only
  # ON_UPDATE_CONFIG_PARAM is allowed.
  #
  # @param type ConfigurationParamListenerType value
  #             ON_UPDATE_CONFIG_PARAM is only allowed.
  #
  # @param listener ConfigurationParamListener listener object.
  # @param autoclean a flag whether if the listener object autocleaned.
  #
  # @endif
  #
  # void addConfigurationParamListener(ConfigurationParamListenerType type,
  #                                    ConfigurationParamListener* listener,
  #                                    bool autoclean = true);
  def addConfigurationParamListener(self, type, listener, autoclean = True):
    self._listeners.configparam_[type].addListener(listener, autoclean)
    return


  ##
  # @if jp
  #
  # @brief ConfigurationParamListener を削除する
  #
  # addConfigurationParamListener で追加されたリスナオブジェクトを削除する。
  #
  # @param type ConfigurationParamListenerType型の値。
  #             ON_UPDATE_CONFIG_PARAM がある。
  # @param listener 与えたリスナオブジェクトへのポインタ
  # 
  # @else
  #
  # @brief Removing ConfigurationParamListener 
  # 
  # This function removes a listener object which is added by
  # addConfigurationParamListener() function.
  #
  # @param type ConfigurationParamListenerType value
  #             ON_UPDATE_CONFIG_PARAM is only allowed.
  # @param listener a pointer to ConfigurationParamListener listener object.
  #
  # @endif
  #
  # void removeConfigurationParamListener(ConfigurationParamListenerType type,
  #                                       ConfigurationParamListener* listener);
  def removeConfigurationParamListener(self, type, listener):
    self._listeners.configparam_[type].removeListener(listener)
    return
    

  ##
  # @if jp
  #
  # @brief ConfigurationSetListener を追加する
  #
  # ConfigurationSet が更新されたときなどに呼ばれるリスナ
  # ConfigurationSetListener を追加する。設定可能なイベントは以下の
  # 2種類がある。
  #
  # - ON_SET_CONFIG_SET: setConfigurationSetValues() で
  #                      ConfigurationSet に値が設定された場合。
  # - ON_ADD_CONFIG_SET: addConfigurationSet() で新しい
  #                      ConfigurationSet が追加された場合。
  #
  # @param type ConfigurationSetListenerType型の値。
  # @param listener ConfigurationSetListener 型のリスナオブジェクト。
  # @param autoclean リスナオブジェクトを自動で削除するかどうかのフラグ
  # 
  # @else
  #
  # @brief Adding ConfigurationSetListener 
  # 
  # This function add a listener object which is called when
  # ConfigurationSet is updated. Available events are the followings.
  #
  # @param type ConfigurationSetListenerType value
  # @param listener ConfigurationSetListener listener object.
  # @param autoclean a flag whether if the listener object autocleaned.
  #
  # @endif
  #
  # void addConfigurationSetListener(ConfigurationSetListenerType type,
  #                                  ConfigurationSetListener* listener,
  #                                  bool autoclean = true);
  def addConfigurationSetListener(self, type, listener, autoclean = True):
    self._listeners.configset_[type].addListener(listener, autoclean)
    return


  ##
  # @if jp
  #
  # @brief ConfigurationSetListener を削除する
  #
  # addConfigurationSetListener で追加されたリスナオブジェクトを削除する。
  #
  # @param type ConfigurationSetListenerType型の値。
  # @param listener 与えたリスナオブジェクトへのポインタ
  # 
  # @else
  #
  # @brief Removing ConfigurationSetListener 
  # 
  # This function removes a listener object which is added by
  # addConfigurationSetListener() function.
  #
  # @param type ConfigurationSetListenerType value
  # @param listener a pointer to ConfigurationSetListener listener object.
  #
  # @endif
  # void removeConfigurationSetListener(ConfigurationSetListenerType type,
  #                                     ConfigurationSetListener* listener);
  def removeConfigurationSetListener(self, type, listener):
    self._listeners.configset_[type].removeListener(listener)
    return
    

  ##
  # @if jp
  #
  # @brief ConfigurationSetNameListener を追加する
  #
  # ConfigurationSetName が更新されたときなどに呼ばれるリスナ
  # ConfigurationSetNameListener を追加する。設定可能なイベントは以下の
  # 3種類がある。
  #
  # - ON_UPDATE_CONFIG_SET: ある ConfigurationSet がアップデートされた
  # - ON_REMOVE_CONFIG_SET: ある ConfigurationSet が削除された
  # - ON_ACTIVATE_CONFIG_SET: ある ConfigurationSet がアクティブ化された
  #
  # @param type ConfigurationSetNameListenerType型の値。
  # @param listener ConfigurationSetNameListener 型のリスナオブジェクト。
  # @param autoclean リスナオブジェクトを自動で削除するかどうかのフラグ
  # 
  # @else
  #
  # @brief Adding ConfigurationSetNameListener 
  # 
  # This function add a listener object which is called when
  # ConfigurationSetName is updated. Available events are the followings.
  #
  # - ON_UPDATE_CONFIG_SET: A ConfigurationSet has been updated.
  # - ON_REMOVE_CONFIG_SET: A ConfigurationSet has been deleted.
  # - ON_ACTIVATE_CONFIG_SET: A ConfigurationSet has been activated.
  #
  # @param type ConfigurationSetNameListenerType value
  # @param listener ConfigurationSetNameListener listener object.
  # @param autoclean a flag whether if the listener object autocleaned.
  #
  # @endif
  # void 
  # addConfigurationSetNameListener(ConfigurationSetNameListenerType type,
  #                                 ConfigurationSetNameListener* listener,
  #                                 bool autoclean = true);
  def addConfigurationSetNameListener(self, type, listener, autoclean = True):
    self._listeners.configsetname_[type].addListener(listener, autoclean)
    return


  ##
  # @if jp
  #
  # @brief ConfigurationSetNameListener を削除する
  #
  # addConfigurationSetNameListener で追加されたリスナオブジェクトを
  # 削除する。
  #
  # @param type ConfigurationSetNameListenerType型の値。
  #             ON_UPDATE_CONFIG_PARAM がある。
  # @param listener 与えたリスナオブジェクトへのポインタ
  # 
  # @else
  #
  # @brief Removing ConfigurationSetNameListener 
  # 
  # This function removes a listener object which is added by
  # addConfigurationSetNameListener() function.
  #
  # @param type ConfigurationSetNameListenerType value
  #             ON_UPDATE_CONFIG_PARAM is only allowed.
  # @param listener a pointer to ConfigurationSetNameListener
  #             listener object.
  #
  # @endif
  # void
  # removeConfigurationSetNameListener(ConfigurationSetNameListenerType type,
  #                                    ConfigurationSetNameListener* listener);
  def removeConfigurationSetNameListener(self, type, listener):
    self._listeners.configsetname_[type].removeListener(listener)
    return
    

  ##
  # @if jp
  #
  # @brief コンフィギュレーションパラメータの更新(ID指定)時にコールされる
  #
  # 設定されてるコールバックオブジェクトを呼び出す。
  #
  # @param self 
  # @param config_set 設定対象のコンフィギュレーションセットID
  #
  # @else
  #
  # @brief When the configuration parameter is updated, it is called. 
  #
  # Call the set callback object.
  # 
  # @param self 
  # @param config_set The target configuration set's ID to setup
  #
  # @endif
  #
  # void onUpdate(const char* config_set);
  def onUpdate(self, config_set):
    self._listeners.configsetname_[OpenRTM_aist.ConfigurationSetNameListenerType.ON_UPDATE_CONFIG_SET].notify(config_set)
    return


  ##
  # @if jp
  #
  # @brief コンフィギュレーションパラメータの更新(名称指定)時にコールされる
  #
  # 設定されてるコールバックオブジェクトを呼び出す。
  #
  # @param self 
  # @param config_set コンフィギュレーションID
  # @param config_param コンフィギュレーションパラメータ名
  #
  # @else
  #
  # @brief When the configuration parameter is updated, it is called. 
  #
  # Call the set callback object.
  # 
  # @param self 
  # @param config_set configuration-set ID.
  # @param config_param configuration parameter name.
  #
  # @endif
  #
  # void onUpdateParam(const char* config_set, const char* config_param);
  def onUpdateParam(self, config_set, config_param):
    self._listeners.configparam_[OpenRTM_aist.ConfigurationParamListenerType.ON_UPDATE_CONFIG_PARAM].notify(config_set,
                                                                                                            config_param)
    return


  ##
  # @if jp
  #
  # @brief コンフィギュレーションセットへの追加時にコールされる
  #
  # 設定されてるコールバックオブジェクトを呼び出す。
  #
  # @param self 
  # @param configuration_set プロパティ
  #
  # @else
  #
  # @brief Called when the property is added to the configuration set
  #
  # Call the set callback object.
  # 
  # @param self 
  # @param configuration_set property
  #
  # @endif
  #
  # void onSetConfigurationSet(const coil::Properties& config_set);
  def onSetConfigurationSet(self, config_set):
    self._listeners.configset_[OpenRTM_aist.ConfigurationSetListenerType.ON_SET_CONFIG_SET].notify(config_set)
    return


  ##
  # @if jp
  #
  # @brief 設定値が追加されたときにコールされる。
  #
  # 設定されてるコールバックオブジェクトを呼び出す。
  #
  # @param self 
  # @param configuration_set プロパティ
  #
  # @else
  #
  # @brief Called when a set value is added to the configuration set
  #
  # Call the set callback object.
  # 
  # @param self 
  # @param configuration_set property
  #
  # @endif
  #
  # void onAddConfigurationSet(const coil::Properties& config_set);
  def onAddConfigurationSet(self, config_set):
    self._listeners.configset_[OpenRTM_aist.ConfigurationSetListenerType.ON_ADD_CONFIG_SET].notify(config_set)
    return


  ##
  # @if jp
  #
  # @brief セットが削除されてるときにコールされる。
  #
  # 設定されてるコールバックオブジェクトを呼び出す。
  #
  # @param self 
  # @param config_id プロパティ
  #
  # @else
  #
  # @brief Called when the configuration set has been deleted
  #
  # Call the set callback object.
  # 
  # @param self 
  # @param config_id property
  #
  # @endif
  #
  # void onRemoveConfigurationSet(const char* config_id);
  def onRemoveConfigurationSet(self, config_id):
    self._listeners.configsetname_[OpenRTM_aist.ConfigurationSetNameListenerType.ON_REMOVE_CONFIG_SET].notify(config_id)
    return


  ##
  # @if jp
  #
  # @brief セットがアクティブ化されたときにコールされる。
  #
  # 設定されてるコールバックオブジェクトを呼び出す。
  #
  # @param self 
  # @param config_id プロパティ
  #
  # @else
  #
  # @brief Called when the configuration set is made active
  #
  # Call the set callback object.
  # 
  # @param self 
  # @param config_id property
  #
  # @endif
  #
  # void onActivateSet(const char* config_id);
  def onActivateSet(self, config_id):
    self._listeners.configsetname_[OpenRTM_aist.ConfigurationSetNameListenerType.ON_ACTIVATE_CONFIG_SET].notify(config_id)
    return


  class find_conf:
    def __init__(self, name):
      self._name = name
      return

    def __call__(self, conf):
      if conf is None or conf is 0:
        return False

      return self._name == conf.name
