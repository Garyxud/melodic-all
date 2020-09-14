// -*- C++ -*-
/*!
 * @file ConfigAdmin.cpp
 * @brief Configuration Administration classes
 * @date $Date: 2007-12-31 03:08:02 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2007-2011
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#include <rtm/ConfigAdmin.h>
#include <algorithm>
#include <assert.h>
#include <string.h>

namespace RTC
{
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @endif
   */
  ConfigAdmin::ConfigAdmin(coil::Properties& configsets)
    : m_configsets(configsets), m_activeId("default"),
      m_active(true), m_changed(false)
  {
  }
  
  /*!
   * @if jp
   * @brief 仮想デストラクタ
   * @else
   * @brief Virtual destructor
   * @endif
   */
  ConfigAdmin::~ConfigAdmin(void)
  {
    for (int i(0), len(m_params.size()); i < len; ++i)
      {
	if (m_params[i] != NULL) { delete m_params[i]; }
      }
    m_params.clear();
  }
  

  /*!
   * @if jp
   * @brief コンフィギュレーションパラメータの更新
   *        (アクティブコンフィギュレーションセット)
   * @else
   * @brief Update the values of configuration parameters
   *        (Active configuration set)
   * @endif
   */
  void ConfigAdmin::update(void)
  {
    if (m_changed && m_active)
      {
	update(m_activeId.c_str());
	m_changed = false;
      }
    return;
  }


  /*!
   * @if jp
   * @brief コンフィギュレーションパラメータの更新(ID指定)
   * @else
   * @brief Update configuration parameter (By ID)
   * @endif
   */
  void ConfigAdmin::update(const char* config_set)
  {
    if (m_configsets.hasKey(config_set) == NULL) { return; }
    
    coil::Properties& prop(m_configsets.getNode(config_set));
    
    for (int i(0), len(m_params.size()); i < len; ++i)
      {
	if (prop.hasKey(m_params[i]->name) != NULL)
	  {
	    m_params[i]->update(prop[m_params[i]->name].c_str());
            onUpdate(config_set);
	  }
      }
  }


  /*!
   * @if jp
   * @brief コンフィギュレーションパラメータの更新(名称指定)
   * @else
   * @brief Update the values of configuration parameters (By name)
   * @endif
   */
  void ConfigAdmin::update(const char* config_set, const char* config_param)
  {
    if ((config_set == 0) || (config_param == 0)) { return; }

    std::string key(config_set);
    key += "."; key += config_param;
    
    std::vector<ConfigBase*>::iterator it;
    it = std::find_if(m_params.begin(), m_params.end(),
		      find_conf(config_param));
    if (it != m_params.end())
      {
	(*it)->update(m_configsets[key].c_str());
        onUpdateParam(config_set, config_param);
	return;
      }
  }


  /*!
   * @if jp
   * @brief コンフィギュレーションパラメータの存在確認
   * @else
   * @brief Check the existence of configuration parameters
   * @endif
   */
  bool ConfigAdmin::isExist(const char* param_name)
  {
    std::vector<ConfigBase*>::iterator it;
    it = std::find_if(m_params.begin(), m_params.end(),
		      find_conf(param_name));
    if (it != m_params.end())
      {
	return true;
      }
    return false;
  }
  
  
  /*!
   * @if jp
   * @brief 全コンフィギュレーションセットの取得
   * @else
   * @brief Get all configuration sets
   * @endif
   */
  const std::vector<coil::Properties*>& ConfigAdmin::getConfigurationSets(void)
  {
    return m_configsets.getLeaf();
  }
  
  /*!
   * @if jp
   * @brief 指定したIDのコンフィギュレーションセットの取得
   * @else
   * @brief Get a configuration set by specified ID
   * @endif
   */
  const coil::Properties&
  ConfigAdmin::getConfigurationSet(const char* config_id)
  {
    coil::Properties* p(m_configsets.findNode(config_id));
    if (p == 0) { return m_emptyconf; }
    return *p;
  }
  
  /*!
   * @if jp
   * @brief 指定したプロパティのコンフィギュレーションセットへの追加
   * @else
   * @brief Add to the configuration set from specified property
   * @endif
   */
  bool
  ConfigAdmin::setConfigurationSetValues(const coil::Properties& config_set)
  {
    std::string node(config_set.getName());
    if (node.empty()) { return false; }
    
    coil::Properties& p(m_configsets.getNode(config_set.getName()));

    p << config_set;
    m_changed = true;
    m_active = false;
    onSetConfigurationSet(config_set);
    return true;
  }
  
  /*!
   * @if jp
   * @brief アクティブ・コンフィギュレーションセットを取得
   * @else
   * @brief Get the active configuration set
   * @endif
   */
  const coil::Properties& ConfigAdmin::getActiveConfigurationSet(void)
  {
    coil::Properties& p(m_configsets.getNode(m_activeId));

    return p;
  }
  
  /*!
   * @if jp
   * @brief コンフィギュレーションセットに設定値を追加
   * @else
   * @brief Add the configuration value to the configuration set
   * @endif
   */
  bool ConfigAdmin::addConfigurationSet(const coil::Properties& config_set)
  {
    std::string node(config_set.getName());
    if (node.empty()) { return false; }
    if (m_configsets.hasKey(node.c_str()) != 0) { return false; }
    
    coil::Properties& p(m_configsets.getNode(node));
    p << config_set;
    m_newConfig.push_back(node);
    
    m_changed = true;
    m_active = false;
    onAddConfigurationSet(config_set);
    return true;
  }
  
  /*!
   * @if jp
   * @brief コンフィギュレーションセットの削除
   * @else
   * @brief Remove the configuration set
   * @endif
   */
  bool ConfigAdmin::removeConfigurationSet(const char* config_id)
  {
    if (strcmp(config_id, "default") == 0) return false;
    if (m_activeId == config_id) return false;

    // removeable config-set is only config-sets newly added
    std::vector<std::string>::iterator it;
    it = std::find(m_newConfig.begin(), m_newConfig.end(), config_id);
    if (it == m_newConfig.end()) { return false; }
    
    coil::Properties* p(m_configsets.removeNode(config_id));
    if (p != NULL) { delete p; }
    m_newConfig.erase(it);
    
    m_changed = true;
    m_active = false;
    onRemoveConfigurationSet(config_id);
    return true;
  }
  
  /*!
   * @if jp
   * @brief コンフィギュレーションセットのアクティブ化
   * @else
   * @brief Activate the configuration set
   * @endif
   */
  bool ConfigAdmin::activateConfigurationSet(const char* config_id)
  {
    if (config_id == NULL) { return false; }
    // '_<conf_name>' is special configuration set name
    if (config_id[0] == '_') { return false; }

    if (m_configsets.hasKey(config_id) == 0) { return false; }
    m_activeId = config_id;
    m_active = true;
    m_changed = true;
    onActivateSet(config_id);
    return true;
  }

  //------------------------------------------------------------
  // obsolete functions
  //
  void ConfigAdmin::setOnUpdate(OnUpdateCallback* cb)
  {
    std::cerr << "setOnUpdate function is obsolete." << std::endl;
    std::cerr << "Use addConfigurationSetNameListener instead." << std::endl;
    m_listeners.configsetname_[ON_UPDATE_CONFIG_SET].addListener(cb, false);
  }

  void ConfigAdmin::setOnUpdateParam(OnUpdateParamCallback* cb)
  {
    std::cerr << "setOnUpdateParam function is obsolete." << std::endl;
    std::cerr << "Use addConfigurationParamListener instead." << std::endl;
    m_listeners.configparam_[ON_UPDATE_CONFIG_PARAM].addListener(cb, false);
  }

  void ConfigAdmin::setOnSetConfigurationSet(OnSetConfigurationSetCallback* cb)
  {
    std::cerr << "setOnSetConfigurationSet function is obsolete." << std::endl;
    std::cerr << "Use addConfigurationSetListener instead." << std::endl;
    m_listeners.configset_[ON_SET_CONFIG_SET].addListener(cb, false);
  }

  void ConfigAdmin::setOnAddConfigurationSet(OnAddConfigurationAddCallback* cb)
  {
    std::cerr << "setOnAddConfigurationSet function is obsolete." << std::endl;
    std::cerr << "Use addConfigurationSetListener instead." << std::endl;
    m_listeners.configset_[ON_ADD_CONFIG_SET].addListener(cb, false);
  }

  void
  ConfigAdmin::setOnRemoveConfigurationSet(OnRemoveConfigurationSetCallback* cb)
  {
    std::cerr << "setOnRemoveConfigurationSet function is obsolete."<<std::endl;
    std::cerr << "Use addConfigurationSetNameListener instead." << std::endl;
    m_listeners.configsetname_[ON_REMOVE_CONFIG_SET].addListener(cb, false);
  }

  void ConfigAdmin::setOnActivateSet(OnActivateSetCallback* cb)
  {
    std::cerr << "setOnActivateSet function is obsolete." << std::endl;
    std::cerr << "Use addConfigurationSetNameListener instead." << std::endl;
    m_listeners.configsetname_[ON_ACTIVATE_CONFIG_SET].addListener(cb, false);
  }
  //
  // end of obsolete functions
  //------------------------------------------------------------

  /*!
   * @if jp
   * @brief ConfigurationParamListener を追加する
   * @else
   * @brief Adding ConfigurationParamListener 
   * @endif
   */
  void ConfigAdmin::
  addConfigurationParamListener(ConfigurationParamListenerType type,
                                ConfigurationParamListener* listener,
                                bool autoclean)
  {
    m_listeners.configparam_[type].addListener(listener, autoclean);
  }

  /*!
   * @if jp
   * @brief ConfigurationParamListener を削除する
   * @else
   * @brief Removing ConfigurationParamListener 
   * @endif
   */
  void ConfigAdmin::
  removeConfigurationParamListener(ConfigurationParamListenerType type,
                                   ConfigurationParamListener* listener)
  {
    m_listeners.configparam_[type].removeListener(listener);
  }
    
  /*!
   * @if jp
   * @brief ConfigurationSetListener を追加する
   * @else
   * @brief Adding ConfigurationSetListener 
   * @endif
   */
  void ConfigAdmin::
  addConfigurationSetListener(ConfigurationSetListenerType type,
                              ConfigurationSetListener* listener,
                              bool autoclean)
  {
    m_listeners.configset_[type].addListener(listener, autoclean);
  }

  /*!
   * @if jp
   * @brief ConfigurationSetListener を削除する
   * @else
   * @brief Removing ConfigurationSetListener 
   * @endif
   */
  void ConfigAdmin::
  removeConfigurationSetListener(ConfigurationSetListenerType type,
                                 ConfigurationSetListener* listener)
  {
    m_listeners.configset_[type].removeListener(listener);
  }
    
  /*!
   * @if jp
   * @brief ConfigurationSetNameListener を追加する
   * @else
   * @brief Adding ConfigurationSetNameListener 
   * @endif
   */
  void ConfigAdmin::
  addConfigurationSetNameListener(ConfigurationSetNameListenerType type,
                                  ConfigurationSetNameListener* listener,
                                  bool autoclean)
  {
    m_listeners.configsetname_[type].addListener(listener, autoclean);
  }

  /*!
   * @if jp
   * @brief ConfigurationSetNameListener を削除する
   * @else
   * @brief Removing ConfigurationSetNameListener 
   * @endif
   */
  void ConfigAdmin::
  removeConfigurationSetNameListener(ConfigurationSetNameListenerType type,
                                     ConfigurationSetNameListener* listener)
  {
    m_listeners.configsetname_[type].removeListener(listener);
  }

  //------------------------------------------------------------
  // protected functions
  /*!
   * @if jp
   * @brief コンフィギュレーションパラメータの更新(ID指定)時にコールされる
   * @else
   * @brief When the configuration parameter is updated, it is called. 
   * @endif
   */
  void ConfigAdmin::onUpdate(const char* config_set)
  {
    m_listeners.configsetname_[ON_UPDATE_CONFIG_SET].notify(config_set);
  }

  /*!
   * @if jp
   * @brief コンフィギュレーションパラメータの更新(名称指定)時にコールされる
   * @else
   * @brief When the configuration parameter is updated, it is called. 
   * @endif
   */
  void
  ConfigAdmin::onUpdateParam(const char* config_set, const char* config_param)
  {
    m_listeners.configparam_[ON_UPDATE_CONFIG_PARAM].notify(config_set,
                                                            config_param);
  }

  /*!
   * @if jp
   * @brief コンフィギュレーションセットへの追加時にコールされる
   * @else
   * @brief Called when the property is added to the configuration set
   * @endif
   */
  void ConfigAdmin::onSetConfigurationSet(const coil::Properties& config_set)
  {
    m_listeners.configset_[ON_SET_CONFIG_SET].notify(config_set);
  }

  /*!
   * @if jp
   * @brief 設定値が追加されたときにコールされる。
   * @else
   * @brief Called when a set value is added to the configuration set
   * @endif
   */
  void ConfigAdmin::onAddConfigurationSet(const coil::Properties& config_set)
  {
    m_listeners.configset_[ON_ADD_CONFIG_SET].notify(config_set);
  }

  /*!
   * @if jp
   * @brief セットが削除されてるときにコールされる。
   * @else
   * @brief Called when the configuration set has been deleted
   * @endif
   */
  void ConfigAdmin::onRemoveConfigurationSet(const char* config_id)
  {
    m_listeners.configsetname_[ON_REMOVE_CONFIG_SET].notify(config_id);
  }

  /*!
   * @if jp
   * @brief セットがアクティブ化されたときにコールされる。
   * @else
   * @brief Called when the configuration set is made active
   * @endif
   */
  void ConfigAdmin::onActivateSet(const char* config_id)
  {
    m_listeners.configsetname_[ON_ACTIVATE_CONFIG_SET].notify(config_id);
  }
  

}; // namespace RTC
