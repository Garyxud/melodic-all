// -*- C++ -*-
/*!
 * @file Factory.h
 * @brief RT component manager class
 * @date $Date: 2007-12-31 03:08:03 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2003-2008
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

#include <rtm/Factory.h>
#include <rtm/RTObject.h>

namespace RTC 
{
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  FactoryBase::FactoryBase(const coil::Properties& profile)
    : m_Profile(profile), m_Number(-1)
  {
  }
  
  /*!
   * @if jp
   * @brief デストラクタ
   * @else
   * @brief Destructor
   * @endif
   */
  FactoryBase::~FactoryBase()
  {
  }
  
  /*!
   * @if jp
   * @brief コンポーネントプロファイルの取得
   * @else
   * @brief Get the component profile
   * @endif
   */
  coil::Properties& FactoryBase::profile()
  {
    return m_Profile;
  }
  
  /*!
   * @if jp
   * @brief 現在のインスタンス数の取得
   * @else
   * @brief Get the number of current instances
   * @endif
   */
  int FactoryBase::number()
  {
    return m_Number;
  }
  
  /*!
   * @if jp
   * @brief コンストラクタ
   * @else
   * @brief Constructor
   * @endif
   */
  FactoryCXX::FactoryCXX(const coil::Properties& profile,
			 RtcNewFunc new_func,
			 RtcDeleteFunc delete_func,
			 NumberingPolicy* policy)
    : FactoryBase(profile),
      m_New(new_func),
      m_Delete(delete_func),
      m_policy(policy)
  {
    if (m_policy == NULL)
      throw std::bad_alloc();
  }
  
  /*!
   * @if jp
   * @brief コンポーネントの生成
   * @else
   * @brief Create RT-Components
   * @endif
   */
  RTObject_impl* FactoryCXX::create(Manager* mgr)
  {
    try
      {
	RTObject_impl* rtobj(m_New(mgr));
	if (rtobj == 0) return NULL;
	
	++m_Number;
	rtobj->setProperties(this->profile());
	
	// create instance_name
	std::string instance_name(rtobj->getTypeName());
	instance_name.append(m_policy->onCreate(rtobj));
	rtobj->setInstanceName(instance_name.c_str());
	
	return rtobj;
      }
    catch (...)
      {
	return NULL;
      }
  }
  
  /*!
   * @if jp
   * @brief コンポーネントの破棄
   * @else
   * @brief Destroy RT-Components
   * @endif
   */
  void FactoryCXX::destroy(RTObject_impl* comp)
  {
    try
      {
        --m_Number;
        m_policy->onDelete(comp);
        m_Delete(comp);
      }
    catch (...)
      {
        
      }
  }
};
