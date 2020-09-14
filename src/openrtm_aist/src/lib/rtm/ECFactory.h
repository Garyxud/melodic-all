// -*- C++ -*-
/*!
 * @file ECFactory.h
 * @brief ExecutionContext Factory class
 * @date $Date: 2007-12-31 03:08:03 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2007-2008
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

#ifndef RTC_ECFACTORY_H
#define RTC_ECFACTORY_H

#include <rtm/ExecutionContextBase.h>
#include <string>

namespace RTC 
{
  
  typedef ExecutionContextBase* (*ECNewFunc)();
  typedef void (*ECDeleteFunc)(ExecutionContextBase* ec);
  
  /*!
   * @if jp
   *
   * @brief ExecutionContext生成用テンプレート関数
   * 
   * ExecutionContextのインスタンスを生成するためのテンプレート関数。
   *
   * @return 生成したExecutionContextインスタンス
   * 
   * @else
   *
   * @brief Template function to create ExecutionContext
   * 
   * Template function to create ExecutionContext's instances.
   *
   * @return Created ExecutionContext's instances
   *
   * @endif
   */
  template <class _New>
  ExecutionContextBase* ECCreate()
  {
    return new _New();
  }
  
  /*!
   * @if jp
   *
   * @brief ExecutionContext破棄用テンプレート関数
   * 
   * ExecutionContextのインスタンスを破棄するためのテンプレート関数。
   *
   * @param ec 破棄対象ExecutionContextのインスタンス
   *
   * @else
   *
   * @brief Template function to destroy ExecutionContext
   * 
   * Template function to destroy ExecutionContext's instances.
   *
   * @param ec The target ExecutionContext's instances for destruction
   *
   * @endif
   */
  template <class _Delete>
  void ECDelete(ExecutionContextBase* ec)
  {
    delete ec;
  }
  
  /*!
   * @if jp
   * @class ECFactoryBase
   * @brief ECFactoryBase 抽象クラス
   * 
   * ExecutionContext生成用Factoryの抽象クラス。
   * 各ExecutionContextを生成するための具象Factoryクラスは、
   * 以下の純粋仮想関数の実装を提供しなければならない。
   *
   * publicインターフェースとして以下のものを提供する。
   * - name()   : 生成対象ExecutionContext名称の取得
   * - create() : ExecutionContextインスタンスの生成
   * - destroy(): ExecutionContextインスタンスの破棄
   *
   * @since 0.4.0
   *
   * @else
   * @class ECFactoryBase
   * @brief ECFactoryBase abstract class
   * 
   * This is the abstruct Factory classes for ExecutionContext creation.
   * Concrete classes for each ExecutionContext creation must implement 
   * the following pure virtual functions.
   *
   * This class provides the following public interfaces.
   * - name()   : Get names of the target ExecutionContext for creation
   * - create() : Create ExecutionContext's instances
   * - destroy(): Destroy ExecutionContext's instances
   *
   * @since 0.4.0
   *
   * @endif
   */
  class ECFactoryBase
  {
  public:
    /*!
     * @if jp
     *
     * @brief 仮想デストラクタ
     * 
     * 仮想デストラクタ。
     *
     * @else
     *
     * @brief Virtual destructor
     * 
     * Virtual destructor
     *
     * @endif
     */
    virtual ~ECFactoryBase(void){};
    
    /*!
     * @if jp
     *
     * @brief 生成対象ExecutionContext名称取得用純粋仮想関数
     * 
     * 生成対象ExecutionContextの名称を取得するための純粋仮想関数。
     *
     * @return 生成対象ExecutionContext名称
     * 
     * @else
     *
     * @brief Pure virtual function to get names of creation target 
     *        ExecutionContext
     * 
     * Pure virtual function to get names of the target ExecutionContext
     * for creation.
     *
     * @return Names of the target ExecutionContext for creation
     *
     * @endif
     */
    virtual const char* name() = 0;
    
    /*!
     * @if jp
     *
     * @brief ExecutionContext生成用純粋仮想関数
     * 
     * ExecutionContextのインスタンスを生成するための純粋仮想関数。
     *
     * @return 生成したExecutionContextインスタンス
     * 
     * @else
     *
     * @brief Pure virtual function to create ExecutionContext.
     * 
     * Pure virtual function to create ExecutionContext's instances.
     *
     * @return Created ExecutionContext's instances
     *
     * @endif
     */
    virtual ExecutionContextBase* create() = 0;
    
    /*!
     * @if jp
     *
     * @brief ExecutionContext破棄用純粋仮想関数
     * 
     * ExecutionContextのインスタンスを破棄するための純粋仮想関数。
     *
     * @param comp 破棄対象のExecutionContextインスタンス
     * 
     * @else
     *
     * @brief Pure virtual function to destroy ExecutionContext.
     * 
     * Pure virtual function to destroy ExecutionContext's instances.
     *
     * @param comp The target ExecutionContext's instances for destruction
     *
     * @endif
     */
    virtual void destroy(ExecutionContextBase* comp) = 0;
  protected:
  };
  
  /*!
   * @if jp
   * @class ECFactoryCXX
   * @brief ECFactoryCXX クラス
   * 
   * C++言語用ExecutionContextインスタンスを生成するFactoryクラス。
   *
   * @since 0.4.0
   *
   * @else
   * @class ECFactoryCXX
   * @brief ECFactoryCXX class
   * 
   * Factory class to create the ExecutionContext's instances for C++.
   *
   * @since 0.4.0
   *
   * @endif
   */
  class ECFactoryCXX
    : public ECFactoryBase
  {
  public:
    /*!
     * @if jp
     *
     * @brief コンストラクタ
     * 
     * コンストラクタ
     *
     * @param name 生成対象ExecutionContext名称
     * @param new_func ExecutionContext生成用関数
     * @param delete_func ExecutionContext破棄用関数
     * 
     * @else
     *
     * @brief Constructor
     * 
     * Constructor
     *
     * @param name Name of the target ExecutionContext for creation
     * @param new_func Function to create ExecutionContext
     * @param delete_func Function to destroy ExecutionContext
     *
     * @endif
     */
    ECFactoryCXX(const char* name,
		 ECNewFunc new_func,
		 ECDeleteFunc delete_func);
    
    /*!
     * @if jp
     *
     * @brief 仮想デストラクタ
     * 
     * 仮想デストラクタ。
     *
     * @else
     *
     * @brief Virtual destructor
     * 
     * Virtual destructor.
     *
     * @endif
     */
    ~ECFactoryCXX(void);
    
    /*!
     * @if jp
     *
     * @brief 生成対象ExecutionContext名称を取得
     * 
     * 生成対象のExecutionContext名称を取得する。
     *
     * @return 生成対象ExecutionContext名称
     * 
     * @else
     *
     * @brief Get names of the target ExecutionContext for creation
     * 
     * Get names of the target ExecutionContext for creation.
     *
     * @return Names of target ExecutionContext for creation
     *
     * @endif
     */
    virtual const char* name();
    
    /*!
     * @if jp
     *
     * @brief 生成対象ExecutionContextインスタンスを生成
     * 
     * 生成対象のExecutionContextクラスのインスタンスを生成する。
     *
     * @return 生成したExecutionContextインスタンス
     * 
     * @else
     *
     * @brief Create the target ExecutionContext's instances
     * 
     * Create the target ExecutionContext class's instances.
     *
     * @return Created ExecutionContext's instances
     *
     * @endif
     */
    virtual ExecutionContextBase* create();
    
    /*!
     * @if jp
     *
     * @brief 対象ExecutionContextインスタンスを破棄
     * 
     * 対象ExecutionContextクラスのインスタンスを破棄する。
     *
     * @param comp 破棄対象ExecutionContextインスタンス
     * 
     * @else
     *
     * @brief Destroy the target ExecutionContext's instances
     * 
     * Destroy the target ExecutionContext's instances.
     *
     * @param comp The target ExecutionContext's instances to destroy
     *
     * @endif
     */
    virtual void destroy(ExecutionContextBase* comp);
    
  protected:
    /*!
     * @if jp
     * @brief  生成対象ExecutionContext名称
     * @else
     * @brief  Names of the target ExecutionContext for creation
     * @endif
     */
    std::string m_name;
    
    /*!
     * @if jp
     * @brief  対象ExecutionContext生成用関数
     * @else
     * @brief  Function to create the target ExecutionContext
     * @endif
     */
    ECNewFunc m_New;
    
    /*!
     * @if jp
     * @brief  対象ExecutionContext破棄用関数
     * @else
     * @brief  Function to destroy the target ExecutionContext
     * @endif
     */
    ECDeleteFunc m_Delete;
  };
};
#endif // RTC_ECFACTORY_H
