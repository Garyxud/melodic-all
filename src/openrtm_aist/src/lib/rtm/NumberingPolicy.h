// -*- C++ -*-
/*!
 * @file NumberingPolicy.h
 * @brief Object numbering policy class
 * @date $Date: 2007-12-31 03:08:04 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006-2008
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

#ifndef RTC_NUMBERINGPOLICY_H
#define RTC_NUMBERINGPOLICY_H

#include <string>
#include <vector>

/*!
 * @if jp
 *
 * @class NumberingPolicy
 * @brief オブジェクト生成時ネーミング・ポリシー(命名規則)管理用抽象クラス
 *
 * オブジェクトを生成する際のネーミング・ポリシー(命名規則)を管理するための
 * 抽象インターフェースクラス。
 * 具象クラスは、以下の純粋仮想関数の実装を提供しなければならない。
 * - onCreate() : オブジェクト生成時の名称作成
 * - onDelete() : オブジェクト削除時の名称解放
 *
 * @since 0.4.0
 *
 * @else
 * @class NumberingPolicy
 * @brief Abstruct class for naming policy management when creating objects
 *
 * This is the abstract interface class to manage the naming policy when
 * creating objects.
 * Concrete classes must implement the following pure virtual functions.
 * - onCreate() : Create the name when creating objects.
 * - onDelete() : Delete the name when deleting objects.
 *
 * @since 0.4.0
 *
 * @endif
 */
class NumberingPolicy
{
public:
  /*!
   * @if jp
   * @brief オブジェクト未発見例外処理用構造体
   * @else
   * @brief The structures for exception handling when object was not
   *        found.
   * @endif
   */
  struct ObjectNotFound {};
  
  /*!
   * @if jp
   *
   * @brief 仮想デストラクタ
   * 
   * @else
   *
   * @brief Virtual destractor
   *
   * @endif
   */
  virtual ~NumberingPolicy(void){};
  
  /*!
   * @if jp
   *
   * @brief オブジェクト生成時の名称作成
   *
   * オブジェクト生成時の名称を生成するための純粋仮想関数
   * 
   * @param obj 名称生成対象オブジェクト
   *
   * @return 生成したオブジェクト名称
   *
   * @else
   *
   * @brief Create the name when creating objects
   *
   * Pure virtual function to create the name when creating objects.
   * 
   * @param obj The target object for the creation
   *
   * @return Name of the created object
   *
   * @endif
   */
  virtual std::string onCreate(void* obj) = 0;
  
  /*!
   * @if jp
   *
   * @brief オブジェクト削除時の名称解放
   *
   * オブジェクト削除時に名称を解放するための純粋仮想関数
   * 
   * @param obj 名称解放対象オブジェクト
   *
   * @else
   *
   * @brief Delete the name when deleting objects
   *
   * Pure virtual function to delete the name when deleting object.
   * 
   * @param obj The target object of the delete
   *
   * @endif
   */
  virtual void onDelete(void* obj) = 0;
};

/*!
 * @if jp
 *
 * @class DefaultNumberingPolicy
 * @brief オブジェクト生成時ネーミング・ポリシー(命名規則)管理用クラス
 *
 * オブジェクトを生成する際のネーミング・ポリシー(命名規則)を管理するための
 * クラス。
 *
 * @since 0.4.0
 *
 * @else
 *
 * @class DefaultNumberingPolicy
 * @brief Class for naming policy management when creating objects
 *
 * This is a class to manage the naming policy when creating objects.
 *
 * @since 0.4.0
 *
 * @endif
 */
class DefaultNumberingPolicy
  : public NumberingPolicy 
{
public:
  /*!
   * @if jp
   *
   * @brief コンストラクタ
   * 
   * コンストラクタ
   * 
   * @else
   *
   * @brief Constructor
   * 
   * Constructor
   * 
   * @endif
   */
  DefaultNumberingPolicy() : m_num(0) {};
  
  /*!
   * @if jp
   *
   * @brief デストラクタ
   * 
   * @else
   *
   * @brief Destractor
   *
   * @endif
   */
  virtual ~DefaultNumberingPolicy(void){};
  
  /*!
   * @if jp
   *
   * @brief オブジェクト生成時の名称作成
   *
   * オブジェクト生成時の名称を生成する。
   * 生成済みインスタンスの数に応じた名称を生成する。
   * 
   * @param obj 名称生成対象オブジェクト
   *
   * @return 生成したオブジェクト名称
   *
   * @else
   *
   * @brief Create the name when creating object
   *
   * Create the name when creating object.
   * Create the name corresponding to the number of generated instances.
   * 
   * @param obj The target object for the name creation
   *
   * @return Names of the created object
   *
   * @endif
   */
  virtual std::string onCreate(void* obj);
  
  /*!
   * @if jp
   *
   * @brief オブジェクト削除時の名称解放
   *
   * オブジェクト削除時に名称を解放する。
   * オブジェクト削除時に生成済みインスタンス数を減算する。
   * 
   * @param obj 名称解放対象オブジェクト
   *
   * @else
   *
   * @brief Delete the name when deleting object
   *
   * Delete the name when deleting object.
   * Substract the generated number of instances when deleting the object.
   * 
   * @param obj The target object for the name delete
   *
   * @endif
   */
  virtual void onDelete(void* obj);
  
protected:
  /*!
   * @if jp
   *
   * @brief オブジェクトの検索
   *
   * オブジェクトリストから指定されたオブジェクトを検索し、
   * 該当するオブジェクトが格納されている場合にはインデックスを返す。
   * 
   * @param obj 検索対象オブジェクト
   *
   * @return オブジェクト格納インデックス
   *
   * @else
   *
   * @brief Find the object
   *
   * Find the specified object in the object list and return its index
   * when it is stored.
   * 
   * @param obj The target object for the find
   *
   * @return Object index for storage
   *
   * @endif
   */
  long int find(void* obj);
  
private:
  int m_num;
  std::vector<void*> m_objects;
};
#endif // RTC_NUMBERINGPOLICY_H
