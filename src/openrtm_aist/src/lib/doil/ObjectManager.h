// -*- C++ -*-
/*!
 * @file ObjectManager.h
 * @brief Object management class
 * @date $Date: 2007-12-31 03:08:04 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2003-2007
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id: ObjectManager.h 845 2008-09-25 11:10:40Z n-ando $
 *
 */

#ifndef ObjectManager_h
#define ObjectManager_h

#include <vector>
#include <string>
#include <algorithm>

#include <coil/Mutex.h>
#include <coil/Guard.h>

/*!
 * @if jp
 *
 * @brief オブジェクト管理用クラス
 *
 * 各種オブジェクトを管理するためのクラス。
 *
 * @since 0.4.0
 *
 * @else
 *
 * @brief Class for managing objects
 *
 * This is a class for managing various objects.
 *
 * @since 0.4.0
 *
 * @endif
 */
template <typename Identifier, typename Object, typename Predicate>
class ObjectManager
{
public:
  typedef std::vector<Object*>                  ObjectVector;
  typedef typename ObjectVector::iterator       ObjectVectorItr;
  typedef typename ObjectVector::const_iterator ObjectVectorConstItr;
  typedef coil::Mutex Mutex;
  typedef coil::Guard<coil::Mutex> Guard;


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
  ObjectManager(){};
  
  /*!
   * @if jp
   *
   * @brief デストラクタ
   * 
   * デストラクタ
   * 
   * @else
   *
   * @brief Destructor
   * 
   * Destructor
   * 
   * @endif
   */
  ~ObjectManager(){};
  
  /*!
   * @if jp
   *
   * @brief 指定したオブジェクトを登録する
   * 
   * 指定したオブジェクトを登録する。
   * 同一オブジェクトが登録済みの場合は、何も行わない。
   *
   * @param obj 登録対象オブジェクト
   *
   * @return 登録処理結果(オブジェクトを登録した場合にtrue)
   * 
   * @else
   *
   * @brief Register the specified object
   * 
   * Register the object that was specified.
   * If the same object is already registered, this does not anything.
   *
   * @param obj The target object for the registration
   *
   * @return Registration result (True if object was registerd)
   * 
   * @endif
   */
  bool registerObject(Object* obj)
  {
    ObjectVectorItr it;
    Guard guard(m_objects._mutex);
    
    it = std::find_if(m_objects._obj.begin(), m_objects._obj.end(),
		      Predicate(obj));
    if (it == m_objects._obj.end())
      {
	m_objects._obj.push_back(obj);
	return true;
      }
    return false;
  }
  
  /*!
   * @if jp
   *
   * @brief 指定したオブジェクトを登録解除する
   * 
   * 指定したオブジェクトの登録を解除し、取得する。
   * 指定したオブジェクトが登録されていない場合にはNULLを返す。
   *
   * @param id 登録解除対象オブジェクトのID
   *
   * @return 登録解除されたオブジェクト
   * 
   * @else
   *
   * @brief Unregister the specified object
   * 
   * Unregister the object that was specified and get it.
   * This operation returns NULL if the specified object is not 
   * registered.
   *
   * @param id ID of the target object for the unregistration
   *
   * @return Unregistered object
   * @endif
   */
  Object* unregisterObject(const Identifier& id)
  {
    ObjectVectorItr it;
    Guard guard(m_objects._mutex);
    
    it = std::find_if(m_objects._obj.begin(), m_objects._obj.end(),
		      Predicate(id));
    if (it != m_objects._obj.end())
      {
	Object* obj(*it);
	m_objects._obj.erase(it);
	return obj;
      }
    return NULL;;
  }
  
  /*!
   * @if jp
   *
   * @brief オブジェクトを検索する
   * 
   * 登録されているオブジェクトの中から指定した条件に合致するオブジェクトを検索
   * して取得する。
   * 指定した条件に合致するオブジェクトが登録されていない場合にはNULLを返す。
   *
   * @param id 検索対象オブジェクトのID
   *
   * @return オブジェクトの検索結果
   * 
   * @else
   *
   * @brief Find the object
   * 
   * Find the object that matches the specified condition among the registered
   * objects and get it.
   * This operation returns NULL if the object that does matches the specified
   * condition is not registered.
   *
   * @param id ID of the target object for finding
   *
   * @return Result of finding object
   * 
   * @endif
   */
  Object* find(const Identifier& id) const
  {
    ObjectVectorConstItr it;
    Guard guard(m_objects._mutex);
    it = std::find_if(m_objects._obj.begin(), m_objects._obj.end(),
		      Predicate(id));
    if (it != m_objects._obj.end())
      {
	return *it;
      }
    return NULL;
  }
  
  /*!
   * @if jp
   *
   * @brief 登録されているオブジェクトのリストを取得する
   * 
   * 登録されているオブジェクトのリストを取得する。
   *
   * @return 登録されているオブジェクト・リスト
   * 
   * @else
   *
   * @brief Get a list of obejects that are registerd
   * 
   * Get a list of objects that are registerd.
   *
   * @return List of registerd objects.
   * 
   * @endif
   */
  std::vector<Object*> getObjects() const
  {
    Guard guard(m_objects._mutex);
    return m_objects._obj;
  }
  
  /*!
   * @if jp
   * @brief オブジェクト検索用ファンクタ
   * @else
   * @brief Functor for searching object
   * @endif
   */
  template <class Pred>
  Pred for_each(Pred p)
  {
    Guard guard(m_objects._mutex);
    return std::for_each(m_objects._obj.begin(), m_objects._obj.end(), p);
  }
  
  /*!
   * @if jp
   * @brief オブジェクト検索用ファンクタ
   * @else
   * @brief Functor for searching object
   * @endif
   */
  template <class Pred>
  Pred for_each(Pred p) const
  {
    Guard guard(m_objects._mutex);
    return std::for_each(m_objects._obj.begin(), m_objects._obj.end(), p);
  }
  
protected:
  /*!
   * @if jp
   * @brief オブジェクト管理用構造体
   * @else
   * @brief The structure for object management
   * @endif
   */
  struct Objects
  {
    mutable Mutex _mutex;
    ObjectVector _obj;
  };
  /*!
   * @if jp
   * @brief 登録済みオブジェクト・リスト
   * @else
   * @brief The list of registered objects
   * @endif
   */
  Objects m_objects;
};
#endif // ObjectManager_h
