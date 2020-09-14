// -*- C++ -*-
/*!
 * @file OrbManager.h
 * @brief Generic ORB manager
 * @date $Date$
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef DOIL_ORB_MANAGER_H
#define DOIL_ORB_MANAGER_H

#include <string>
#include <vector>
#include <coil/Mutex.h>
#include <doil/doil.h>
#include <doil/ObjectManager.h>
#include <doil/IORB.h>

namespace doil
{
  class ORBManager
  {
  public:
    /*!
     * @if jp
     * @brief 初期化関数
     * @else
     * @brief initializer
     * @endif
     */
    static ORBManager* init(coil::Properties prop)
      throw();
    
    /*!
     * @if jp
     * @brief インスタンス取得関数
     * @else
     * @brief getting instance
     * @endif
     */
    static ORBManager& instance()
      throw();

    /*!
     * @if jp
     * @brief ORBManager を shutdown する
     *
     * 登録済みのすべての ORB に対して、IOR::shutdown() を呼び出し
     * shutdown する。
     *
     * @else
     * @brief shutdown ORBManager
     *
     * This operation will shutdown all the registered ORBs calling
     * IORB::shutdown() operation.
     *
     * @endif
     */
    virtual void shutdown() throw();

    /*!
     * @if jp
     * @brief ORBManager に ORB を登録する
     *
     * ORBManager に IORB を継承した ORB オブジェクトを登録する。
     *
     * @else
     * @brief Register an ORB to the ORBManager
     *
     * This operation will register an ORB that inherits IORB
     * interface to the ORBManager.
     *
     * @endif
     */
    ReturnCode_t addORB(IORB* orb) throw();

    /*!
     * @if jp
     * @brief 登録済みの ORB を取得する
     *
     * 名前を指定し登録済みの ORB を取得する。
     * 指定した名前の ORB がない場合は NULL が返される。
     *
     * @else
     * @brief Get an ORB that is registered in this ORBManager
     *
     * This operation will return an registered ORB specified as the name.
     * NULL will be returned if no ORB found in this ORBManager.
     *
     * @endif
     */
    IORB* getORB(const char* name) throw();

    /*!
     * @if jp
     * @brief 登録済みのすべての ORB を取得する
     *
     * 登録済みのすべての ORB を取得する。
     * ORB が一つも登録されていない場合長さ0のリストが返される。
     *
     * @else
     * @brief Get all the ORBs that are registered in this ORBManager
     *
     * This operation will return all the registered ORBs.
     * Zero length list will be returned if no ORB registered.
     *
     * @endif
     */
    const std::vector<IORB*> getORBs() throw();

    /*!
     * @if jp
     * @brief 登録済みのすべての ORB を名前を取得する
     *
     * 登録済みのすべての ORB を名前を取得する。
     * ORB が一つも登録されていない場合長さ0のリストが返される。
     *
     * @else
     * @brief Get all the ORBs' name that are registered in this ORBManager
     *
     * This operation will return all the names of registered ORBs.
     * Zero length list will be returned if no ORB registered.
     *
     * @endif
     */
    const std::vector<std::string> availableORBs() throw();

    /*!
     * @if jp
     * @brief 登録済みの ORB を ORBManager のリストから削除する
     *
     * 登録済みの ORB を ORBManager のリストから削除する。
     * この関数では、ORBをリストから削除するのみで実体は削除されない。
     * 戻り値には引数で与えた名称の ORB のポインタが返される。
     * ORB の実体を削除するのはユーザの責任である。
     * 指定した名前の ORB が存在しない場合には NULL が返される。
     *
     * @else
     * @brief Delete registered ORB by name.
     *
     * This operation will delete the ORB from ORBManager's list.
     * This operation never delete the ORB itself.
     * ORB's pointer deleted from the list will be returned.
     * NULL will be returned if there is no specified name's ORB.
     *
     * @endif
     */
    IORB* deleteORB(const char* name) throw();

    //------------------------------------------------------------

    /*!
     * @if jp
     * @brief オブジェクトをアクティブ化する
     *
     * 与えられたオブジェクトを ORB 上でアクティブ化する。
     * オブジェクトはアクティブ化と同時に ORBManager に登録される。
     *
     * @else
     * @brief Activate object
     *
     * This operation will activate a given object on ORBs.
     *
     * @endif
     */
    virtual ReturnCodes activateObject(ImplBase* impl,
                                         const char* orbname = "") throw();
    /*!
     * @if jp
     * @brief オブジェクトを非アクティブ化する
     *
     * 与えられたオブジェクトを ORB 上で非アクティブ化する。
     *
     * @else
     * @brief Deactivate object
     *
     * This operation will deactivate a given object on ORBs.
     *
     * @endif
     */
    virtual ReturnCodes deactivateObject(ImplBase* impl,
                                           const char* orbname = "") throw();

    /*!
     * @if jp
     * @brief オブジェクトを非アクティブ化する
     *
     * 与えられたオブジェクトを ORB 上で非アクティブ化する。
     *
     * @else
     * @brief Deactivate object
     *
     * This operation will deactivate a given object on ORBs.
     *
     * @endif
     */
    virtual ReturnCodes  deactivateObject(const char* name,
                                            const char* orbname = "") throw();

    /*!
     * @if jp
     * @brief オブジェクトを削除する
     *
     * 与えられたオブジェクトを ORBManager のリストから削除する。
     *
     * @param impl リストから削除するオブジェクトのポインタ
     * @return ReturnCode_t OK:        正常終了
     *                      NOT_FOUND: 該当するオブジェクトが見つからない
     *                      UNKNOWN:   不明なエラー
     *
     * @else
     * @brief Delete object
     *
     * This operation will delete a given object from ORBManager's list.
     *
     * @endif
     */
    ReturnCode_t deleteObject(ImplBase* impl) throw();

    /*!
     * @if jp
     * @brief オブジェクトを削除する
     *
     * 与えられたオブジェクトを ORBManager のリストから削除する。
     *
     * @param  name リストから削除するオブジェクトの名前
     * @return ReturnCode_t OK:        正常終了
     *                      NOT_FOUND: 該当するオブジェクトが見つからない
     *                      UNKNOWN:   不明なエラー
     *
     * @else
     * @brief Delete object
     *
     * This operation will delete a given object from ORBManager's list.
     *
     * @endif
     */
    ReturnCode_t deleteObject(const char* name) throw();

    /*!
     * @if jp
     * @brief オブジェクトを取得する
     *
     * 登録済みのオブジェクトを取得する
     *
     * @else
     * @brief Delete object
     *
     * This operation will returen register objects.
     *
     * @endif
     */
    const std::vector<ImplBase*> getObjects() throw();

    /*!
     * @if jp
     * @brief オブジェクトを取得する
     *
     * 登録済みのオブジェクトを取得する
     *
     * @else
     * @brief Delete object
     *
     * This operation will returen register objects.
     *
     * @endif
     */
    ImplBase* getObject(const char* name) throw();

  protected:
    //------------------------------------------------------------
    // private functions
    //------------------------------------------------------------
  private:
    ORBManager(){}
    ORBManager(const ORBManager&);
    ORBManager& operator=(const ORBManager&);
    ~ORBManager(){}


    //------------------------------------------------------------
    // private variables
    //------------------------------------------------------------
  private:
    // static data member for singleton
    static ORBManager* manager;
    static coil::Mutex mutex;

    //------------------------------------------------------------
    // ORB map
    //------------------------------------------------------------
    class ORBPred
    {
    public:
      ORBPred(const char* name)
        : m_name(name)
      {
      }
      ORBPred(IORB* orb)
        : m_name(orb->name())
      {
      }
      bool operator()(IORB* orb)
      {
        return m_name == orb->name();
      }
      std::string m_name;
    };

    typedef ObjectManager<const char*, IORB, ORBPred> ORBMap;
    ORBMap m_orbs;
    // end of ORBMap
    //------------------------------------------------------------


    //------------------------------------------------------------
    // impl object map
    //------------------------------------------------------------
    class ImplPred
    {
    public:
      ImplPred(const char* name)
        : m_name(name)
      {
      }
      ImplPred(ImplBase* impl)
        : m_name(impl->name())
      {
      }
      bool operator()(ImplBase* impl)
      {
        return m_name == impl->name();
      }
      std::string m_name;
    };

    typedef ObjectManager<const char*, ImplBase, ImplPred> ImplMap;
    ImplMap m_impls;
  
    //    std::vector<IORB*> m_orbs;
    //    std::vector<ImplBase*> m_impl;
    //    typedef std::vector<IORB*>::iterator OrbIt;
    //    typedef std::vector<ImplBase*>::iterator ImplIt;


    //------------------------------------------------------------
    // functors for IORB operation
    //------------------------------------------------------------

    //------------------------------------------------------------
    // shutdown ORBs
    class orb_shutdown
    {
    public:
      void operator()(IORB* orb)
      {
        orb->shutdown();
      }
    };
    //
//    class find_orb
//    {
//    public:
//      find_orb(const char* name) : m_name(name)
//      {
//      }
//      bool operator()(IORB* orb)
//      {
//        return m_name == orb->name();
//      }
//      std::string m_name;
//    };
    
    class collect_orbs
    {
    public:
      void operator()(IORB* orb)
      {
        orbs_.push_back(orb);
      }
      std::vector<IORB*> orbs_;
    };

    class available_orbs
    {
    public:
      void operator()(IORB* orb)
      {
        orbs_.push_back(orb->name());
      }
      std::vector<std::string> orbs_;
    };

    /*!
     * @if jp
     * @brief Implをactivateするfunctor
     * @else
     * @brief A functor to activate a given impl object
     * @endif
     */
    class activate_impl
    {
    public:
      activate_impl(ImplBase* impl,
                    const char* orbname = "")
        : m_impl(impl), m_orbname(orbname)
      {
      }
      void operator()(IORB* orb)
      {
        if (m_orbname.empty() || m_orbname == orb->name())
          {
            ReturnCode_t ret = orb->activateObject(m_impl);
            retcodes_.push_back(orb->name(), ret);
          }
      }
      ReturnCodes retcodes_;
    private:
      ImplBase* m_impl;
      std::string m_orbname;
    };
    

    /*!
     * @if jp
     * @brief Implをdeactivateするfunctor
     * @else
     * @brief A functor to deactivate a given impl object
     * @endif
     */
    class deactivate_impl
    {
    public:
      deactivate_impl(ImplBase* impl, const char* orbname = "")
        : m_impl(impl), m_orbname(orbname)
      {
      }
      void operator()(IORB* orb)
      {
        if (m_orbname.empty() || m_orbname == orb->name())
          {
            ReturnCode_t ret = orb->deactivateObject(m_impl);
            retcodes_.push_back(orb->name(), ret);
          }
      }
      ReturnCodes retcodes_;
    private:
      ImplBase* m_impl;
      std::string m_orbname;
    };
    
    class deactivate_by_name
    {
    public:
      deactivate_by_name(const char* name, const char* orbname = "")
        : m_name(name), m_orbname(orbname)
      {
      }
      void operator()(IORB* orb)
      {
        if (m_orbname.empty() || m_orbname == orb->name())
          {
            ReturnCode_t ret(orb->deactivateObject(m_name.c_str()));
            retcodes_.push_back(orb->name(), ret);
          }
      }
      ReturnCodes retcodes_;
      std::string m_name;
      std::string m_orbname;
    };

    class collect_impl
    {
    public:
      collect_impl()
      {
      }
      void operator()(ImplBase* impl)
      {
        impls_.push_back(impl);
      }
      std::vector<ImplBase*> impls_;
    };
  };
}; // namespace doil
#endif // DOIL_ORB_MANAGER_H
