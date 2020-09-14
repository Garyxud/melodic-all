// -*- C++ -*-
/*!
 * @file NamingManager.h
 * @brief naming Service helper class
 * @date $Date: 2007-12-31 03:08:04 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2006
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#ifndef RTC_NAMINGMANAGER_H
#define RTC_NAMINGMANAGER_H

#include <rtm/RTC.h>

#include <coil/Task.h>
#include <coil/Mutex.h>
#include <coil/Guard.h>
#include <rtm/CorbaNaming.h>
#include <rtm/RTObject.h>
#include <rtm/SystemLogger.h>
#include <rtm/ManagerServant.h>

namespace RTC
{
  class Manager;
  /*!
   * @if jp
   *
   * @class NamingBase
   * @brief NamingService 管理用抽象クラス
   *
   * NamingServer 管理用抽象インターフェースクラス。
   * 具象管理クラスは、以下の純粋仮想関数の実装を提供しなければならない。
   * - bindObject() : 指定したオブジェクトのNamingServiceへのバインド
   * - unbindObject() : 指定したオブジェクトのNamingServiceからのアンバインド
   *
   * @since 0.4.0
   *
   * @else
   *
   * @class NamingBase
   * @brief NamingService management abstract class
   *
   * This is the abstract interface class for NamingServer management.
   * Concrete management classes must implement the following pure virtual 
   * functions.
   * - bindObject() : Bind the specified object to NamingService
   * - unbindObject() : Unbind the specified object from NamingService
   *
   * @since 0.4.0
   *
   * @endif
   */
  class NamingBase
  {
    typedef coil::Mutex Mutex;
    typedef coil::Guard<Mutex> Guard;
  public:
    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * @else
     *
     * @brief Constructor
     *
     * @endif
     */
    NamingBase() {};
    
    /*!
     * @if jp
     *
     * @brief デストラクタ
     *
     * @else
     *
     * @brief Destructor
     *
     * @endif
     */
    virtual ~NamingBase(void) {};
    
    /*!
     * @if jp
     *
     * @brief 指定したオブジェクトをNamingServiceへバインドする純粋仮想関数
     *
     * @param name バインド時の名称
     * @param rtobj バインド対象オブジェクト
     *
     * @else
     *
     * @brief Pure virtual function to bind the specified objects 
     *        to the NamingService
     *
     * @param name The name to be bound to the NamingService
     * @param rtobj The target objects to be bound to the NamingSerivce
     *
     * @endif
     */
    virtual void bindObject(const char* name, const RTObject_impl* rtobj) = 0;

    /*!
     * @if jp
     *
     * @brief 指定したManagerServantをNamingServiceへバインドする純粋仮想関数
     *
     * @param name バインド時の名称
     * @param rtobj バインド対象ManagerServant
     *
     * @else
     *
     * @brief Pure virtual function to bind the specified ManagerServants 
     *        to NamingService
     *
     * @param name The name to be bound to the NamingService
     * @param rtobj The target objects to be bound to the NamingSerivce
     *
     * @endif
     */
    virtual void bindObject(const char* name,
                            const RTM::ManagerServant* mgr) = 0;
    
    /*!
     * @if jp
     *
     * @brief 指定したオブジェクトをNamingServiceからアンバインドするための
     *        純粋仮想関数
     *
     * @param name アンバインド対象オブジェクト
     *
     * @else
     *
     * @brief Pure virtual function to unbind the specified objects from 
     *        NamingService
     *
     * @param name The name of the object released from NamingService
     *
     * @endif
     */
    virtual void unbindObject(const char* name) = 0;

    /*!
     * @if jp
     *
     * @brief ネームサーバの生存を確認する。
     * 
     * @return true:生存している, false:生存していない
     *
     * @else
     *
     * @brief Check if the name service is alive
     * 
     * @return true: alive, false:non not alive
     *
     * @endif
     */
    virtual bool isAlive() = 0;
  };
  

  /*!
   * @if jp
   *
   * @class NamingOnCorba
   * @brief CORBA 用 NamingServer 管理クラス
   *
   * CORBA 用 NamingServer 管理用クラス。
   * CORBA コンポーネントの NamingService への登録、解除などを管理する。
   *
   * @since 0.4.0
   *
   * @else
   *
   * @class NamingOnCorba
   * @brief NamingServer management class for CORBA
   *
   * NamingServer management class for CORBA.
   * Manage to register and unregister CORBA components to NamingService.
   *
   * @since 0.4.0
   *
   * @endif
   */
  class NamingOnCorba
    : public virtual NamingBase
  {
  public:
    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * コンストラクタ。第2引数に与えるネームサービス名は、ネームサービ
     * スのホスト名とポート番号を ":" で区切ったものである。ポート番号
     * が省略された場合、2809番ポートが使用される。
     *
     * @param orb ORB
     * @param names NamingServer 名称
     *
     * @else
     *
     * @brief Constructor
     *
     * Constructor. Naming service name that is given at the second
     * argument is host name and port number hoined with ":". If the
     * port number is abbreviated, the default port number 2809 is
     * used.
     *
     * @param orb ORB
     * @param names Name of NamingServer
     *
     * @endif
     */
    NamingOnCorba(CORBA::ORB_ptr orb, const char* names);
    
    /*!
     * @if jp
     *
     * @brief デストラクタ
     *
     * @else
     *
     * @brief Destructor
     *
     * @endif
     */
    virtual ~NamingOnCorba(void){};
    
    /*!
     * @if jp
     *
     * @brief 指定した CORBA オブジェクトのNamingServiceへバインド
     * 
     * 指定した CORBA オブジェクトを指定した名称で CORBA NamingService へ
     * バインドする。
     * 
     * @param name バインド時の名称
     * @param rtobj バインド対象オブジェクト
     *
     * @else
     *
     * @brief Bind the specified CORBA objects to NamingService
     * 
     * Bind the specified CORBA objects to CORBA NamingService
     * by specified names.
     * 
     * @param name Names at the binding
     * @param rtobj The target objects for the binding
     *
     * @endif
     */
    virtual void bindObject(const char* name, const RTObject_impl* rtobj);
    /*!
     * @if jp
     *
     * @brief 指定したManagerServantをNamingServiceへバインド
     *
     * @param name バインド時の名称
     * @param rtobj バインド対象ManagerServant
     *
     * @else
     *
     * @brief Bind the specified ManagerServants to NamingService
     *
     * @param name Names at the binding
     * @param mgr The target ManagerServants for the binding
     *
     * @endif
     */
    virtual void bindObject(const char* name, const RTM::ManagerServant* mgr);
    
    /*!
     * @if jp
     *
     * @brief 指定した CORBA オブジェクトをNamingServiceからアンバインド
     * 
     * 指定した CORBA オブジェクトを CORBA NamingService からアンバインドする。
     * 
     * @param name アンバインド対象オブジェクト
     *
     * @else
     *
     * @brief Unbind the specified CORBA objects from NamingService
     * 
     * Unbind the specified CORBA objects from CORBA NamingService.
     * 
     * @param name The target objects for the unbinding
     *
     * @endif
     */
    virtual void unbindObject(const char* name);
    
    /*!
     * @if jp
     *
     * @brief ネームサーバの生存を確認する。
     * 
     * @return true:生存している, false:生存していない
     *
     * @else
     *
     * @brief Check if the name service is alive
     * 
     * @return true: alive, false:non not alive
     *
     * @endif
     */
    virtual bool isAlive();

  private:
    Logger rtclog;
    CorbaNaming m_cosnaming;
    std::string m_endpoint;
    bool m_replaceEndpoint;
    std::map<std::string, RTObject_impl*> m_names;
  };
  
  /*!
   * @if jp
   *
   * @class NamingManager
   * @brief NamingServer 管理クラス
   *
   * NamingServer 管理用クラス。
   * コンポーネントのNamingServiceへの登録、解除などを管理する。
   *
   * @since 0.4.0
   *
   * @else
   *
   * @class NamingManager
   * @brief NamingServer management class
   *
   * NamingServer management class.
   * Manage to register and unregister components to NamingService.
   *
   * @since 0.4.0
   *
   * @endif
   */
  class NamingManager
  {
    typedef coil::Mutex Mutex;
    typedef coil::Guard<Mutex> Guard;
  public:
    /*!
     * @if jp
     *
     * @brief コンストラクタ
     *
     * コンストラクタ
     *
     * @param manager マネージャオブジェクト
     *
     * @else
     *
     * @brief Constructor
     *
     * Constructor
     *
     * @param manager Manager object
     *
     * @endif
     */
    NamingManager(Manager* manager);
    
    /*!
     * @if jp
     *
     * @brief デストラクタ
     *
     * @else
     *
     * @brief Destructor
     *
     * @endif
     */
    virtual ~NamingManager(void);
    
    /*!
     * @if jp
     *
     * @brief NameServer の登録
     *
     * 指定した形式の NameServer を登録する。
     * 現在指定可能な形式は CORBA のみ。
     *
     * @param method NamingService の形式
     * @param name_server 登録する NameServer の名称
     *
     * @else
     *
     * @brief Regster the NameServer
     *
     * Register NameServer by specified format.
     * Currently. only CORBA can be specified.
     *
     * @param method Format of NamingService
     * @param name_server Name of NameServer for registration
     *
     * @endif
     */
    void registerNameServer(const char* method, const char* name_server);
    
    /*!
     * @if jp
     *
     * @brief 指定したオブジェクトのNamingServiceへバインド
     * 
     * 指定したオブジェクトを指定した名称で CORBA NamingService へバイ
     * ンドする。
     * 
     * @param name バインド時の名称
     * @param rtobj バインド対象オブジェクト
     *
     * @else
     *
     * @brief Bind the specified objects to NamingService
     * 
     * Bind the specified objects to CORBA NamingService by specified names.
     * 
     * @param name Names at the binding
     * @param rtobj The target objects for the binding
     *
     * @endif
     */
    void bindObject(const char* name, const RTObject_impl* rtobj);

    /*!
     * @if jp
     *
     * @brief 指定したManagerServantのNamingServiceへバインド
     * 
     * 指定したManagerServantを指定した名称で CORBA NamingService へバ
     * インドする。
     * 
     * @param name バインド時の名称
     * @param mgr バインド対象ManagerServant
     *
     * @else
     *
     * @brief Bind the specified ManagerServants to NamingService
     * 
     * Bind the specified ManagerServants to CORBA NamingService 
     * by specified names.
     * 
     * @param name Names at the binding
     * @param mgr The target ManagerServants for the binding
     *
     * @endif
     */
    void bindObject(const char* name, const RTM::ManagerServant* mgr);
    
    /*!
     * @if jp
     *
     * @brief NamingServer の情報の更新
     * 
     * 設定されている NameServer 内に登録されているオブジェクトの情報を
     * 更新する。
     * 
     * @else
     *
     * @brief Update information of NamingServer
     * 
     * Update the object information registered in the specified NameServer.
     * 
     * @endif
     */
    void update();
    
    /*!
     * @if jp
     *
     * @brief 指定したオブジェクトをNamingServiceからアンバインド
     * 
     * 指定したオブジェクトを NamingService からアンバインドする。
     * 
     * @param name アンバインド対象オブジェクト
     *
     * @else
     *
     * @brief Unbind the specified objects from NamingService
     * 
     * Unbind the specified objects from NamingService.
     * 
     * @param name The target objects for the unbinding
     *
     * @endif
     */
    void unbindObject(const char* name);
    
    /*!
     * @if jp
     *
     * @brief 全てのオブジェクトをNamingServiceからアンバインド
     * 
     * 全てのオブジェクトを CORBA NamingService からアンバインドする。
     * 
     * @else
     *
     * @brief Unbind all objects from NamingService
     * 
     * Unbind all objects from CORBA NamingService.
     * 
     * @endif
     */
    void unbindAll();
    
    /*!
     * @if jp
     *
     * @brief バインドされている全てのオブジェクトを取得
     * 
     * バインドされている全てのオブジェクトを 取得する。
     *
     * @return バインド済みオブジェクト リスト
     * 
     * @else
     *
     * @brief Get all bound objects
     * 
     * Get all bound objects.
     *
     * @return Bound object list
     * 
     * @endif
     */
    std::vector<RTObject_impl*> getObjects();
    
  protected:
    /*!
     * @if jp
     *
     * @brief NameServer 管理用オブジェクトの生成
     * 
     * 指定した型のNameServer 管理用オブジェクトを生成する。
     *
     * @param method NamingService 形式
     * @param name_server NameServer 名称
     * 
     * @return 生成した NameServer オブジェクト
     * 
     * @else
     *
     * @brief Create objects for NameServer management
     * 
     * Create objects of specified type for NameServer management.
     *
     * @param method NamingService format
     * @param name_server NameServer name
     * 
     * @return Created NameServer objects
     * 
     * @endif
     */
    NamingBase* createNamingObj(const char* method, const char* name_server);
    
    /*!
     * @if jp
     *
     * @brief 設定済みコンポーネントを NameServer に登録
     * 
     * 設定済みコンポーネントを指定した NameServer に登録する。
     *
     * @param ns 登録対象 NameServer
     * 
     * @else
     *
     * @brief Register the configured component to NameServer
     * 
     * Register the already configured components to NameServer.
     *
     * @param ns The target NameServer for the registration
     * 
     * @endif
     */
    void bindCompsTo(NamingBase* ns);
    
    /*!
     * @if jp
     *
     * @brief NameServer に登録するコンポーネントの設定
     * 
     * NameServer に登録するコンポーネントを設定する。
     *
     * @param name コンポーネントの登録時名称
     * @param rtobj 登録対象オブジェクト
     * 
     * @else
     *
     * @brief Configure the components that will be registered to NameServer
     * 
     * Configure the components that will be registered to NameServer.
     *
     * @param name Names of components at the registration
     * @param rtobj The target objects for registration
     * 
     * @endif
     */
    void registerCompName(const char* name, const RTObject_impl* rtobj);

    /*!
     * @if jp
     *
     * @brief NameServer に登録するManagerServantの設定
     * 
     * NameServer に登録するManagerServantを設定する。
     *
     * @param name ManagerServantの登録時名称
     * @param mgr 登録対象ManagerServant
     * 
     * @else
     *
     * @brief Configure the ManagerServants that will be registered 
     * to NameServer
     * 
     * Configure the ManagerServants that will be registered to NameServer.
     *
     * @param name Names of ManagerServants at the registration
     * @param mgr The target ManagerServants for registration
     * 
     * @endif
     */
    void registerMgrName(const char* name, const RTM::ManagerServant* mgr);
    
    /*!
     * @if jp
     *
     * @brief NameServer に登録するコンポーネントの設定解除
     * 
     * NameServer に登録するコンポーネントの設定を解除する。
     *
     * @param name 設定解除対象コンポーネントの名称
     * 
     * @else
     *
     * @brief Unregister the components that will be registered to NameServer
     * 
     * Unregister the components that will be registered to NameServer.
     *
     * @param name Names of the target components for unregistration
     * 
     * @endif
     */
    void unregisterCompName(const char* name);

    /*!
     * @if jp
     *
     * @brief NameServer に登録するManagerServantの設定解除
     * 
     * NameServer に登録するManagerServantの設定を解除する。
     *
     * @param name 設定解除対象ManagerServantの名称
     * 
     * @else
     *
     * @brief Unregister the ManagerServants that will be registered 
     * to NameServer
     * 
     * Unregister the ManagerServants that will be registered to NameServer.
     *
     * @param name Names of the target ManagerServants for unregistration
     * 
     * @endif
     */
    void unregisterMgrName(const char* name);

    /*!
     * @if jp
     *
     * @brief コンポネントをリバインドする
     * 
     * ネームサーバと接続してコンポネントをリバインドする。
     *
     * @param ns NameServer
     * 
     * @else
     *
     * @brief Rebind the component to NameServer
     * 
     * Connect with the NameServer and rebind the component. 
     *
     * @param ns NameServer
     * 
     * @endif
     */
    class Names;
    void retryConnection(Names* ns);
    
  protected:
    // Name Servers' method/name and object
    /*!
     * @if jp
     * @brief NameServer 管理用構造体
     * @else
     * @brief Structure for NameServer management
     * @endif
     */
    class Names
    {
    public:
      Names(const char* meth, const char* name, NamingBase* naming)
	: method(meth), nsname(name), ns(naming)
      {
      }
      
      ~Names()
      {
        delete ns;
      }
      
      std::string method;
      std::string nsname;
      NamingBase* ns;
    };
    /*!
     * @if jp
     * @brief NameServer リスト
     * @else
     * @brief NameServer list
     * @endif
     */
    std::vector<Names*> m_names;
    /*!
     * @if jp
     * @brief NameServer リストのmutex
     * @else
     * @brief Mutex of NameServer list
     * @endif
     */
    Mutex m_namesMutex;
    
    // Components' name and object
    /*!
     * @if jp
     * @brief コンポーネント管理用構造体
     * @else
     * @brief Structure for component management
     * @endif
     */
    struct Comps
    {
      Comps(const char* n, const RTObject_impl* obj)
	: name(n), rtobj(obj)
      {}
      std::string name;
      const RTObject_impl* rtobj;
    };
    /*!
     * @if jp
     * @brief ManagerServant管理用構造体
     * @else
     * @brief Structure for ManagerServant management
     * @endif
     */
    struct Mgr
    {
      Mgr(const char* n, const RTM::ManagerServant* obj)
	: name(n), mgr(obj)
      {}
      std::string name;
      const RTM::ManagerServant* mgr;
    };
    /*!
     * @if jp
     * @brief コンポーネントリスト
     * @else
     * @brief Component list
     * @endif
     */
    std::vector<Comps*> m_compNames;
    /*!
     * @if jp
     * @brief コンポーネントリストのmutex
     * @else
     * @brief Mutex of Component list
     * @endif
     */
    Mutex m_compNamesMutex;
    /*!
     * @if jp
     * @brief ManagerServantリスト
     * @else
     * @brief ManagerServant list
     * @endif
     */
    std::vector<Mgr*> m_mgrNames;
    /*!
     * @if jp
     * @brief ManagerServantリストのmutex
     * @else
     * @brief Mutex of ManagerServant list
     * @endif
     */
    Mutex m_mgrNamesMutex;
    
    /*!
     * @if jp
     * @brief マネージャオブジェクト
     * @else
     * @brief Manager object
     * @endif
     */
    Manager* m_manager;
    
    /*!
     * @if jp
     * @brief ロガーストリーム
     * @else
     * @brief Logger stream
     * @endif
     */
    Logger rtclog;
  }; // class NamingManager
}; // namespace RTC

#endif // RTC_NAMINGMANAGER_H
