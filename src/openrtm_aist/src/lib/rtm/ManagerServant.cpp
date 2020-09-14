// -*- C++ -*-
/*!
 * @file ManagerServant.cpp
 * @brief RTComponent manager servant implementation class
 * @date $Date: 2007-12-31 03:08:04 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2008-2010
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
#include <coil/Process.h>
#include <rtm/Manager.h>
#include <rtm/ManagerServant.h>
#include <rtm/NVUtil.h>
#include <rtm/RTObject.h>
#include <rtm/CORBA_SeqUtil.h>
#include <rtm/CORBA_IORUtil.h>

namespace RTM
{
  //
  // Example implementational code for IDL interface RTM::Manager
  //
  ManagerServant::ManagerServant()
    : m_mgr(::RTC::Manager::instance())
  {
    rtclog.setName("ManagerServant");
    coil::Properties config(m_mgr.getConfig());    
    
    if (coil::toBool(config["manager.is_master"], "YES", "NO", true))
      { // this is master manager
        RTC_TRACE(("This manager is master."));

        if (!createINSManager())
          {
            RTC_WARN(("Manager CORBA servant creation failed."));
            return;
            
          }
        m_isMaster = true;
        RTC_WARN(("Manager CORBA servant was successfully created."));
        return;
      }
    else
      { // manager is slave
        RTC_TRACE(("This manager is slave."));
        try
          {
            RTM::Manager_var owner;
            owner = findManager(config["corba.master_manager"].c_str());
            if (CORBA::is_nil(owner))
              {
                RTC_INFO(("Master manager not found"));
                return;
              }
            if (!createINSManager())
              {
                RTC_WARN(("Manager CORBA servant creation failed."));
                return;
              }
            add_master_manager(owner);
            owner->add_slave_manager(m_objref.in());
            return;
          }
        catch (...)
          {
            RTC_ERROR(("Unknown exception cought."));
          }
      }
  }
  
  ManagerServant::~ManagerServant()
  {
    Guard guardm(m_masterMutex);
    for (CORBA::ULong i(0); i < m_masters.length(); ++i)
      {
        try
          {
            if (CORBA::is_nil(m_masters[i])) { continue; }
            m_masters[i]
              ->remove_slave_manager(RTM::Manager::_duplicate(m_objref));
          }
        catch (...)
          {
            m_masters[i] = RTM::Manager::_nil();
          }
      }
    m_masters.length(0);

    Guard guards(m_slaveMutex);
    for (CORBA::ULong i(0); i < m_slaves.length(); ++i)
      {
        try
          {
            if (CORBA::is_nil(m_slaves[i])) { continue; }
            m_slaves[i]
              ->remove_master_manager(RTM::Manager::_duplicate(m_objref));
          }
        catch (...)
          {
            m_slaves[i] = RTM::Manager::_nil();
          }
      }
    m_slaves.length(0);

  }
  
  /*!
   * @if jp
   * @brief モジュールをロードする
   * @else
   * @brief Loading a module
   * @endig
   */
  RTC::ReturnCode_t
  ManagerServant::load_module(const char* pathname, const char* initfunc)
  {
    RTC_TRACE(("ManagerServant::load_module(%s, %s)", pathname, initfunc));

    m_mgr.load(pathname, initfunc);
    
    return ::RTC::RTC_OK;
  }
  
  /*!
   * @if jp
   * @brief モジュールをアンロードする
   * @else
   * @brief Unloading a module
   * @endig
   */  
  RTC::ReturnCode_t ManagerServant::unload_module(const char* pathname)
  {
    RTC_TRACE(("ManagerServant::unload_module(%s)", pathname));
    
    m_mgr.unload(pathname);
    
    return ::RTC::RTC_OK;
  }
  
  /*!
   * @if jp
   * @brief ロード可能なモジュールのプロファイルを取得する
   * @else
   * @brief Getting loadable module profiles
   * @endig
   */
  RTM::ModuleProfileList* ManagerServant::get_loadable_modules()
  {
    RTC_TRACE(("get_loadable_modules()"));
    
    // copy local module profiles
    ::RTM::ModuleProfileList_var cprof = new ::RTM::ModuleProfileList();
    std::vector<coil::Properties> prof(m_mgr.getLoadableModules());

    cprof->length((CORBA::Long)prof.size());
    for (int i(0), len(prof.size()); i < len; ++i)
      {
        RTC_VERBOSE_STR((prof[i]));
        NVUtil::copyFromProperties(cprof[(CORBA::Long)i].properties, prof[i]);
      }

    if (0)
      {
        // copy slaves' module profiles
        Guard gurad(m_slaveMutex);
        RTC_DEBUG(("%d slaves exists.", m_slaves.length()));
        for (int i(0), len(m_slaves.length()); i < len; ++i)
          {
            try
              {
                if (!CORBA::is_nil(m_slaves[i]))
                  {
                    ::RTM::ModuleProfileList_var sprof;
                    sprof = m_slaves[i]->get_loadable_modules();
#ifndef ORB_IS_RTORB
                    CORBA_SeqUtil::push_back_list(cprof.inout(), sprof.in());
#else // ORB_IS_RTORB
                    CORBA_SeqUtil::push_back_list(cprof, sprof);
#endif // ORB_IS_RTORB
                    continue; 
                  }
              }
            catch (...)
              {
            RTC_INFO(("slave (%d) has disappeared.", i));
            m_slaves[i] = RTM::Manager::_nil();
              }
            CORBA_SeqUtil::erase(m_slaves, i); --i;
          }
      }
    return cprof._retn();
  }
  
  /*!
   * @if jp
   * @brief ロード済みのモジュールのプロファイルを取得する
   * @else
   * @brief Getting loaded module profiles
   * @endig
   */
  RTM::ModuleProfileList* ManagerServant::get_loaded_modules()
  {
    RTC_TRACE(("get_loaded_modules()"));
    
    // copy local module profiles
    ::RTM::ModuleProfileList_var cprof = new RTM::ModuleProfileList();
    std::vector<coil::Properties> prof(m_mgr.getLoadedModules());
    
    cprof->length(prof.size());
    for (int i(0), len(prof.size()); i < len; ++i)
      {
        RTC_VERBOSE_STR((prof[i]));
        NVUtil::copyFromProperties(cprof[(CORBA::Long)i].properties, prof[i]);
      }

    if (0)
      {
        // copy slaves' module profile
        Guard guard(m_slaveMutex);
        RTC_DEBUG(("%d slave managers exists.", m_slaves.length()));
        for (int i(0), len(m_slaves.length()); i < len; ++i)
          {
            try
              {
                if (!CORBA::is_nil(m_slaves[i]))
                  {
                    ::RTM::ModuleProfileList_var sprof;
                    sprof = m_slaves[i]->get_loaded_modules();
#ifndef ORB_IS_RTORB
                    CORBA_SeqUtil::push_back_list(cprof.inout(), sprof.in());
#else // ORB_IS_RTORB
                    CORBA_SeqUtil::push_back_list(cprof, sprof);
#endif // ORB_IS_RTORB
                    continue;
                  }
              }
            catch (...)
              {
                RTC_INFO(("slave (%d) has disappeared.", i));
                m_slaves[i] = RTM::Manager::_nil();
              }
            CORBA_SeqUtil::erase(m_slaves, i); --i;
          }
      }
    return cprof._retn();
  }
  
  /*!
   * @if jp
   * @brief コンポーネントファクトリのプロファイルを取得する
   * @else
   * @brief Getting component factory profiles
   * @endig
   */
  RTM::ModuleProfileList* ManagerServant::get_factory_profiles()
  {
    RTC_TRACE(("get_factory_profiles()"));

    // copy local factory profiles
    ::RTM::ModuleProfileList_var cprof = new RTM::ModuleProfileList();
    std::vector<coil::Properties> prof(m_mgr.getFactoryProfiles());
    
    cprof->length(prof.size());
    for (int i(0), len(prof.size()); i < len; ++i)
      {
        RTC_VERBOSE_STR((prof[i]));
        NVUtil::copyFromProperties(cprof[(CORBA::Long)i].properties, prof[i]);
      }

    if (0)
      {
        // copy slaves' factory profile
        Guard guard(m_slaveMutex);
        RTC_DEBUG(("%d slave managers exists.", m_slaves.length()));
        for (int i(0), len(m_slaves.length()); i < len; ++i)
          {
            try
              {
                if (!CORBA::is_nil(m_slaves[i]))
                  {
                    ::RTM::ModuleProfileList_var sprof;
                    sprof = m_slaves[i]->get_factory_profiles();
#ifndef ORB_IS_RTORB
                    CORBA_SeqUtil::push_back_list(cprof.inout(), sprof.in());
#else // ORB_IS_RTORB
                    CORBA_SeqUtil::push_back_list(cprof, sprof);
#endif // ORB_IS_RTORB
                    continue;
                  }
              }
            catch (...)
              {
                RTC_INFO(("slave (%d) has disappeared.", i));
                m_slaves[i] = RTM::Manager::_nil();
              }
            CORBA_SeqUtil::erase(m_slaves, i); --i;
          }
      }
    return cprof._retn();
  }
  
  /*!
   * @if jp
   * @brief コンポーネントを生成する
   * @else
   * @brief Creating an RT-Component
   * @endig
   */
  RTC::RTObject_ptr ManagerServant::create_component(const char* module_name)
  {
    RTC_TRACE(("create_component(%s)", module_name));
    
    std::string arg(module_name);
    std::string::size_type pos0(arg.find("&manager="));
    std::string::size_type pos1(arg.find("?manager="));

    if (pos0 == std::string::npos && pos1 == std::string::npos)
      {
        if (false) //is_master())
          {
            RTC_ERROR(("Master manager cannot create component: %s",
                       module_name));
            return RTC::RTObject::_nil();
          }
        // create on this manager
        RTC::RTObject_impl* rtc = m_mgr.createComponent(module_name);
        if (rtc == NULL)
          {
            return RTC::RTObject::_nil();
          }
        return RTC::RTObject::_duplicate(rtc->getObjRef());
      }
    // create other manager

    // extract manager's location
    std::string::size_type pos;
    pos = (pos0 != std::string::npos) ? pos0 : pos1;
    
    std::string::size_type endpos;
    endpos = arg.find('&', pos + 1);
    std::string mgrstr(arg.substr(pos + 1, endpos - 1 - pos));
    RTC_VERBOSE(("Manager arg: %s", mgrstr.c_str()));
    coil::vstring mgrvstr(coil::split(mgrstr, ":"));
    if (mgrvstr.size() != 2)
      {
        RTC_WARN(("Invalid manager name: %s", mgrstr.c_str()));
        return RTC::RTObject::_nil();
      }
    std::string::size_type eqpos(mgrstr.find("="));
    if (eqpos == std::string::npos)
      {
        RTC_WARN(("Invalid argument: %s", module_name));
        return RTC::RTObject::_nil();
      }
    mgrstr.erase(0, eqpos + 1);
    RTC_DEBUG(("Manager is %s", mgrstr.c_str()))

    // find manager
    RTM::Manager_var mgrobj = findManager(mgrstr.c_str());
    if (CORBA::is_nil(mgrobj))
      {
        std::string cmd("rtcd -p ");
        cmd += mgrvstr[1]; // port number

        RTC_DEBUG(("Invoking command: %s.", cmd.c_str()));
        int ret(coil::launch_shell(cmd.c_str()));
        if (ret == -1)
          {
            RTC_DEBUG(("%s: failed", cmd.c_str()));
            return RTC::RTObject::_nil();
          }

        // find manager
        coil::usleep(10000);
        int count(0);
        while (CORBA::is_nil(mgrobj))
          {
            mgrobj = findManager(mgrstr.c_str());
            ++count;
            if (count > 1000) { break; }
            coil::usleep(10000);
          }
      }

    if (CORBA::is_nil(mgrobj))
      {
        RTC_WARN(("Manager cannot be found."));
        return RTC::RTObject::_nil();
      }
    
    // create component on the manager    
    arg.erase(pos + 1, endpos - pos);
    RTC_DEBUG(("Creating component on %s",  mgrstr.c_str()));
    RTC_DEBUG(("arg: %s", arg.c_str()));
    try
      {
        RTC::RTObject_var rtobj;
        rtobj = mgrobj->create_component(arg.c_str());
        RTC_DEBUG(("Component created %s",  arg.c_str()));
        return rtobj._retn();
      }
    catch (CORBA::SystemException& e)
      {
        RTC_DEBUG(("Exception was caught while creating component."));
        return RTC::RTObject::_nil();
      }
    return RTC::RTObject::_nil();
  }
  
  /*!
   * @if jp
   * @brief コンポーネントを削除する
   * @else
   * @brief Deleting an RT-Component
   * @endig
   */
  RTC::ReturnCode_t ManagerServant::delete_component(const char* instance_name)
  {
    RTC_TRACE(("delete_component(%s)", instance_name));
    
    m_mgr.deleteComponent(instance_name);
    return ::RTC::RTC_OK;
  }
  
  /*!
   * @if jp
   * @brief 起動中のコンポーネントのリストを取得する
   * @else
   * @brief Getting RT-Component list running on this manager
   * @endig
   */
  RTC::RTCList* ManagerServant::get_components()
  {
    RTC_TRACE(("get_components()"));

    // get local component references
    std::vector<RTC::RTObject_impl*> rtcs = m_mgr.getComponents();
    ::RTC::RTCList_var crtcs = new ::RTC::RTCList();

    crtcs->length((CORBA::Long)rtcs.size());
    for (int i(0), len(rtcs.size()); i < len; ++i)
      {
        crtcs[(CORBA::Long)i] = RTC::RTObject::_duplicate(rtcs[i]->getObjRef());
      }

    // get slaves' component references
    RTC_DEBUG(("%d slave managers exists.", m_slaves.length()));
    for (int i(0), len(m_slaves.length()); i < len; ++i)
      {
        try
          {
            if (!CORBA::is_nil(m_slaves[i]))
              {
                ::RTC::RTCList_var srtcs;
                srtcs = m_slaves[i]->get_components();
#ifndef ORB_IS_RTORB
                CORBA_SeqUtil::push_back_list(crtcs.inout(), srtcs.in());
#else // ORB_IS_RTORB
                CORBA_SeqUtil::push_back_list(srtcs, crtcs);
#endif // ORB_IS_RTORB
                continue;
              }
          }
        catch (...)
          {
            RTC_INFO(("slave (%d) has disappeared.", i));
            m_slaves[i] = RTM::Manager::_nil();
          }
        CORBA_SeqUtil::erase(m_slaves, i); --i;
      }
    return crtcs._retn();
  }
  
  /*!
   * @if jp
   * @brief 起動中のコンポーネントプロファイルのリストを取得する
   * @else
   * @brief Getting RT-Component's profile list running on this manager
   * @endig
   */  
  RTC::ComponentProfileList* ManagerServant::get_component_profiles()
  {
    RTC_TRACE(("get_component_profiles()"));

    // copy local component profiles
    ::RTC::ComponentProfileList_var cprofs = new ::RTC::ComponentProfileList();
    std::vector<RTC::RTObject_impl*> rtcs = m_mgr.getComponents();
    cprofs->length(rtcs.size());
    for (int i(0), len(rtcs.size()); i < len; ++i)
      {
        ::RTC::ComponentProfile_var prof = rtcs[i]->get_component_profile();
        cprofs[(CORBA::Long)i] = prof;
      }

    // copy slaves' component profiles
    Guard guard(m_slaveMutex);
    RTC_DEBUG(("%d slave managers exists.", m_slaves.length()));
    for (int i(0), len(m_slaves.length()); i < len; ++i)
      {
        try
          {
            if (!CORBA::is_nil(m_slaves[i]))
              {
                ::RTC::ComponentProfileList_var sprofs;
                sprofs = m_slaves[i]->get_component_profiles();
#ifndef ORB_IS_RTORB
                CORBA_SeqUtil::push_back_list(cprofs.inout(), sprofs.in());
#else // ORB_IS_RTORB
                CORBA_SeqUtil::push_back_list(cprofs, sprofs);
#endif // ORB_IS_RTORB
                continue;
              }
          }
        catch (...)
          {
            RTC_INFO(("slave (%d) has disappeared.", i));
            m_slaves[i] = RTM::Manager::_nil();
          }
        CORBA_SeqUtil::erase(m_slaves, i); --i;
      }
    return cprofs._retn();
  }
  
  // manager 基本
  /*!
   * @if jp
   * @brief マネージャのプロファイルを取得する
   * @else
   * @brief Getting this manager's profile.
   * @endig
   */
  RTM::ManagerProfile* ManagerServant::get_profile()
  {
    RTC_TRACE(("get_profile()"));
    RTM::ManagerProfile* prof = new RTM::ManagerProfile();
    NVUtil::copyFromProperties(prof->properties,
                               m_mgr.getConfig().getNode("manager"));
    return prof;
  }
  
  /*!
   * @if jp
   * @brief マネージャのコンフィギュレーションを取得する
   * @else
   * @brief Getting this manager's configuration.
   * @endig
   */
  RTM::NVList* ManagerServant::get_configuration()
  {
    RTC_TRACE(("get_configuration()"));
#ifndef ORB_IS_RTORB
    ::RTC::NVList* nvlist = new ::RTC::NVList();
    NVUtil::copyFromProperties(*nvlist, m_mgr.getConfig());
#else
    ::RTC::NVList* nvlist;
    RTC_NVList nvlist_in;
    NVUtil::copyFromProperties(nvlist_in, m_mgr.getConfig());
    nvlist = new ::RTC::NVList(nvlist_in);
#endif
    return nvlist;
  }
  
  /*!
   * @if jp
   * @brief マネージャのコンフィギュレーションを設定する
   * @else
   * @brief Setting manager's configuration
   * @endig
   */
  RTC::ReturnCode_t
  ManagerServant::set_configuration(const char* name, const char* value)
  {
    RTC_TRACE(("set_configuration(name = %s, value = %s)", name, value));
    m_mgr.getConfig().setProperty(name, value);
    return ::RTC::RTC_OK;
  }
  /*!
   * @if jp
   * @brief マネージャがマスターかどうか
   * @else
   * @brief Whether this manager is master or not
   * @endig
   */
  CORBA::Boolean ManagerServant::is_master()
  {
    RTC_TRACE(("is_master(): %s", m_isMaster ? "YES" : "NO"));
    return m_isMaster;
  }
  
  /*!
   * @if jp
   * @brief マスターマネージャの取得
   * @else
   * @brief Getting master managers
   * @endig
   */
  RTM::ManagerList* ManagerServant::get_master_managers()
  {
    RTC_TRACE(("get_master_managers()"));
    Guard guard(m_masterMutex);
    return new ManagerList(m_masters);
  }
  
  /*!
   * @if jp
   * @brief マスターマネージャの追加
   * @else
   * @brief Getting a master manager
   * @endig
   */
  RTC::ReturnCode_t ManagerServant::add_master_manager(RTM::Manager_ptr mgr)
  {
    Guard guard(m_masterMutex);
    CORBA::Long index;
    RTC_TRACE(("add_master_manager(), %d masters", m_masters.length()));
    index = CORBA_SeqUtil::find(m_masters, is_equiv(mgr));
    
    if (!(index < 0)) // found in my list
      {
        RTC_ERROR(("Already exists."));
        return RTC::BAD_PARAMETER;
      }
    
    CORBA_SeqUtil::push_back(m_masters, RTM::Manager::_duplicate(mgr));
    RTC_TRACE(("add_master_manager() done, %d masters", m_masters.length()));
    return RTC::RTC_OK;
  }
  
  /*!
   * @if jp
   * @brief マスターマネージャの削除
   * @else
   * @brief Removing a master manager
   * @endig
   */
  RTC::ReturnCode_t
  ManagerServant::remove_master_manager(RTM::Manager_ptr mgr)
  {
    Guard guard(m_masterMutex);
    RTC_TRACE(("remove_master_manager(), %d masters", m_masters.length()));

    CORBA::Long index;
    index = CORBA_SeqUtil::find(m_masters, is_equiv(mgr));
    
    if (index < 0) // not found in my list
      {
        RTC_ERROR(("Not found."));
        return RTC::BAD_PARAMETER;
      }
    
    CORBA_SeqUtil::erase(m_masters, index);
    RTC_TRACE(("remove_master_manager() done, %d masters", m_masters.length()));
    return RTC::RTC_OK;
  }
  
  /*!
   * @if jp
   * @brief スレーブマネージャの取得
   * @else
   * @brief Getting slave managers
   * @endig
   */
  ManagerList* ManagerServant::get_slave_managers()
  {
    Guard guard(m_slaveMutex);
    RTC_TRACE(("get_slave_managers(), %d slaves", m_slaves.length()));
    
    return new ManagerList(m_slaves);
  }

  /*!
   * @if jp
   * @brief スレーブマネージャの追加
   * @else
   * @brief Getting a slave manager
   * @endig
   */
  RTC::ReturnCode_t ManagerServant::add_slave_manager(RTM::Manager_ptr mgr)
  {
    Guard guard(m_slaveMutex);
    RTC_TRACE(("add_slave_manager(), %d slaves", m_slaves.length()));
    
    CORBA::Long index;
    index = CORBA_SeqUtil::find(m_slaves, is_equiv(mgr));
    
    if (!(index < 0)) // found in my list
      {
        RTC_ERROR(("Already exists."));
        return RTC::BAD_PARAMETER;
      }
    
    CORBA_SeqUtil::push_back(m_slaves, RTM::Manager::_duplicate(mgr));
    RTC_TRACE(("add_slave_manager() done, %d slaves", m_slaves.length()));
    return RTC::RTC_OK;;
  }
  
  /*!
   * @if jp
   * @brief スレーブマネージャの削除
   * @else
   * @brief Removing a slave manager
   * @endig
   */
  RTC::ReturnCode_t ManagerServant::remove_slave_manager(RTM::Manager_ptr mgr)
  {
    Guard guard(m_slaveMutex);
    RTC_TRACE(("remove_slave_manager(), %d slaves", m_slaves.length()));
    CORBA::Long index;
    index = CORBA_SeqUtil::find(m_slaves, is_equiv(mgr));
    
    if (index < 0) // not found in my list
      {
        RTC_ERROR(("Not found."));
        return RTC::BAD_PARAMETER;
      }
    
    CORBA_SeqUtil::erase(m_slaves, index);
    RTC_TRACE(("remove_slave_manager() done, %d slaves", m_slaves.length()));
    return RTC::RTC_OK;
  }
  
  
  
  RTC::ReturnCode_t ManagerServant::fork()
  {
    //    m_mgr.fork();
    return ::RTC::RTC_OK;
  }
  
  RTC::ReturnCode_t ManagerServant::shutdown()
  {
    m_mgr.terminate();
    return ::RTC::RTC_OK;
  }
  
  RTC::ReturnCode_t ManagerServant::restart()
  {
    //    m_mgr.restart();
    return ::RTC::RTC_OK;
  }
  
  CORBA::Object_ptr ManagerServant::get_service(const char* name)
  {
    return ::CORBA::Object::_nil();
  }
  
  RTM::Manager_ptr ManagerServant::getObjRef() const
  {
    return m_objref;
  }


  bool ManagerServant::createINSManager()
  {
    try
      {
        //Ppreparing INS POA
        CORBA::Object_var obj;
#ifndef ORB_IS_RTORB
        obj = m_mgr.getORB()->resolve_initial_references("omniINSPOA");
#else // ROB_IS_RTORB
        obj = m_mgr.getORB()->resolve_initial_references((char*)"omniINSPOA");
#endif // ORB_IS_RTORB
        PortableServer::POA_ptr poa = PortableServer::POA::_narrow(obj);
        poa->the_POAManager()->activate();

        // Create readable object ID
        coil::Properties config(m_mgr.getConfig());
        PortableServer::ObjectId_var id; 
#ifndef ORB_IS_RTORB
        id = PortableServer::string_to_ObjectId(config["manager.name"].c_str());
#else // ORB_IS_RTORB
        id = PortableServer::
          string_to_ObjectId((char *)config["manager.name"].c_str());
#endif // ORB_IS_RTORB

        // Object activation
        poa->activate_object_with_id(id.in(), this);
        CORBA::Object_var mgrobj = poa->id_to_reference(id);

        // Set m_objref 
        m_objref = ::RTM::Manager::_narrow(mgrobj);

        CORBA::String_var ior;
        ior = m_mgr.getORB()->
          object_to_string(RTM::Manager::_duplicate(m_objref));
        std::string iorstr((const char*)ior);
        RTC_DEBUG(("Manager's IOR information:\n %s",
                   CORBA_IORUtil::formatIORinfo(iorstr.c_str()).c_str()));
      }
    catch (...)
      {
        return false;
      }
    return true;
  }



  RTM::Manager_ptr ManagerServant::findManager(const char* host_port)
  {
    RTC_TRACE(("findManager(host_port = %s)", host_port));
    try
      {
        coil::Properties config(m_mgr.getConfig());
        // Why RtORB does not allow corbaloc:iiop: ?
        //        std::string mgrloc("corbaloc:iiop:");
        std::string mgrloc("corbaloc::");
        mgrloc += host_port;
        mgrloc += "/" + config["manager.name"];

        RTC_DEBUG(("corbaloc: %s", mgrloc.c_str()));

        CORBA::Object_var mobj;
        mobj = m_mgr.getORB()->string_to_object(mgrloc.c_str());
#ifndef ORB_IS_RTORB
        RTM::Manager_var mgr = ::RTM::Manager::_narrow(mobj);
#else // ORB_IS_RTORB
        RTM::Manager_var mgr;
        if(!make_client_connection(mobj->impl()->connection))
          {
            return RTM::Manager_ptr();
          }
        else
          {
            mgr = ::RTM::Manager::_narrow(mobj);
          }
#endif // ORB_IS_RTORB

        CORBA::String_var ior;
        ior = m_mgr.getORB()->object_to_string(RTM::Manager::_duplicate(mgr));
        std::string iorstr((const char*)ior);
        RTC_DEBUG(("Manager's IOR information:\n %s",
                   CORBA_IORUtil::formatIORinfo(iorstr.c_str()).c_str()));
     
        return mgr._retn();
      }
    catch(CORBA::SystemException& e)
      {
#ifndef ORB_IS_RTORB
        RTC_DEBUG(("CORBA SystemException cought (CORBA::%s)", e._name()));
#endif // ORB_IS_RTORB
      }
    catch (...)
      {
        RTC_ERROR(("Unknown exception cought."));
      }
    return RTM::Manager::_nil();
  }

  

};
