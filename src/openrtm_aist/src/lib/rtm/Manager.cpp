// -*- C++ -*-
/*!
 * @file Manager.h
 * @brief RTComponent manager class
 * @date $Date: 2008-03-06 06:58:40 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * Copyright (C) 2003-2005
 *     Task-intelligence Research Group,
 *     Intelligent Systems Research Institute,
 *     National Institute of
 *         Advanced Industrial Science and Technology (AIST), Japan
 *     All rights reserved.
 *
 * $Id$
 *
 */

#include <rtm/Manager.h>
#include <rtm/ManagerConfig.h>
#include <rtm/ModuleManager.h>
#include <rtm/CorbaNaming.h>
#include <rtm/NamingManager.h>
#include <rtm/RTC.h>
#include <rtm/PeriodicExecutionContext.h>
#include <rtm/ExtTrigExecutionContext.h>
#include <rtm/OpenHRPExecutionContext.h>
#include <rtm/PeriodicECSharedComposite.h>
#include <rtm/RTCUtil.h>
#include <rtm/ManagerServant.h>
#include <fstream>
#include <iostream>
#include <coil/Properties.h>
#include <coil/stringutil.h>
#include <coil/Signal.h>
#include <coil/TimeValue.h>
#include <coil/Timer.h>
#include <coil/OS.h>
#include <rtm/FactoryInit.h>
#include <rtm/CORBA_IORUtil.h>
#include <rtm/SdoServiceConsumerBase.h>

#if defined(minor)
#undef minor
#endif

//static sig_atomic_t g_mgrActive = true;
extern "C" void handler (int)
{
  RTC::Manager::instance().terminate();
}

namespace RTC
{
  Manager* Manager::manager = NULL;
  coil::Mutex Manager::mutex;

  Manager::InstanceName::InstanceName(RTObject_impl* comp)
    : m_name(comp->getInstanceName())
  {}
  Manager::InstanceName::InstanceName(const char* name)
    : m_name(name)
  {}
  Manager::InstanceName::InstanceName(const std::string name)
    : m_name(name)
  {}
  bool Manager::InstanceName::operator()(RTObject_impl* comp)
  {
    return m_name == comp->getInstanceName();
  }

  
  /*!
   * @if jp
   * @brief Protected •≥•Û•π•»•È•Ø•ø
   * @else
   * @brief Protected Constructor
   * @endif
   */
  Manager::Manager()
    : m_initProc(0), m_namingManager(0), m_timer(0),
      m_logStreamBuf(), rtclog(&m_logStreamBuf),
      m_runner(0), m_terminator(0)
  {
    new coil::SignalAction((coil::SignalHandler) handler, SIGINT);
  }
  
  /*!
   * @if jp
   * @brief Protected •≥•‘°º•≥•Û•π•»•È•Ø•ø
   * @else
   * @brief Protected Copy Constructor
   * @endif
   */
  Manager::Manager(const Manager& manager)
    : m_initProc(0), m_namingManager(0), m_timer(0),
      m_logStreamBuf(), rtclog(&m_logStreamBuf),
      m_runner(0), m_terminator(0)
  {
    new coil::SignalAction((coil::SignalHandler) handler, SIGINT);
  }
  
  /*!
   * @if jp
   * @brief •ﬁ•Õ°º•∏•„§ŒΩÈ¥ÅEΩ
   * @else
   * @brief Initialize manager
   * @endif
   */
  Manager* Manager::init(int argc, char** argv)
  {
    // DCL for singleton
    if (!manager)
      {
	Guard guard(mutex);
	if (!manager)
	  {
	    manager = new Manager();
	    manager->initManager(argc, argv);
	    manager->initLogger();
	    manager->initORB();
	    manager->initNaming();
            manager->initFactories();
	    manager->initExecContext();
	    manager->initComposite();
	    manager->initTimer();
            manager->initManagerServant();
	  }
      }
    return manager;
  }
  
  /*!
   * @if jp
   * @brief •ﬁ•Õ°º•∏•„§Œ•§•Û•π•ø•Û•π§ŒºË∆¿
   * @else
   * @brief Get instance of the manager
   * @endif
   */ 
  Manager& Manager::instance()
  {
    // DCL for singleton
    if (!manager)
      {
	Guard guard(mutex);
	if (!manager)
	  {
	    manager = new Manager();
	    manager->initManager(0, NULL);
	    manager->initLogger();
	    manager->initORB();
	    manager->initNaming();
            manager->initFactories();
	    manager->initExecContext();
	    manager->initComposite();
	    manager->initTimer();
	  }
      }
    return *manager;
  }
  
  /*!
   * @if jp
   * @brief •ﬁ•Õ°º•∏•„Ω™ŒªΩËÕ˝
   * @else
   * @brief Terminate Manager
   * @endif
   */ 
  void Manager::terminate()
  {
    if (m_terminator != NULL)
      m_terminator->terminate();
  }
  
  /*!
   * @if jp
   * @brief •ﬁ•Õ°º•∏•„°¶•∑•„•√•»•¿•¶•ÅE   * @else
   * @brief Shutdown Manager
   * @endif
   */
  void Manager::shutdown()
  {
    RTC_TRACE(("Manager::shutdown()"));
    shutdownComponents();
    shutdownNaming();
    shutdownORB();
    shutdownManager();
    // Ω™Œª¬‘§¡πÁ§ÅEª
    if (m_runner != NULL)
      {
	m_runner->wait();
      }
    else
      {
	join();
      }
    shutdownLogger();
  }
  
  /*!
   * @if jp
   * @brief •ﬁ•Õ°º•∏•„Ω™ŒªΩËÕ˝§Œ¬‘§¡πÁ§ÅEª
   * @else
   * @brief Wait for Manager's termination
   * @endif
   */ 
  void Manager::join()
  {
    RTC_TRACE(("Manager::wait()"));
    {
      Guard guard(m_terminate.mutex);
      ++m_terminate.waiting;
    }
    while (1)
      {
	{
	  Guard guard(m_terminate.mutex);
	  if (m_terminate.waiting > 1) break;
	}
        coil::usleep(100000);
      }
  }
  
  /*!
   * @if jp
   * @brief ΩÈ¥ÅEΩ•◊•˙¡∑°º•∏•„§Œ•ª•√•»
   * @else
   * @brief Set initial procedure
   * @endif
   */ 
  void Manager::setModuleInitProc(ModuleInitProc proc)
  {
    m_initProc = proc;
  }
  
  /*!
   * @if jp
   * @brief Manager§Œ•¢•Ø•∆•£•÷≤Ω
   * @else
   * @brief Activate the Manager
   * @endif
   */ 
  bool Manager::activateManager()
  {
    RTC_TRACE(("Manager::activateManager()"));
    
    try
      {
        if(CORBA::is_nil(this->getPOAManager()))
        {
          RTC_ERROR(("Could not get POA manager."));
          return false;
        }
	this->getPOAManager()->activate();
        RTC_TRACE(("POA Manager activated."));
      }
    catch (...)
      {
        RTC_ERROR(("POA Manager activatatin failed."));
	return false;
      }

    std::vector<std::string> mods;
    mods = coil::split(m_config["manager.modules.preload"], ",");

    for (int i(0), len(mods.size()); i < len; ++i)
      {
	size_t begin_pos(mods[i].find_first_of('('));
	size_t end_pos(mods[i].find_first_of(')'));
	std::string filename, initfunc;
	if (begin_pos != std::string::npos && end_pos != std::string::npos &&
	    begin_pos < end_pos)
	  {
	    initfunc = mods[i].substr(begin_pos + 1, end_pos - (begin_pos + 1));
	    filename = mods[i].substr(0, begin_pos);
	    coil::eraseBothEndsBlank(initfunc);
	    coil::eraseBothEndsBlank(filename);
	  }
	else
	  {
	    initfunc = coil::split(mods[i], ".").operator[](0) + "Init";
	    filename = mods[i];
	  }
	if (filename.find_first_of('.') == std::string::npos)
	  {
	    std::cout <<  m_config["manager.modules.C++.suffixes"] << std::endl;
	    if (m_config.findNode("manager.modules.C++.suffixes") != 0)
	      {
		filename += "." + m_config["manager.modules.C++.suffixes"];
	      }
	  }
	try
	  {
	    m_module->load(mods[i], initfunc);
	  }
	catch (ModuleManager::Error& e)
	  {
	    RTC_ERROR(("Module load error: %s", e.reason.c_str()));
	  }
	catch (ModuleManager::SymbolNotFound& e)
	  {
	    RTC_ERROR(("Symbol not found: %s", e.name.c_str()));
	  }
	catch (ModuleManager::ModuleNotFound& e)
	  {
	    RTC_ERROR(("Module not found: %s", e.name.c_str()));
	  }
	catch (...)
	  {
	    RTC_ERROR(("Unknown Exception"));
	  }
      }

    m_config["sdo.service.consumer.available_services"]
      = coil::flatten(SdoServiceConsumerFactory::instance().getIdentifiers());

    if (m_initProc != NULL)
      {
        m_initProc(this);
      }

    RTC_TRACE(("Components pre-creation: %s",
               m_config["manager.components.precreate"].c_str()));
    std::vector<std::string> comp;
    comp = coil::split(m_config["manager.components.precreate"], ",");
    for (int i(0), len(comp.size()); i < len; ++i)
      {
        this->createComponent(comp[i].c_str());
      }

    { // pre-connection
      RTC_TRACE(("Connection pre-creation: %s",
                 m_config["manager.components.preconnect"].c_str()));
      std::vector<std::string> connectors;
      connectors = coil::split(m_config["manager.components.preconnect"], ",");
      for (int i(0), len(connectors.size()); i < len; ++i)
        {
          // ConsoleIn.out:Console.in(dataflow_type=push,....)
          coil::vstring conn_prop = coil::split(connectors[i], "(");
          coil::replaceString(conn_prop[1], ")", "");
          coil::vstring comp_ports;
          comp_ports = coil::split(conn_prop[0], ":");
          if (comp_ports.size() != 2)
            {
              RTC_ERROR(("Invalid format for pre-connection."));
              RTC_ERROR(("Format must be Comp0.port0:Comp1.port1"));
              continue;
            }
          std::string comp0_name = coil::split(comp_ports[0], ".")[0];
          std::string comp1_name = coil::split(comp_ports[1], ".")[0];
          RTObject_impl* comp0 = getComponent(comp0_name.c_str());
          RTObject_impl* comp1 = getComponent(comp1_name.c_str());
          if (comp0 == NULL)
            { RTC_ERROR(("%s not found.", comp0_name.c_str())); continue; }
          if (comp1 == NULL)
            { RTC_ERROR(("%s not found.", comp1_name.c_str())); continue; }
          std::string port0 = comp_ports[0];
          std::string port1 = comp_ports[1];
          
          PortServiceList_var ports0 = comp0->get_ports();
          PortServiceList_var ports1 = comp1->get_ports();
          RTC_DEBUG(("%s has %d ports.", comp0_name.c_str(), ports0->length()));
          RTC_DEBUG(("%s has %d ports.", comp1_name.c_str(), ports1->length()));
          
          PortService_var port0_var;
          for (size_t p(0); p < ports0->length(); ++p)
            {
              PortProfile_var pp = ports0[p]->get_port_profile();
              std::string s(CORBA::string_dup(pp->name));
              if (comp_ports[0] == s)
                {
                  RTC_DEBUG(("port %s found: ", comp_ports[0].c_str()));
                  port0_var = ports0[p];
                }
            }
          PortService_var port1_var;
          for (size_t p(0); p < ports1->length(); ++p)
            {
              PortProfile_var pp = ports1[p]->get_port_profile();
              std::string s(CORBA::string_dup(pp->name));
              if (port1 == s)
                {
                  RTC_DEBUG(("port %s found: ", comp_ports[1].c_str()));
                  port1_var = ports1[p];
                }
            }
          if (CORBA::is_nil(port0_var))
            {
              RTC_ERROR(("port0 %s is nil obj", comp_ports[0].c_str()));
              continue;
            }
          if (CORBA::is_nil(port1_var))
            {
              RTC_ERROR(("port1 %s is nil obj", comp_ports[1].c_str()));
              continue;
            }
          ConnectorProfile conn_prof;
          std::string prof_name;
          conn_prof.name = CORBA::string_dup(connectors[i].c_str());
          conn_prof.connector_id = CORBA::string_dup("");
          conn_prof.ports.length(2);
          conn_prof.ports[0] = port0_var;
          conn_prof.ports[1] = port1_var;
          coil::Properties prop;
          prop["dataport.dataflow_type"] = "push";
          prop["dataport.interface_type"] = "corba_cdr";
          coil::vstring opt_props = coil::split(conn_prop[1], "&");
          for (size_t o(0); o < opt_props.size(); ++o)
            {
              coil::vstring temp = coil::split(opt_props[o], "=");
              prop["dataport." + temp[0]] = temp[1];
            }
          NVUtil::copyFromProperties(conn_prof.properties, prop);
          if (RTC::RTC_OK != port0_var->connect(conn_prof))
            {
              RTC_ERROR(("Connection error: %s",
                         connectors[i].c_str()));
            }
        }
    } // end of pre-connection

    { // pre-activation
      RTC_TRACE(("Components pre-activation: %s",
                 m_config["manager.components.preactivation"].c_str()));
      std::vector<std::string> comps;
      comps = coil::split(m_config["manager.components.preactivation"],
                               ",");
      for (int i(0), len(comps.size()); i < len; ++i)
        {
          RTObject_impl* comp = getComponent(comps[i].c_str());
          if (comp == NULL)
            { RTC_ERROR(("%s not found.", comps[i].c_str())); continue; }

          ExecutionContextList_var ecs = comp->get_owned_contexts();
          ecs[0]->activate_component(comp->getObjRef());
        }
    } // end of pre-activation
    return true;
  }
  
  /*!
   * @if jp
   * @brief Manager§Œº¬π‘
   * @else
   * @brief Run the Manager
   * @endif
   */ 
  void Manager::runManager(bool no_block)
  {
    if (no_block)
      {
	RTC_TRACE(("Manager::runManager(): non-blocking mode"));
	m_runner = new OrbRunner(m_pORB);
	m_runner->open(0);
      }
    else
      {
	RTC_TRACE(("Manager::runManager(): blocking mode"));
	m_pORB->run();
	RTC_TRACE(("Manager::runManager(): ORB was terminated"));
	join();
      }
    return;
  }
  
  //============================================================
  // Module management
  //============================================================
  /*!
   * @if jp
   * @brief •‚•∏•Â°º•ÅEŒ•˙Ωº•…
   * @else
   * @brief Load module
   * @endif
   */
  void Manager::load(const char* fname, const char* initfunc)
  {
    RTC_TRACE(("Manager::load(fname = %s, initfunc = %s)",
               fname, initfunc));
    std::string file_name(fname);
    std::string init_func(initfunc);
    try
      {
        if (init_func.empty())
          {
            coil::vstring mod(coil::split(fname, "."));
            init_func = mod[0] + "Init";
          }
        std::string path(m_module->load(file_name, init_func));
        RTC_DEBUG(("module path: %s", path.c_str()));
      }
    catch (...)
      {
        RTC_ERROR(("module load error."));
      }
    return;
  }
  
  /*!
   * @if jp
   * @brief •‚•∏•Â°º•ÅEŒ•¢•Û•˙Ωº•…
   * @else
   * @brief Unload module
   * @endif
   */
  void Manager::unload(const char* fname)
  {
    RTC_TRACE(("Manager::unload()"));
    m_module->unload(fname);
    return;
  }
  
  /*!
   * @if jp
   * @brief ¡¥•‚•∏•Â°º•ÅEŒ•¢•Û•˙Ωº•…
   * @else
   * @brief Unload all modules
   * @endif
   */ 
  void Manager::unloadAll()
  {
    RTC_TRACE(("Manager::unloadAll()"));
    m_module->unloadAll();
    return;
  }
  
  /*!
   * @if jp
   * @brief •˙Ωº•…∫—§ﬂ§Œ•‚•∏•Â°º•ÅEÅEπ•»§ÚºË∆¿§π§ÅE   * @else
   * @brief Get a list of loaded modules
   * @endif
   */
  std::vector<coil::Properties> Manager::getLoadedModules()
  {
    RTC_TRACE(("Manager::getLoadedModules()"));
    return m_module->getLoadedModules();
  }
  
  /*!
   * @if jp
   * @brief •˙Ωº•…≤ƒ«Ω§ •‚•∏•Â°º•ÅEÅEπ•»§ÚºË∆¿§π§ÅE   * @else
   * @brief Get a list of loadable modules
   * @endif
   */
std::vector<coil::Properties> Manager::getLoadableModules()
  {    
    RTC_TRACE(("Manager::getLoadableModules()"));
    return m_module->getLoadableModules();
  }
  
  //============================================================
  // Component factory management
  //============================================================
  /*!
   * @if jp
   * @brief RT•≥•Û•›°º•Õ•Û•»Õ—•’•°•Ø•»•Í§Ú≈–œø§π§ÅE   * @else
   * @brief Register RT-Component Factory
   * @endif
   */
  bool Manager::registerFactory(coil::Properties& profile,
				RtcNewFunc new_func,
				RtcDeleteFunc delete_func)
  {
    RTC_TRACE(("Manager::registerFactory(%s)", profile["type_name"].c_str()));
    FactoryBase* factory;
    factory = new FactoryCXX(profile, new_func, delete_func);
    try
      {    
	bool ret = m_factory.registerObject(factory);
	if (!ret) {
	  delete factory;
	  return false;
	}
	return true;
      }
    catch (...)
      {
	delete factory;
	return false;
      }
  }
  
  std::vector<coil::Properties> Manager::getFactoryProfiles()
  {
    std::vector<FactoryBase*> factories(m_factory.getObjects());
    std::vector<coil::Properties> props;
    for (int i(0), len(factories.size()); i < len; ++i)
      {
        props.push_back(factories[i]->profile());
      }
    return props;
  }
  
  /*!
   * @if jp
   * @brief ExecutionContextÕ—•’•°•Ø•»•Í§Ú≈–œø§π§ÅE   * @else
   * @brief Register ExecutionContext Factory
   * @endif
   */
  bool Manager::registerECFactory(const char* name,
				  ECNewFunc new_func,
				  ECDeleteFunc delete_func)
  {
    RTC_TRACE(("Manager::registerECFactory(%s)", name));
    try
      {    
	ECFactoryBase* factory;
	factory = new ECFactoryCXX(name, new_func, delete_func);
	if(m_ecfactory.registerObject(factory))
          {
            return true;
          }
      }
    catch (...)
      {
	return false;
      }
    return false;
  }
  
  /*!
   * @if jp
   * @brief •’•°•Ø•»•ÅE¥•ÅEπ•»§ÚºË∆¿§π§ÅE   * @else
   * @brief Get the list of all Factories
   * @endif
   */
  std::vector<std::string> Manager::getModulesFactories()
  {
    RTC_TRACE(("Manager::getModulesFactories()"));
    
    ModuleFactories m;
    return m_factory.for_each(m).modlist;
  }
  
  //============================================================
  // Component management
  //============================================================
  /*!
   * @if jp
   * @brief RT•≥•Û•›°º•Õ•Û•»§Ú¿∏¿Æ§π§ÅE   * @else
   * @brief Create RT-Components
   * @endif
   */
  RTObject_impl* Manager::createComponent(const char* comp_args)
  {
    RTC_TRACE(("Manager::createComponent(%s)", comp_args));
    //------------------------------------------------------------
    // extract "comp_type" and "comp_prop" from comp_arg
    coil::Properties comp_prop, comp_id;
    if (!procComponentArgs(comp_args, comp_id, comp_prop)) return NULL;

    //------------------------------------------------------------
    // Because the format of port-name had been changed from <port_name> 
    // to <instance_name>.<port_name>, the following processing was added. 
    // (since r1648)

    if (comp_prop.findNode("exported_ports") != 0)
      {
        coil::vstring exported_ports;
        exported_ports = coil::split(comp_prop["exported_ports"], ",");

				std::string exported_ports_str("");
        for (size_t i(0), len(exported_ports.size()); i < len; ++i)
          {
            coil::vstring keyval(coil::split(exported_ports[i], "."));
            if (keyval.size() > 2)
              {
                exported_ports_str += (keyval[0] + "." + keyval.back());
              }
            else
              {
                exported_ports_str += exported_ports[i];
              }
	    
            if (i != exported_ports.size() - 1)
              {
                exported_ports_str += ",";
              }
          }
				
        comp_prop["exported_ports"] = exported_ports_str;
        comp_prop["conf.default.exported_ports"] = exported_ports_str;
 
      }
    //------------------------------------------------------------

    //------------------------------------------------------------
    // Create Component
    FactoryBase* factory(m_factory.find(comp_id));
    if (factory == 0)
      {
        RTC_ERROR(("Factory not found: %s",
                   comp_id["implementation_id"].c_str()));
        
        // automatic module loading
        std::vector<coil::Properties> mp(m_module->getLoadableModules());
        RTC_INFO(("%d loadable modules found", mp.size()));
        
        std::vector<coil::Properties>::iterator it;
        it = std::find_if(mp.begin(), mp.end(), ModulePredicate(comp_id));
        if (it == mp.end())
          {
            RTC_ERROR(("No module for %s in loadable modules list",
                       comp_id["implementation_id"].c_str()));
            return 0;
          }
        if (it->findNode("module_file_name") == 0)
          {
            RTC_ERROR(("Hmm...module_file_name key not found."));
            return 0;
          }
        // module loading
        RTC_INFO(("Loading module: %s", (*it)["module_file_name"].c_str()))
          load((*it)["module_file_name"].c_str(), "");
        factory = m_factory.find(comp_id);
        if (factory == 0) 
          {
            RTC_ERROR(("Factory not found for loaded module: %s",
                       comp_id["implementation_id"].c_str()));
            return 0;
          }
      }

    coil::Properties prop;
    prop = factory->profile();

    const char* inherit_prop[] = {
      "config.version",
      "openrtm.name",
      "openrtm.version",
      "os.name",
      "os.release",
      "os.version",
      "os.arch",
      "os.hostname",
      "corba.endpoint",
      "corba.id",
      "exec_cxt.periodic.type",
      "exec_cxt.periodic.rate",
      "exec_cxt.evdriven.type",
      "logger.enable",
      "logger.log_level",
      "naming.enable",
      "naming.type",
      "naming.formats",
      ""
    };

    for (int i(0); inherit_prop[i][0] != '\0'; ++i)
      {
        const char* key(inherit_prop[i]);
        if (m_config.findNode(key) != NULL)
          {
            prop[key] = m_config[key];
          }
      }
      
    RTObject_impl* comp;
    comp = factory->create(this);
    if (comp == NULL)
      {
	RTC_ERROR(("RTC creation failed: %s",
                   comp_id["implementation_id"].c_str()));
	return NULL;
      }
    RTC_TRACE(("RTC created: %s", comp_id["implementation_id"].c_str()));

    prop << comp_prop;

    //------------------------------------------------------------
    // Load configuration file specified in "rtc.conf"
    //
    // rtc.conf:
    //   [category].[type_name].config_file = file_name
    //   [category].[instance_name].config_file = file_name
    configureComponent(comp, prop);

    // comp->setProperties(prop);
    
    //------------------------------------------------------------
    // Component initialization
    if (comp->initialize() != RTC::RTC_OK)
      {
        RTC_TRACE(("RTC initialization failed: %s",
                   comp_id["implementation_id"].c_str()));
        RTC_TRACE(("%s was finalized", comp_id["implementation_id"].c_str()));
        if (comp->exit() != RTC::RTC_OK)
          {
            RTC_DEBUG(("%s finalization was failed.",
                       comp_id["implementation_id"].c_str()));
          }
        return NULL;
      }
    RTC_TRACE(("RTC initialization succeeded: %s",
               comp_id["implementation_id"].c_str()));
    //------------------------------------------------------------
    // Bind component to naming service
    registerComponent(comp);
    return comp;
  }
  
  /*!
   * @if jp
   * @brief RT•≥•Û•›°º•Õ•Û•»§Úƒæ¿‹ Manager §À≈–œø§π§ÅE   * @else
   * @brief Register RT-Component directly without Factory
   * @endif
   */
  bool Manager::registerComponent(RTObject_impl* comp)
  {
    RTC_TRACE(("Manager::registerComponent(%s)", comp->getInstanceName()));
    // ### NamingManager §Œ§ﬂ§«¬ÂÕ—≤ƒ«Ω
    m_compManager.registerObject(comp);
    
    std::vector<std::string> names(comp->getNamingNames());
    
    for (int i(0), len(names.size()); i < len; ++i)
      {
	RTC_TRACE(("Bind name: %s", names[i].c_str()));
	m_namingManager->bindObject(names[i].c_str(), comp);
      }
    return true;
  }
  
  /*!
   * @if jp
   * @brief RT•≥•Û•›°º•Õ•Û•»§Œ≈–œø§Ú≤ÚΩÅEπ§ÅE   * @else
   * @brief Unregister RT-Components
   * @endif
   */
  bool Manager::unregisterComponent(RTObject_impl* comp)
  {
    RTC_TRACE(("Manager::unregisterComponent(%s)", comp->getInstanceName()));
    // ### NamingManager §Œ§ﬂ§«¬ÂÕ—≤ƒ«Ω
    m_compManager.unregisterObject(comp->getInstanceName());
    
    coil::vstring names(comp->getNamingNames());
    
    for (int i(0), len(names.size()); i < len; ++i)
      {
	RTC_TRACE(("Unbind name: %s", names[i].c_str()));
	m_namingManager->unbindObject(names[i].c_str());
      }

    return true;
  }
  

  ExecutionContextBase* Manager::createContext(const char* ec_args)
  {
    RTC_TRACE(("Manager::createContext()"));
    RTC_TRACE(("ExecutionContext type: %s", 
	       m_config.getProperty("exec_cxt.periodic.type").c_str()));

    std::string ec_id;
    coil::Properties ec_prop;
    if (!procContextArgs(ec_args, ec_id, ec_prop)) return NULL;

    ECFactoryBase* factory(m_ecfactory.find(ec_id.c_str()));
    if (factory == NULL)
      {
        RTC_ERROR(("Factory not found: %s", ec_id.c_str()));
        return NULL;
      }

    ExecutionContextBase* ec;
    ec = factory->create();
    return ec;
  }
  
  /*!
   * @if jp
   * @brief Manager §À≈–œø§µ§ÅE∆§§§ÅET•≥•Û•›°º•Õ•Û•»§Ú∫ÅEÅEπ§ÅE   * @else
   * @brief Unregister RT-Components that have been registered to Manager
   * @endif
   */
  void Manager::deleteComponent(RTObject_impl* comp)
  {
    RTC_TRACE(("deleteComponent(RTObject*)"));
    // cleanup from manager's table, and naming serivce
    unregisterComponent(comp);

    // find factory
    coil::Properties& comp_id(comp->getProperties());
    FactoryBase* factory(m_factory.find(comp_id));
    if (factory == NULL)
      {
        RTC_DEBUG(("Factory not found: %s",
                   comp_id["implementation_id"].c_str()));
	return;
      }
    else
      {
        RTC_DEBUG(("Factory found: %s",
                   comp_id["implementation_id"].c_str()));
	factory->destroy(comp);
      } 
    
    if (coil::toBool(m_config["manager.shutdown_on_nortcs"],
                     "YES", "NO", true) &&
        !coil::toBool(m_config["manager.is_master"], "YES", "NO", false))
      {
        std::vector<RTObject_impl*> comps;
        comps = getComponents();
        if (comps.size() == 0)
          {
            shutdown();
          }
      }
  } 

  void Manager::deleteComponent(const char* instance_name)
  {
    RTC_TRACE(("deleteComponent(%s)", instance_name));
    RTObject_impl* comp;
    comp = m_compManager.find(instance_name);
    if (comp == 0)
      {
        RTC_WARN(("RTC %s was not found in manager.", instance_name));
        return;
      }
    deleteComponent(comp);
  }
  
  /*!
   * @if jp
   * @brief Manager §À≈–œø§µ§ÅE∆§§§ÅET•≥•Û•›°º•Õ•Û•»§Ú∏°∫˜§π§ÅE   * @else
   * @brief Get RT-Component's pointer
   * @endif
   */
  RTObject_impl* Manager::getComponent(const char* instance_name)
  {
    RTC_TRACE(("Manager::getComponent(%s)", instance_name));
    return m_compManager.find(instance_name);
  }
  
  /*!
   * @if jp
   * @brief Manager §À≈–œø§µ§ÅE∆§§§ÅE¥RT•≥•Û•›°º•Õ•Û•»§ÚºË∆¿§π§ÅE   * @else
   * @brief Get all RT-Components registered in the Manager
   * @endif
   */
  std::vector<RTObject_impl*> Manager::getComponents()
  {
    RTC_TRACE(("Manager::getComponents()"));
    return m_compManager.getObjects();
  }
  
  //============================================================
  // CORBA ¥ÿœ¢
  //============================================================
  /*!
   * @if jp
   * @brief ORB §Œ•›•§•Û•ø§ÚºË∆¿§π§ÅE   * @else
   * @brief Get the pointer to the ORB
   * @endif
   */
  CORBA::ORB_ptr Manager::getORB()
  {
    RTC_TRACE(("Manager::getORB()"));
    return m_pORB;
  }
  
  /*!
   * @if jp
   * @brief Manager §¨ª˝§ƒ RootPOA §Œ•›•§•Û•ø§ÚºË∆¿§π§ÅE   * @else
   * @brief Get the pointer to the RootPOA 
   * @endif
   */
  PortableServer::POA_ptr Manager::getPOA()
  {
    RTC_TRACE(("Manager::getPOA()"));
    return m_pPOA;
  }
  
  /*!
   * @if jp
   * @brief Manager §¨ª˝§ƒ POAManager §ÚºË∆¿§π§ÅE   * @else
   * @brief Get the pointer to the POAManager 
   * @endif
   */
  PortableServer::POAManager_ptr Manager::getPOAManager()
  {
    RTC_TRACE(("Manager::getPOAManager()"));
    return m_pPOAManager;
  }
  
  //============================================================
  // Protected functions
  //============================================================
  
  //============================================================
  // Manager initialization and finalization
  //============================================================
  /*!
   * @if jp
   * @brief Manager §Œ∆‚…ÙΩÈ¥ÅEΩΩËÕ˝
   * @else
   * @brief Manager internal initialization
   * @endif
   */
  void Manager::initManager(int argc, char** argv)
  {
    // load configurations
    ManagerConfig config(argc, argv);
    config.configure(m_config);
    m_config["logger.file_name"] = 
      formatString(m_config["logger.file_name"].c_str(), m_config);
    
    // initialize ModuleManager
    m_module = new ModuleManager(m_config);

    // initialize Terminator
    m_terminator = new Terminator(this);
    {
      Guard guard(m_terminate.mutex);
      m_terminate.waiting = 0;
    }
    
    // initialize Timer
    if (coil::toBool(m_config["timer.enable"], "YES", "NO", true))
      {
        coil::TimeValue tm(0, 100000);
				std::string tick(m_config["timer.tick"]);
				if (!tick.empty())
					{
						tm = atof(tick.c_str());
						m_timer = new coil::Timer(tm);
						m_timer->start();
					}
      }

    if (coil::toBool(m_config["manager.shutdown_auto"], "YES", "NO", true) &&
        !coil::toBool(m_config["manager.is_master"], "YES", "NO", false))
      {
        coil::TimeValue tm(10, 0);
        if (m_config.findNode("manager.auto_shutdown_duration") != NULL)
          {
            double duration;
            const char* s = m_config["manager.auto_shutdown_duration"].c_str();
            if (coil::stringTo(duration, s))
              {
                tm = duration;
              }
          }
        if (m_timer != NULL)
          {
            m_timer->registerListenerObj(this, 
                                         &Manager::shutdownOnNoRtcs, tm);
          }
      }
    
    {
      coil::TimeValue tm(1, 0); 
      if (m_timer != NULL)
        {
          m_timer->registerListenerObj(this, 
                                       &Manager::cleanupComponents, tm);
        }
    }

  }
  
  /*!
   * @if jp
   * @brief Manager §ŒΩ™ŒªΩËÕ˝
   * @else
   * @brief Manager internal finalization
   * @endif
   */
  void Manager::shutdownManager()
  {
    RTC_TRACE(("Manager::shutdownManager()"));
    m_timer->stop();
  }

  void  Manager::shutdownOnNoRtcs()
  {
    RTC_TRACE(("Manager::shutdownOnNoRtcs()"));
    if (coil::toBool(m_config["manager.shutdown_on_nortcs"], "YES", "NO", true))
      {
        std::vector<RTObject_impl*> comps(getComponents());
        if (comps.size() == 0)
          {
            shutdown();
          }
      }

  }
  
  //============================================================
  // Logger initialization and finalization
  //============================================================
  /*!
   * @if jp
   * @brief System logger §ŒΩÈ¥ÅEΩ
   * @else
   * @brief System logger initialization
   * @endif
   */
  bool Manager::initLogger()
  {
    rtclog.setLevel("SILENT");
    rtclog.setName("manager");
    
    if (!coil::toBool(m_config["logger.enable"], "YES", "NO", true))
      {
        return true;
      }

    std::vector<std::string> logouts;
    logouts = coil::split(m_config["logger.file_name"], ",");

    for (int i(0), len(logouts.size()); i < len; ++i)
      {
        std::string logfile(logouts[i]);
        if (logfile == "") continue;
	
        // Open logfile
        if (logfile == "STDOUT" || logfile == "stdout")
          {
            m_logStreamBuf.addStream(std::cout.rdbuf());
            continue;
          }
        
        std::filebuf* of = new std::filebuf();
        of->open(logfile.c_str(), std::ios::out | std::ios::app);

        if (!of->is_open())
          {
            std::cerr << "Error: cannot open logfile: "
                      << logfile << std::endl;
            delete of;
            continue;
          }
        m_logStreamBuf.addStream(of, true);
        m_logfiles.push_back(of);
      }
	

    // Set date format for log entry header
    rtclog.setDateFormat(m_config["logger.date_format"].c_str());
    
    // Loglevel was set from configuration file.
    rtclog.setLevel(m_config["logger.log_level"].c_str());
	
    // Log stream mutex locking mode
    coil::toBool(m_config["logger.stream_lock"],
                 "enable", "disable", false) ? 
      rtclog.enableLock() : rtclog.disableLock();
                 
	
    RTC_INFO(("%s", m_config["openrtm.version"].c_str()));
    RTC_INFO(("Copyright (C) 2003-2014"));
    RTC_INFO(("  Noriaki Ando"));
    RTC_INFO(("  Intelligent Systems Research Institute, AIST"));
    RTC_INFO(("Manager starting."));
    RTC_INFO(("Starting local logging."));

    return true;;
  }
  
  /*!
   * @if jp
   * @brief System Logger §ŒΩ™ŒªΩËÕ˝
   * @else
   * @brief System Logger finalization
   * @endif
   */
  void Manager::shutdownLogger()
  {
    RTC_TRACE(("Manager::shutdownLogger()"));
    rtclog.flush();

    for (int i(0), len(m_logfiles.size()); i < len; ++i)
      {
        m_logfiles[i]->close();
        //        m_logStreamBuf.removeStream(m_logfiles[i]->rdbuf());
        delete m_logfiles[i];
      }
    if (!m_logfiles.empty())
      {
        m_logfiles.clear();
      }
  }
  
  //============================================================
  // ORB initialization and finalization
  //============================================================
  /*!
   * @if jp
   * @brief CORBA ORB §ŒΩÈ¥ÅEΩΩËÕ˝
   * @else
   * @brief CORBA ORB initialization
   * @endif
   */
  bool Manager::initORB()
  {
    RTC_TRACE(("Manager::initORB()"));
    // Initialize ORB
    try
      {
	std::vector<std::string> args(coil::split(createORBOptions(), " "));
	// TAO's ORB_init needs argv[0] as command name.
	args.insert(args.begin(), "manager");
	char** argv = coil::toArgv(args);
	int argc(args.size());
	
	// ORB initialization
	m_pORB = CORBA::ORB_init(argc, argv);
	// Get the RootPOA
	CORBA::Object_var obj =
          m_pORB->resolve_initial_references((char*)"RootPOA");
	m_pPOA = PortableServer::POA::_narrow(obj);
	if (CORBA::is_nil(m_pPOA))
	  {
	    RTC_ERROR(("Could not resolve RootPOA."));
	    return false;
	  }
	// Get the POAManager
	m_pPOAManager = m_pPOA->the_POAManager();

#ifdef ORB_IS_OMNIORB
        const char* conf = "corba.alternate_iiop_addresses";
        if (m_config.findNode(conf) != NULL)
          {
            coil::vstring addr_list;
            addr_list = coil::split(m_config[conf], ",", true);

            for (size_t i(0); i < addr_list.size(); ++i)
              {
                coil::vstring addr_port = coil::split(addr_list[i], ":");
                if (addr_port.size() == 2)
                  {
                    IIOP::Address iiop_addr;
                    iiop_addr.host = addr_port[0].c_str();
                    CORBA::UShort port; 
                    coil::stringTo(port, addr_port[1].c_str());
                    iiop_addr.port = port;
#if defined(RTM_OMNIORB_40) || defined(RTM_OMNIORB_41)
                    omniIOR::add_IIOP_ADDRESS(iiop_addr);
#else
                    omniIOR::add_IIOP_ADDRESS(iiop_addr, 0);
#endif // defined(RTC_OMNIORB_40) and defined(RTC_OMNIORB_41)
                  }
              }
          }
#endif // ORB_IS_OMNIORB
      }
    catch (...)
      {
	RTC_ERROR(("Exception: Caught unknown exception in initORB()." ));
	return false;
      }
    return true;
  }
  
  /*!
   * @if jp
   * @brief ORB §Œ•≥•ﬁ•Û•…•È•§•Û•™•◊•∑•Á•Û∫˚‹Æ
   * @else
   * @brief ORB command option creation
   * @endif
   */
  std::string Manager::createORBOptions()
  {
    std::string opt(m_config["corba.args"]);
    RTC_DEBUG(("corba.args: %s", opt.c_str()));

    RTC_DEBUG_STR((m_config));

    coil::vstring endpoints;
    createORBEndpoints(endpoints);
    createORBEndpointOption(opt, endpoints);

    RTC_PARANOID(("ORB options: %s", opt.c_str()));
    return opt;
  }

  void Manager::createORBEndpoints(coil::vstring& endpoints)
  {
    // corba.endpoint is obsolete
    // corba.endpoints with comma separated values are acceptable
    if (m_config.findNode("corba.endpoints") != 0)
      {
        endpoints = coil::split(m_config["corba.endpoints"], ",");
        RTC_DEBUG(("corba.endpoints: %s", m_config["corba.endpoints"].c_str()));
      }

    if (m_config.findNode("corba.endpoint") != 0)
      {
        coil::vstring tmp(coil::split(m_config["corba.endpoint"], ","));
        endpoints.insert(endpoints.end(), tmp.begin(), tmp.end());
        RTC_DEBUG(("corba.endpoint: %s", m_config["corba.endpoint"].c_str()));
      }
    // If this process has master manager,
    // master manager's endpoint inserted at the top of endpoints
    RTC_DEBUG(("manager.is_master: %s",
               m_config["manager.is_master"].c_str()));
    if (coil::toBool(m_config["manager.is_master"], "YES", "NO", false))
      {
        std::string mm(m_config.getProperty("corba.master_manager", ":2810"));
        coil::vstring mmm(coil::split(mm, ":"));
        if (mmm.size() == 2)
          {
            endpoints.insert(endpoints.begin(), std::string(":") + mmm[1]);
          }
        else
          {
            endpoints.insert(endpoints.begin(), ":2810");
          }
      }
    coil::vstring tmp(endpoints);
    endpoints = coil::unique_sv(tmp);
  }

  void Manager::createORBEndpointOption(std::string& opt,
                                        coil::vstring& endpoints)
  {
    std::string corba(m_config["corba.id"]);
    RTC_DEBUG(("corba.id: %s", corba.c_str()));

    for (size_t i(0); i < endpoints.size(); ++i)
      {
        std::string& endpoint(endpoints[i]);
        RTC_DEBUG(("Endpoint is : %s", endpoint.c_str()));
        if (endpoint.find(":") == std::string::npos) { endpoint += ":"; }

	if (corba == "omniORB")
          {
            coil::normalize(endpoint);
            if (coil::normalize(endpoint) == "all:")
              {
#ifdef ORB_IS_OMNIORB
#ifdef RTC_CORBA_CXXMAPPING11
                // omniORB 4.1 or later
                opt += " -ORBendPointPublish all(addr)";
#else
                // omniORB 4.0
                opt += " -ORBendPointPublishAllIFs 1";
#endif // RTC_CORBA_CXXMAPPING1
#endif // ORB_IS_OMNIORB
              }
            else
              {
                opt += " -ORBendPoint giop:tcp:" + endpoint;
              }
          }
	else if (corba == "TAO")
          {
            opt += "-ORBEndPoint iiop://" + endpoint;
          }
	else if (corba == "MICO")
          {
            opt += "-ORBIIOPAddr inet:" + endpoint;
          }
      }
  }

  
  /*!
   * @if jp
   * @brief ORB §ŒΩ™ŒªΩËÕ˝
   * @else
   * @brief ORB finalization
   * @endif
   */
  void Manager::shutdownORB()
  {
    RTC_TRACE(("Manager::shutdownORB()"));
    if(CORBA::is_nil(m_pORB))
      {
        return;
      }
    try
      {
      while (m_pORB->work_pending())
        {
	  RTC_PARANOID(("Pending work still exists."));
	  if (m_pORB->work_pending())
	    m_pORB->perform_work();
        }
        RTC_DEBUG(("No pending works of ORB. Shutting down POA and ORB."));
      }
    catch(...)
      { 
        RTC_ERROR(("Caught SystemException during perform_work."));
      }
    
    if (!CORBA::is_nil(m_pPOA))
      {
	try
	  {
	    if (!CORBA::is_nil(m_pPOAManager))
	      m_pPOAManager->deactivate(false, true);
	    RTC_DEBUG(("POA Manager was deactivated."));
	    m_pPOA->destroy(false, true);
	    m_pPOA = PortableServer::POA::_nil();
	    RTC_DEBUG(("POA was destroid."));
	  }
	catch (CORBA::SystemException& ex)
	  {
	    RTC_ERROR(("Exception cought during root POA destruction"));
#ifndef ORB_IS_RTORB
	    RTC_ERROR(("CORBA::SystemException(minor=%d)", ex.minor()));
#endif // ORB_IS_RTORB
	  }
	catch (...)
	  {
	    RTC_ERROR(("Caught unknown exception during POA destruction."));
	  }
      }
    
    if (!CORBA::is_nil(m_pORB))
      {
	try
	  {
	    m_pORB->shutdown(true);
	    RTC_DEBUG(("ORB was shutdown."));
            //m_pORB->destroy();
	    RTC_DEBUG(("ORB was destroied."));
	    m_pORB = CORBA::ORB::_nil();
	  }
	catch (CORBA::SystemException& ex)
	  {
	    RTC_ERROR(("Exception caught during ORB shutdown"));
#ifndef ORB_IS_RTORB
            RTC_ERROR(("CORBA::SystemException(minodor=%d)", ex.minor()));
#endif // ORB_IS_RTORB
	  }
	catch (...)
	  {
	    RTC_ERROR(("Caught unknown exception during ORB shutdown."));
	  }
      }
  }
  
  //============================================================
  // Naming initialization and finalization
  //============================================================
  /*!
   * @if jp
   * @brief NamingManager §ŒΩÈ¥ÅEΩ
   * @else
   * @brief NamingManager initialization
   * @endif
   */
  bool Manager::initNaming()
  {
    RTC_TRACE(("Manager::initNaming()"));
    
    m_namingManager = new NamingManager(this);
    
    // If NameService is disabled, return immediately
    if (!coil::toBool(m_config["naming.enable"], "YES", "NO", true))
      {
        return true;
      }
    
    // NameServer registration for each method and servers
    std::vector<std::string> meth(coil::split(m_config["naming.type"], ","));
    
    for (int i(0), len_i(meth.size()); i < len_i; ++i)
      {
	std::vector<std::string> names;
	names = coil::split(m_config[meth[i] + ".nameservers"], ",");
	
	
	for (int j(0), len_j(names.size()); j < len_j; ++j)
	  {
	    RTC_TRACE(("Register Naming Server: %s/%s",		\
		       meth[i].c_str(), names[j].c_str()));	
	    m_namingManager->registerNameServer(meth[i].c_str(),
						names[j].c_str());
	  }
      }
    
    // NamingManager Timer update initialization
    if (coil::toBool(m_config["naming.update.enable"], "YES", "NO", true))
      {
        coil::TimeValue tm(10, 0); // default interval = 10sec for safty
	std::string intr(m_config["naming.update.interval"]);
	if (!intr.empty())
	  {
	    tm = atof(intr.c_str());
	  }
	if (m_timer != NULL)
	  {
	    m_timer->registerListenerObj(m_namingManager, 
					 &NamingManager::update, tm);
	  }
      }
    return true;
  }
  
  /*!
   * @if jp
   * @brief NamingManager §ŒΩ™ŒªΩËÕ˝
   * @else
   * @brief NamingManager finalization
   * @endif
   */
  void Manager::shutdownNaming()
  {
    RTC_TRACE(("Manager::shutdownNaming()"));
    m_namingManager->unbindAll();
    delete m_namingManager;
  }
  
  //============================================================
  // Naming initialization and finalization
  //============================================================
  /*!
   * @if jp
   * @brief ExecutionContextManager §ŒΩÈ¥ÅEΩ
   * @else
   * @brief ExecutionContextManager initialization
   * @endif
   */
  bool Manager::initExecContext()
  {
    RTC_TRACE(("Manager::initExecContext()"));
    PeriodicExecutionContextInit(this);
    ExtTrigExecutionContextInit(this);
    OpenHRPExecutionContextInit(this);
    return true;
  }

  bool Manager::initComposite()
  {
    RTC_TRACE(("Manager::initComposite()"));
    PeriodicECSharedCompositeInit(this);

    return true;
  }
  
  bool Manager::initFactories()
  {
    RTC_TRACE(("Manager::initFactories()"));
    FactoryInit();
    return true;
  }

  /*!
   * @if jp
   * @brief Timer §ŒΩÈ¥ÅEΩ
   * @else
   * @brief Timer initialization
   * @endif
   */
  bool Manager::initTimer()
  {
    return true;
  }


  bool Manager::initManagerServant()
  {
    RTC_TRACE(("Manager::initManagerServant()"));
    if (!coil::toBool(m_config["manager.corba_servant"], "YES", "NO", true))
      {
        return true;
      }
    m_mgrservant = new ::RTM::ManagerServant();
    coil::Properties& prop(m_config.getNode("manager"));
    std::vector<std::string> names(coil::split(prop["naming_formats"], ","));

    if (coil::toBool(prop["is_master"], "YES", "NO", true))
      {
        for (int i(0), len(names.size()); i < len; ++i)
          {
            std::string mgr_name(formatString(names[i].c_str(), prop));
            m_namingManager->bindObject(mgr_name.c_str(), m_mgrservant);
          }
      }

    std::ifstream otherref(m_config["manager.refstring_path"].c_str());
    if (otherref.fail() != 0)
      {
        otherref.close();
        std::ofstream reffile(m_config["manager.refstring_path"].c_str());
	RTM::Manager_var mgr_v(RTM::Manager::
                               _duplicate(m_mgrservant->getObjRef()));
        CORBA::String_var str_var = m_pORB->object_to_string(mgr_v);
	reffile << str_var;
        reffile.close();
      }
    else
      {
        std::string refstring;
        otherref >> refstring;
        otherref.close();

        CORBA::Object_var obj = m_pORB->string_to_object(refstring.c_str());
        RTM::Manager_var mgr = RTM::Manager::_narrow(obj);
        //        if (CORBA::is_nil(mgr)) return false;
        //        mgr->set_child(m_mgrservant->getObjRef());
        //        m_mgrservant->set_owner(mgr);
      }

    return true;
  }
  
  /*!
   * @if jp
   * @brief NamingManager §ŒΩ™ŒªΩËÕ˝
   * @else
   * @brief NamingManager finalization
   * @endif
   */
  void Manager::shutdownComponents()
  {
    RTC_TRACE(("Manager::shutdownComponents()"));
    std::vector<RTObject_impl*> comps;
    comps = m_namingManager->getObjects();
    for (int i(0), len(comps.size()); i < len; ++i)
      {
	try
	  {
	    comps[i]->exit();
	    coil::Properties p(comps[i]->getInstanceName());
	    p << comps[i]->getProperties();
            rtclog.lock();
	    rtclog.level(Logger::RTL_PARANOID) << p;
            rtclog.unlock();
	  }
	catch (...)
	  {
	    ;
	  }
      }
    for (CORBA::ULong i(0), len(m_ecs.size()); i < len; ++i)
      {
	try{
          PortableServer::ObjectId_var oid = m_pPOA->servant_to_id(m_ecs[i]);
	  m_pPOA->deactivate_object(oid);
	}
	catch (...)
	  {
	    ;
	  }
      }
  }
  
  /*!
   * @if jp
   * @brief RT•≥•Û•›°º•Õ•Û•»§Œ≈–œø≤ÚΩÅE   * @else
   * @brief Unregister RT-Components
   * @endif
   */
  void Manager::cleanupComponent(RTObject_impl* comp)
  {
    RTC_TRACE(("Manager::cleanupComponent()"));
    unregisterComponent(comp);
  }

  void Manager::cleanupComponents()
  {
    RTC_VERBOSE(("Manager::cleanupComponents()"));
    Guard guard(m_finalized.mutex);
    RTC_VERBOSE(("%d components are marked as finalized.",
               m_finalized.comps.size()));
    for (size_t i(0); i < m_finalized.comps.size(); ++i)
      {
        deleteComponent(m_finalized.comps[i]);
      }
    m_finalized.comps.clear();
  }

  void Manager::notifyFinalized(RTObject_impl* comp)
  {
    RTC_TRACE(("Manager::notifyFinalized()"));
    Guard guard(m_finalized.mutex);
    m_finalized.comps.push_back(comp);
  }
  
  /*!
   * @if jp
   * @brief createComponent§Œ∞˙øÙ§ÚΩËÕ˝§π§ÅE   * @else
   *
   * @endif
   */
  bool Manager::procComponentArgs(const char* comp_arg,
                                  coil::Properties& comp_id,
                                  coil::Properties& comp_conf)
  {
    std::vector<std::string> id_and_conf(coil::split(comp_arg, "?"));
    // arg should be "id?key0=value0&key1=value1...".
    // id is mandatory, conf is optional
    if (id_and_conf.size() != 1 && id_and_conf.size() != 2)
      {
        RTC_ERROR(("Invalid arguments. Two or more '?' in arg : %s", comp_arg));
        return false;
      }
    if (id_and_conf[0].find(":") == std::string::npos)
      {
        id_and_conf[0].insert(0, "RTC:::");
        id_and_conf[0] += ":";
      }
    std::vector<std::string> id(coil::split(id_and_conf[0], ":"));

    // id should be devided into 1 or 5 elements
    // RTC:[vendor]:[category]:impl_id:[version] => 5
    if (id.size() != 5) 
      {
        RTC_ERROR(("Invalid RTC id format.: %s", id_and_conf[0].c_str()));
        return false;
      }

    const char* prof[] =
      {
        "RTC", "vendor", "category", "implementation_id", "version"
      };

    if (id[0] != prof[0])
      {
        RTC_ERROR(("Invalid id type: %s", id[0].c_str()));
        return false;
      }
    for (int i(1); i < 5; ++i)
      {
        comp_id[prof[i]] = id[i];
        RTC_TRACE(("RTC basic propfile %s: %s", prof[i], id[i].c_str()));
      }

    if (id_and_conf.size() == 2)
      {
        std::vector<std::string> conf(coil::split(id_and_conf[1], "&"));
        for (int i(0), len(conf.size()); i < len; ++i)
          {
            if (conf[i].empty()) { continue; }
            std::vector<std::string> keyval(coil::split(conf[i], "="));
            if (keyval.size() != 2) { continue; }
            comp_conf[keyval[0]] = keyval[1];
            RTC_TRACE(("RTC property %s: %s",
                       keyval[0].c_str(), keyval[1].c_str()));
          }
      }
    return true;
  }

  bool Manager::procContextArgs(const char* ec_args,
                                std::string& ec_id,
                                coil::Properties& ec_conf)
  {
    std::vector<std::string> id_and_conf(coil::split(ec_args, "?"));
    if (id_and_conf.size() != 1 && id_and_conf.size() != 2)
      {
        RTC_ERROR(("Invalid arguments. Two or more '?' in arg : %s", ec_args));
        return false;
      }
    if (id_and_conf[0].empty())
      {
        RTC_ERROR(("Empty ExecutionContext's name"));
        return false;
      }
    ec_id =id_and_conf[0];
    
    if (id_and_conf.size() == 2)
      {
        std::vector<std::string> conf(coil::split(id_and_conf[1], "&"));
        for (int i(0), len(conf.size()); i < len; ++i)
          {
            std::vector<std::string> k(coil::split(conf[i], "="));
            ec_conf[k[0]] = k[1];
            RTC_TRACE(("EC property %s: %s", k[0].c_str(), k[1].c_str()));
          }
      }
    return true;
  }
  
  /*!
   * @if jp
   * @brief RT•≥•Û•›°º•Õ•Û•»§Œ•≥•Û•’•£•Æ•Â•ÅEº•∑•Á•ÛΩËÕ˝
   * @else
   *
   * @endif
   */
  void Manager::configureComponent(RTObject_impl* comp,
                                   const coil::Properties& prop)
  {
    std::string category(comp->getCategory());
    std::string type_name(comp->getTypeName());
    std::string inst_name(comp->getInstanceName());
    
    std::string type_conf(category + "." + type_name + ".config_file");
    std::string name_conf(category + "." + inst_name + ".config_file");
    coil::vstring config_fname;
    
    coil::Properties type_prop, name_prop;
    
    // Load "category.instance_name.config_file"
    if (!m_config[name_conf].empty())
      {
        std::ifstream conff(m_config[name_conf].c_str());
        if (!conff.fail())
          {
            name_prop.load(conff);
            RTC_INFO(("Component instance conf file: %s loaded.",
                      m_config[name_conf].c_str()));
            RTC_DEBUG_STR((name_prop))
              config_fname.push_back(m_config[name_conf].c_str());
          }
      }
    if (m_config.findNode(category + "." + inst_name) != NULL)
      {
        coil::Properties& temp(m_config.getNode(category + "." + inst_name));
        coil::vstring keys(temp.propertyNames());
        if (!(keys.size() == 1 && keys.back() == "config_file"))
          {
            name_prop << m_config.getNode(category + "." + inst_name);
            RTC_INFO(("Component type conf exists in rtc.conf. Merged."));
            RTC_DEBUG_STR((name_prop));
            if (m_config.findNode("config_file") != NULL)
              {
                config_fname.push_back(m_config["config_file"]);
              }
          }
      }
    
    if (!m_config[type_conf].empty())
      {
        std::ifstream conff(m_config[type_conf].c_str());
        if (!conff.fail())
          {
            type_prop.load(conff);
            RTC_INFO(("Component type conf file: %s loaded.",
                      m_config[type_conf].c_str()));
            RTC_DEBUG_STR((type_prop));
            config_fname.push_back(m_config[type_conf].c_str());
          }
      }
    if (m_config.findNode(category + "." + type_name) != NULL)
      {
        coil::Properties& temp(m_config.getNode(category + "." + type_name));
        coil::vstring keys(temp.propertyNames());
        if (!(keys.size() == 1 && keys.back() == "config_file"))
          {
            type_prop << m_config.getNode(category + "." + type_name);
            RTC_INFO(("Component type conf exists in rtc.conf. Merged."));
            RTC_DEBUG_STR((type_prop));
            if (m_config.findNode("config_file") != NULL)
              {
                config_fname.push_back(m_config["config_file"]);
              }
          }
      }
    
    // Merge Properties. type_prop is merged properties
    comp->setProperties(prop);
    type_prop << name_prop;
    type_prop["config_file"] = coil::flatten(coil::unique_sv(config_fname));    
    comp->setProperties(type_prop);
    
    //------------------------------------------------------------
    // Format component's name for NameService
    std::string naming_formats;
    coil::Properties& comp_prop(comp->getProperties());
    
    naming_formats += m_config["naming.formats"];
    if (comp_prop.findNode("naming.formats") != 0)
      {
        naming_formats = comp_prop["naming.formats"];
      }
    naming_formats = coil::flatten(coil::unique_sv(coil::split(naming_formats,
                                                               ",")));
    
    std::string naming_names;
    naming_names = formatString(naming_formats.c_str(), comp->getProperties());
    comp->getProperties()["naming.formats"] = naming_formats;
    comp->getProperties()["naming.names"] = naming_names;
  }
  
  /*!
   * @if jp
   * @brief •◊•˙¡—•∆•£æ Û§Œ•ﬁ°º•∏
   * @else
   * @brief Merge property information
   * @endif
   */
  bool Manager::mergeProperty(coil::Properties& prop, const char* file_name)
  {
    if (file_name == NULL)
      {
	RTC_ERROR(("Invalid configuration file name."));
	return false;
      }
    if (file_name[0] != '\0')
      {
	std::ifstream conff(file_name);
	if (!conff.fail())
	  {
	    prop.load(conff);
	    conff.close();
	    return true;
	  }
      }
    return false;
  }
  
  /*!
   * @if jp
   * @brief NamingServer §À≈–œø§π§ÅE›§Œ≈–œøæ Û§Ú¡»§ﬂŒ©§∆§ÅE   * @else
   * @brief Construct registration information when registering to Naming server
   * @endif
   */
  std::string Manager::formatString(const char* naming_format,
                                    coil::Properties& prop)
  {
    std::string name(naming_format), str("");
    std::string::iterator it, it_end;
    int count(0);
    
    it = name.begin();
    it_end = name.end();
    for ( ; it != it_end; ++it)
      {
        char c(*it);
        if (c == '%')
          {
            ++count;
            if (!(count % 2)) str.push_back((*it));
          }
        else if (c == '$')
          {
            count = 0;
            ++it;
            if (*it == '{' || *it == '(')
              {
                ++it;
                std::string env;
                for ( ; it != it_end && (*it) != '}' && (*it) != ')'; ++it)
                  {
                    env += *it;
                  }
                char* envval = coil::getenv(env.c_str());
                if (envval != NULL) str += envval;
              }
            else
              {
               str.push_back(c);
              }
          }
        else
          {
            if (count > 0 && (count % 2))
              {
                count = 0;
                if      (c == 'n')  str += prop["instance_name"];
                else if (c == 't')  str += prop["type_name"];
                else if (c == 'm')  str += prop["type_name"];
                else if (c == 'v')  str += prop["version"];
                else if (c == 'V')  str += prop["vendor"];
                else if (c == 'c')  str += prop["category"];
                else if (c == 'i')  str += prop["implementation_id"];
                else if (c == 'N')
                  {
                    size_t n = prop["implementation_id"].size();
                    str += prop["instance_name"].substr(n);
                  }
                else if (c == 'h')  str += m_config["os.hostname"];
                else if (c == 'M')  str += m_config["manager.name"];
                else if (c == 'p')  str += m_config["manager.pid"];
                else str.push_back(c);
              }
            else
              {
                count = 0;
                str.push_back(c);
              }
          }
      }
    return str;
  }
  
};
