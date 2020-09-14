// -*- C++ -*-
/*!
 * @file   ManagerServantTests.cpp
 * @brief  ManagerServant test class
 * @date   $Date$ 
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$ 
 *
 */

/*
 * $Log$
 *
 */

#ifndef ManagerServant_cpp
#define ManagerServant_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <string>
#include <iostream>
#include <idl/SDOPackageSkel.h>
#include <idl/RTCSkel.h>
#include <rtm/idl/ManagerSkel.h>
#include <rtm/ManagerServant.h>
#include <rtm/NVUtil.h>

/*!
 * @class ManagerServantTests class
 * @brief ManagerServant test
 */
namespace ManagerServant
{
  /*!
   * 
   * 
   *
   */
  class Logger
  {
  public:
    void log(const std::string& msg)
    {
      m_log.push_back(msg);
    }

    int countLog(const std::string& msg)
    {
      int count = 0;
      for (int i = 0; i < (int) m_log.size(); ++i)
        {
          if (m_log[i] == msg) ++count;
        }
     return count;
    }
		
  private:
    std::vector<std::string> m_log;
  };

  class ManagerServantTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ManagerServantTests);

    CPPUNIT_TEST(test_add_master_manager);
    CPPUNIT_TEST(test_add_slave_manager);
    CPPUNIT_TEST(test_get_loadable_modules);
    CPPUNIT_TEST(test_load_module);
    CPPUNIT_TEST(test_unload_modules);
    CPPUNIT_TEST(test_get_loaded_modules);
    CPPUNIT_TEST(test_get_factory_profiles);
    CPPUNIT_TEST(test_create_component);
    CPPUNIT_TEST(test_get_components);
    CPPUNIT_TEST(test_get_component_profiles);
    CPPUNIT_TEST(test_get_profile);
    CPPUNIT_TEST(test_get_configuration);
    CPPUNIT_TEST(test_set_configuration);
    CPPUNIT_TEST(test_fork);
    CPPUNIT_TEST(test_get_service);
    CPPUNIT_TEST(test_getObjRef);
    CPPUNIT_TEST(test_delete_component);

//    CPPUNIT_TEST(test_shutdown);  //OK
//    CPPUNIT_TEST(test_restart);   //OK
    CPPUNIT_TEST_SUITE_END();
  
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
    RTM::Manager_ptr m_objref;

    /*!
     *
     */
    bool isFound(const ::RTM::ModuleProfileList* list, const std::string& mod)
    {
        const char *pch;
        for (CORBA::ULong ic = 0; ic < list->length(); ++ic)
        {
            if( (*list)[ic].properties[0].value >>= pch )
            {
                if(mod == ::std::string(pch))
                {
                    return true;
                }
            }
        }
        return false;
    }
  
  public:
  
    /*!
     * @brief Constructor
     */
    ManagerServantTests()
    {

      int argc = 0;
      char** argv = NULL;
      m_pORB = CORBA::ORB_init(argc, argv);
      m_pPOA = PortableServer::POA::_narrow(
                       m_pORB->resolve_initial_references("RootPOA"));
      m_pPOA->the_POAManager()->activate();

    }
    
    /*!
     * @brief Destructor
     */
    ~ManagerServantTests()
    {
    }
  
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
    }
    
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    { 
    }
  

    /*! 
     * @brief tests for is_master(), createINSManager(), findManager(), add_master_manager(), get_master_managers(), remove_master_manager()
     *
     *
     */
    void test_add_master_manager()
    {
      ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
      RTM::ManagerList* list;
      try
      {
        ::RTC::ReturnCode_t ret;
        CORBA::Boolean cbret;
        cbret = pman->is_master();
        // is_master(), default is false
        CPPUNIT_ASSERT(!cbret);

        // get_master_managers()
        list = pman->get_master_managers();
         CPPUNIT_ASSERT_EQUAL(0, (int)list->length());

        // createINSManager()
        bool bret = pman->createINSManager();
        CPPUNIT_ASSERT(bret);

        bret = pman->createINSManager();
        CPPUNIT_ASSERT(!bret);

        std::string host_port("localhost:2810");
        RTM::Manager_var owner;
        // findManager()
        owner = pman->findManager(host_port.c_str());

        // add_master_manager()
        ret = pman->add_master_manager(owner);
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ret);
        list = pman->get_master_managers();
         CPPUNIT_ASSERT_EQUAL(1, (int)list->length());

        // remove_master_manager()
        ret = pman->remove_master_manager(RTM::Manager::_duplicate(owner));
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ret);
        list = pman->get_master_managers();
         CPPUNIT_ASSERT_EQUAL(0, (int)list->length());

        CORBA::Object_var obj;
        obj = m_pORB->resolve_initial_references("omniINSPOA");
        PortableServer::POA_ptr poa = PortableServer::POA::_narrow(obj);
        poa->the_POAManager()->deactivate(false, true);
      }
      catch(CORBA::SystemException& e)
      {
        std::cout << "test_add_master_manager() SystemException: " << e._name() << std::endl;
      }
      catch (...)
      {
        std::cout << "test_add_master_manager() other Exception" << std::endl;
      }

      delete list;
      delete pman;
    }

    /*! 
     * @brief tests for add_slave_manager(), get_slave_managers(), remove_slave_manager()
     *
     *
     */
    void test_add_slave_manager()
    {
      ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
      RTM::ManagerList* list;
      try
      {
        ::RTC::ReturnCode_t ret;

        // get_slave_managers()
        list = pman->get_slave_managers();
         CPPUNIT_ASSERT_EQUAL(0, (int)list->length());

        // createINSManager()
        bool bret = pman->createINSManager();
        CPPUNIT_ASSERT(!bret);

        std::string host_port("localhost:2810");
        RTM::Manager_var owner;

        // findManager()
        owner = pman->findManager(host_port.c_str());

        // add_slave_manager()
        ret = pman->add_slave_manager(owner);
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ret);
        list = pman->get_slave_managers();
         CPPUNIT_ASSERT_EQUAL(1, (int)list->length());

        // remove_slave_manager()
        ret = pman->remove_slave_manager(RTM::Manager::_duplicate(owner));
        CPPUNIT_ASSERT_EQUAL(RTC::RTC_OK, ret);
        list = pman->get_slave_managers();
         CPPUNIT_ASSERT_EQUAL(0, (int)list->length());

        CORBA::Object_var obj;
        obj = m_pORB->resolve_initial_references("omniINSPOA");
        PortableServer::POA_ptr poa = PortableServer::POA::_narrow(obj);
        poa->the_POAManager()->deactivate(false, true);
      }
      catch(CORBA::SystemException& e)
      {
        std::cout << "test_add_slave_manager() SystemException: " << e._name() << std::endl;
      }
      catch (...)
      {
        std::cout << "test_add_slave_manager() other Exception" << std::endl;
      }

      delete list;
      delete pman;
    }

    /*! 
     * @brief tests for load_module()
     *
     *
     *
     */
    void test_load_module()
    {
        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
        ::RTC::ReturnCode_t ret;
        try
        {
            ret = pman->load_module(".libs/DummyModule1.so","DummyModule1Init");
            CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
            CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(), 
                                   ".//.libs/DummyModule1.so"));
        }
        catch(...)
        {
	    CPPUNIT_FAIL("Exception thrown.");
        }

        //illegal file name.
        try
        {
            ret = pman->load_module("bar.so","DummyModule1Init");
	    CPPUNIT_FAIL("Exception not thrown.");
        }
        catch(...)
        {
            CPPUNIT_ASSERT(!isFound(pman->get_loaded_modules(), ".//bar.so"));
        }
        //illegal function name.
        try
        {
            ret = pman->load_module("DummyModule1i.so","foo");
	    CPPUNIT_FAIL("Exception not thrown.");
        }
        catch(...)
        {
            CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(), 
                                   ".//.libs/DummyModule1.so"));
        }
        //loading overlaps
        ret = pman->load_module(".libs/DummyModule1.so","DummyModule1Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule1.so"));

        //lodding another module
        ret = pman->load_module(".libs/DummyModule2.so","DummyModule2Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule2.so"));

        delete pman;
    }
    /*! 
     * @brief tests for unload_modules()
     *
     *
     *
     */
    void test_unload_modules()
    {
        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
        ::RTC::ReturnCode_t ret;
        try
        {
            ret = pman->load_module(".libs/DummyModule1.so","DummyModule1Init");
            CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                                   ".//.libs/DummyModule1.so"));
            CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        }
        catch(...)
        {
	    CPPUNIT_FAIL("Exception thrown.");
        }
        try
        {
            ret = pman->load_module(".libs/DummyModule2.so","DummyModule2Init");
            CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
            CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                                   ".//.libs/DummyModule2.so"));
        }
        catch(...)
        {
	    CPPUNIT_FAIL("Exception thrown.");
        }
        //
        try
        {
            ret = pman->unload_module(".//.libs/DummyModule2.so");
            CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        }
        catch(...)
        {
            CPPUNIT_FAIL( "unload error" );
        }
        //
        try
        {
            pman->unload_module("non-loaded-module.so");
	    CPPUNIT_FAIL("Exception not thrown.");
        }
        catch(...)
        {
//            CPPUNIT_FAIL( "unload error" );  //OK
        }
        delete pman;
    }
    /*! 
     * @brief tests for get_loaded_modules()
     *
     *
     *
     */
    void test_get_loaded_modules()
    {
        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
        ::RTC::ReturnCode_t ret;
        try
        {
            ret = pman->load_module(".libs/DummyModule1.so","DummyModule1Init");
            CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
            CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(), 
                                   ".//.libs/DummyModule1.so"));
        }
        catch(...)
        {
	    CPPUNIT_FAIL("Exception thrown.");
        }
        try
        {
            ret = pman->load_module(".libs/DummyModule2.so","DummyModule2Init");
            CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
            CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(), 
                                   ".//.libs/DummyModule2.so"));
        }
        catch(...)
        {
	    CPPUNIT_FAIL("Exception thrown.");
        }

        //Execute the function
        ::RTM::ModuleProfileList* list;
        list = pman->get_loaded_modules();
        ::RTM::ModuleProfileList modlist(*list);
        delete list;

        //Check returns(ModuleProfileList).
        CPPUNIT_ASSERT_EQUAL((::CORBA::ULong)2, modlist.length());
        CPPUNIT_ASSERT_EQUAL(::std::string("file_path"), 
                             ::std::string(modlist[0].properties[0].name));
        const char* ch;
        if( modlist[0].properties[0].value >>= ch )
        {
            CPPUNIT_ASSERT_EQUAL(::std::string(".//.libs/DummyModule1.so"), 
                                 ::std::string(ch));
        }
        else
        {
            CPPUNIT_FAIL( "ModuleProfileList is illegal." );
        }

        CPPUNIT_ASSERT_EQUAL(::std::string("file_path"), 
                             ::std::string(modlist[1].properties[0].name));

        if( modlist[1].properties[0].value >>= ch )
        {
            CPPUNIT_ASSERT_EQUAL(::std::string(".//.libs/DummyModule2.so"), 
                                 ::std::string(ch));
        }
        else
        {
            CPPUNIT_FAIL( "ModuleProfileList is illegal." );
        }
        delete pman;
    }
    /*! 
     * @brief tests for get_factory_profiles()
     *
     *
     *
     */
    void test_get_factory_profiles()
    {
        typedef struct data_struct {
            ::std::string name;
            ::std::string value;
        } DATA_STRUCT;
        DATA_STRUCT composite_spec[] =
        {
            {"implementation_id", "PeriodicECSharedComposite"},
            {"type_name",         "PeriodicECSharedComposite"},
            {"description",       "PeriodicECSharedComposite"},
            {"version",           "1.0"},
            {"vendor",            "jp.go.aist"},
            {"category",          "composite.PeriodicECShared"},
            {"activity_type",     "DataFlowComponent"},
            {"max_instance",      "0"},
            {"language",          "C++"},
            {"lang_type",         "compile"},
            {"exported_ports",    ""},
            {"conf.default.members", ""},
            {"conf.default.exported_ports", ""},
            {"",""},
        };
        DATA_STRUCT consolein_spec[] =
        {
            {"implementation_id", "DummyModule1"},
            {"type_name",         "DummyModule1"},
            {"description",       "Console input component"},
            {"version",           "1.0"},
            {"vendor",            "Noriaki Ando, AIST"},
            {"category",          "example"},
            {"activity_type",     "DataFlowComponent"},
            {"max_instance",      "10"},
            {"language",          "C++"},
            {"lang_type",         "compile"},
            {"",""},
        };
        DATA_STRUCT consoleout_spec[] =
        {
            {"implementation_id", "DummyModule2"},
            {"type_name",         "DummyModule2"},
            {"description",       "Console output component"},
            {"version",           "1.0"},
            {"vendor",            "Noriaki Ando, AIST"},
            {"category",          "example"},
            {"activity_type",     "DataFlowComponent"},
            {"max_instance",      "10"},
            {"language",          "C++"},
            {"lang_type",         "compile"},
            {"",""},
        };
        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();

        //Load modules.
        ::RTC::ReturnCode_t ret;
        ret = pman->load_module(".libs/DummyModule1.so","DummyModule1Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule1.so"));
        ret = pman->load_module(".libs/DummyModule2.so","DummyModule2Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule2.so"));

        //Execute the function
        ::RTM::ModuleProfileList *list;
        list = pman->get_factory_profiles(); 
        ::RTM::ModuleProfileList profiles(*list);
        delete list;

        //Check returns(ModuleProfileList).
        ::CORBA::ULong len;
        CPPUNIT_ASSERT_EQUAL((::CORBA::ULong)3, profiles.length());
        len = profiles[0].properties.length(); 
        CPPUNIT_ASSERT_EQUAL((::CORBA::ULong)13, len);
        for (CORBA::ULong ic = 0; ic < len; ++ic)
        {
            CPPUNIT_ASSERT_EQUAL(composite_spec[ic].name,
                           ::std::string(profiles[0].properties[ic].name));
             
            const char* ch;
            if( profiles[0].properties[ic].value >>= ch )
            {
                CPPUNIT_ASSERT_EQUAL(composite_spec[ic].value, 
                                     ::std::string(ch));
            }
            else
            {
                CPPUNIT_FAIL( "ModuleProfileList is illegal." );
            }
        }
        len =profiles[1].properties.length(); 
        CPPUNIT_ASSERT_EQUAL((::CORBA::ULong)10, len);
        for (CORBA::ULong ic = 0; ic < len; ++ic)
        {
            CPPUNIT_ASSERT_EQUAL(consolein_spec[ic].name,
                           ::std::string(profiles[1].properties[ic].name));
             
            const char* ch;
            if( profiles[1].properties[ic].value >>= ch )
            {
                CPPUNIT_ASSERT_EQUAL(consolein_spec[ic].value, 
                                     ::std::string(ch));
            }
            else
            {
                CPPUNIT_FAIL( "ModuleProfileList is illegal." );
            }
        }
        len =profiles[2].properties.length(); 
        CPPUNIT_ASSERT_EQUAL((::CORBA::ULong)10, len);
        for (CORBA::ULong ic = 0; ic < len; ++ic)
        {
            CPPUNIT_ASSERT_EQUAL(consoleout_spec[ic].name,
                           ::std::string(profiles[2].properties[ic].name));
             
            const char* ch;
            if( profiles[2].properties[ic].value >>= ch )
            {
                CPPUNIT_ASSERT_EQUAL(consoleout_spec[ic].value, 
                                     ::std::string(ch));
            }
            else
            {
                CPPUNIT_FAIL( "ModuleProfileList is illegal." );
            }
        }
        delete pman;
    }
    /*! 
     *
     *
     *
     */
    void test_create_component()
    {
        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
        //Load modules.
        ::RTC::ReturnCode_t ret;
        ret = pman->load_module(".libs/DummyModule1.so","DummyModule1Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule1.so"));
        ret = pman->load_module(".libs/DummyModule2.so","DummyModule2Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule2.so"));

        //Execute the function
        ::RTC::RTObject_ptr inobj;
        inobj = pman->create_component("DummyModule1AA");
        CPPUNIT_ASSERT(::CORBA::is_nil(inobj));
        inobj = pman->create_component("DummyModule1");
        CPPUNIT_ASSERT(!::CORBA::is_nil(inobj));

        ::RTC::RTObject_ptr outobj;
        outobj = pman->create_component("DummyModule2");
        CPPUNIT_ASSERT(!::CORBA::is_nil(outobj));
        delete pman;
    }
    /*! 
     * @brief tests for delete_components()
     *
     *
     *
     */
    void test_delete_component()
    {
        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
        //Load modules.
        ::RTC::ReturnCode_t ret;
        ret = pman->load_module(".libs/DummyModule1.so","DummyModule1Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule1.so"));
        ret = pman->load_module(".libs/DummyModule2.so","DummyModule2Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule2.so"));

        ::RTC::RTObject_ptr inobj;
        inobj = pman->create_component("DummyModule1");
        CPPUNIT_ASSERT(!::CORBA::is_nil(inobj));

        ::RTC::RTObject_ptr outobj;
        outobj = pman->create_component("DummyModule2");
        CPPUNIT_ASSERT(!::CORBA::is_nil(outobj));

        ::RTC::ComponentProfileList *list;
        list = pman->get_component_profiles();
        CPPUNIT_ASSERT(list!=NULL);
        ::RTC::ComponentProfileList profiles(*list);
        delete list;

//        m_pPOA->deactivate_object(*m_pPOA->reference_to_id(inobj));
        m_pPOA->the_POAManager()->deactivate(false, true);
        ret = pman->delete_component(profiles[0].instance_name);
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);

        // deleteComponent()でexit()を実行しているため、これ以降のテストはできません。
        // 以降のdelete_component は、実施せず。
//        m_pPOA->deactivate_object(*m_pPOA->reference_to_id(outobj));
//        ret = pman->delete_component(profiles[1].instance_name);
//        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        delete pman;
    }
    /*! 
     * @brief tests for get_components()
     *
     *
     *
     */
    void test_get_components()
    {
        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
        ::RTC::ReturnCode_t ret;
        ret = pman->load_module(".libs/DummyModule1.so","DummyModule1Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule1.so"));
        ret = pman->load_module(".libs/DummyModule2.so","DummyModule2Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule2.so"));

        ::RTC::RTObject_ptr inobj;
        inobj = pman->create_component("DummyModule1");
        CPPUNIT_ASSERT(!::CORBA::is_nil(inobj));

        ::RTC::RTObject_ptr outobj;
        outobj = pman->create_component("DummyModule2");
        CPPUNIT_ASSERT(!::CORBA::is_nil(outobj));

        //Execute the functions
        ::RTC::RTCList *list;
        list = pman->get_components();
        CPPUNIT_ASSERT(list != NULL);
        ::RTC::RTCList rtclist(*list);
        delete list;

        ::CORBA::ULong len(rtclist.length());
        bool bflag;
        bflag = false;
        for (::CORBA::ULong ic = 0; ic < len; ++ic)
        {
            if( rtclist[ic] == inobj )
            {
                bflag = true;
            }
        }
        CPPUNIT_ASSERT_EQUAL( bflag,true );

        bflag = false;
        for (::CORBA::ULong ic = 0; ic < len; ++ic)
        {
            if( rtclist[ic] == outobj )
            {
                bflag = true;
            }
        }
        CPPUNIT_ASSERT_EQUAL( bflag,true );
        delete pman;
    }
    /*! 
     * @brief tests for get_component_profiles()
     *
     *
     */
    void test_get_component_profiles()
    {
        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
        ::RTC::ReturnCode_t ret;
        ret = pman->load_module(".libs/DummyModule1.so","DummyModule1Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule1.so"));
        ret = pman->load_module(".libs/DummyModule2.so","DummyModule2Init");
        CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        CPPUNIT_ASSERT(isFound(pman->get_loaded_modules(),
                               ".//.libs/DummyModule2.so"));

        //create components.
        ::RTC::RTObject_ptr inobj;
        inobj = pman->create_component("DummyModule1");
        CPPUNIT_ASSERT(!::CORBA::is_nil(inobj));

        ::RTC::RTObject_ptr outobj;
        outobj = pman->create_component("DummyModule2");
        CPPUNIT_ASSERT(!::CORBA::is_nil(outobj));

        //Execute the functions.
        ::RTC::ComponentProfileList *list;
        list = pman->get_component_profiles();
        CPPUNIT_ASSERT(list!=NULL);
        ::RTC::ComponentProfileList profiles(*list);
        delete list;
        
        //Execute the functions
        ::RTC::RTCList *plist;
        plist = pman->get_components();
        CPPUNIT_ASSERT(plist != NULL);
        ::RTC::RTCList rtclist(*plist);
        delete plist;

        ::CORBA::ULong len(rtclist.length());
        bool bflag;
        bflag = false;
        for (::CORBA::ULong ic = 0; ic < len; ++ic)
        {
            if( rtclist[ic] == inobj )
            {
                bflag = true;
                ::std::string str(profiles[ic].instance_name);
                CPPUNIT_ASSERT(str.find("DummyModule1") != ::std::string::npos);
                CPPUNIT_ASSERT_EQUAL(::std::string("DummyModule1"),
                                     ::std::string(profiles[ic].type_name));
                CPPUNIT_ASSERT_EQUAL(::std::string("Console input component"),
                                     ::std::string(profiles[ic].description));
                CPPUNIT_ASSERT_EQUAL(::std::string("1.0"),
                                     ::std::string(profiles[ic].version));
                CPPUNIT_ASSERT_EQUAL(::std::string("Noriaki Ando, AIST"),
                                     ::std::string(profiles[ic].vendor));
                CPPUNIT_ASSERT_EQUAL(::std::string("example"),
                                     ::std::string(profiles[ic].category));
                break;
            }
        }
        CPPUNIT_ASSERT_EQUAL( bflag,true );

        bflag = false;
        for (::CORBA::ULong ic = 0; ic < len; ++ic)
        {
            if( rtclist[ic] == outobj )
            {
                bflag = true;
                ::std::string str(profiles[ic].instance_name);
                CPPUNIT_ASSERT(str.find("DummyModule2") != ::std::string::npos);
                CPPUNIT_ASSERT_EQUAL(::std::string("DummyModule2"),
                                     ::std::string(profiles[ic].type_name));
                CPPUNIT_ASSERT_EQUAL(::std::string("Console output component"),
                                     ::std::string(profiles[ic].description));
                CPPUNIT_ASSERT_EQUAL(::std::string("1.0"),
                                     ::std::string(profiles[ic].version));
                CPPUNIT_ASSERT_EQUAL(::std::string("Noriaki Ando, AIST"),
                                     ::std::string(profiles[ic].vendor));
                CPPUNIT_ASSERT_EQUAL(::std::string("example"),
                                     ::std::string(profiles[ic].category));
                break;
            }
        }
        CPPUNIT_ASSERT_EQUAL( bflag,true );
        delete pman;
    }
    /*! 
     * @brief tests for get_profile()
     *
     *
     *
     */
    void test_get_profile()
    {
        typedef struct data_struct {
            ::std::string name;
            ::std::string value;
        } DATA_STRUCT;
        DATA_STRUCT manager_profile[] =
        {
            {"instance_name",            "manager"},
            {"name",                     "manager"},
            {"naming_formats",           "%h.host_cxt/%n.mgr"},
            {"pid",                      ""},
            {"refstring_path",           "/var/log/rtcmanager.ref"},
            {"modules.load_path",        ""},
            {"modules.abs_path_allowed", "YES"},
            {"modules.C++.manager_cmd",  ""},
            {"modules.C++.profile_cmd",  ""},
            {"modules.C++.suffixes",     ""},
            {"modules.C++.load_paths",   ""},
            {"modules.Python.manager_cmd",""},
            {"modules.Python.profile_cmd",""},
            {"modules.Python.suffixes",  ""},
            {"modules.Python.load_paths",""},
            {"modules.Java.manager_cmd", ""},
            {"modules.Java.profile_cmd", ""},
            {"modules.Java.suffixes",    ""},
            {"modules.Java.load_paths",  ""},
            {"modules.config_path",      ""},
            {"modules.download_allowed", ""},
            {"modules.init_func_suffix", ""},
            {"modules.init_func_prefix", ""},
//            {"modules.config_ext",       ""},
            {"is_master",                ""},
            {"corba_servant",            "YES"},
            {"shutdown_on_nortcs",       "YES"},
            {"shutdown_auto",            "YES"},
            {"command",                  "rtcd"},
            {"supported_languages",     ""},
            {"os.name",                  "Linux"},
            {"os.release",               ""},
            {"os.version",               ""},
            {"os.arch",                  ""},
            {"os.hostname",              ""},
            {"",""},
        };

        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();

        //Execute the functions.
        ::RTM::ManagerProfile *list;
        list = pman->get_profile();
        ::RTM::ManagerProfile profile(*list);
        delete list;
        int  len;
        len = profile.properties.length(); 
        CPPUNIT_ASSERT_EQUAL(34,len);
        for(int ic = 0; ic < len; ++ic) 
        {
            CPPUNIT_ASSERT_EQUAL(manager_profile[ic].name,
                               ::std::string(profile.properties[ic].name));
            const char* ch;
            if( profile.properties[ic].value >>= ch )
            {
                if(!manager_profile[ic].value.empty())
                {
                    CPPUNIT_ASSERT_EQUAL(manager_profile[ic].value,
                                         ::std::string(ch)); 
                }
            }
        }
        delete pman;
    }
    /*! 
     * @brief tests for get_configuration()
     *
     *
     *
     */
    void test_get_configuration()
    {
        typedef struct data_struct {
            ::std::string name;
            ::std::string value;
        } DATA_STRUCT;
        DATA_STRUCT config[] =
        {
            {"config.version",                  "1.0.0"},
            {"openrtm.version",                 "OpenRTM-aist-1.0.0"},
            {"manager.instance_name",           "manager"},
            {"manager.name",                    "manager"},
            {"manager.naming_formats",          "%h.host_cxt/%n.mgr"},
            {"manager.pid",                     ""},
            {"manager.refstring_path",          "/var/log/rtcmanager.ref"},
            {"manager.modules.load_path",       ""},
            {"manager.modules.abs_path_allowed","YES"},
            {"manager.modules.C++.manager_cmd",  ""},
            {"manager.modules.C++.profile_cmd",  ""},
            {"manager.modules.C++.suffixes",     ""},
            {"manager.modules.C++.load_paths",   ""},
            {"manager.modules.Python.manager_cmd",""},
            {"manager.modules.Python.profile_cmd",""},
            {"manager.modules.Python.suffixes",  ""},
            {"manager.modules.Python.load_paths",""},
            {"manager.modules.Java.manager_cmd", ""},
            {"manager.modules.Java.profile_cmd", ""},
            {"manager.modules.Java.suffixes",    ""},
            {"manager.modules.Java.load_paths",  ""},
            {"manager.modules.config_path",     ""},
            {"manager.modules.download_allowed",""},
            {"manager.modules.init_func_suffix",""},
            {"manager.modules.init_func_prefix",""},
            {"manager.is_master",               ""},
            {"manager.corba_servant",           "YES"},
            {"manager.shutdown_on_nortcs",      "YES"},
            {"manager.shutdown_auto",           "YES"},
            {"manager.command",                 "rtcd"},
            {"manager.supported_languages",     ""},
            {"manager.os.name",                 "Linux"},
            {"manager.os.release",              ""},
            {"manager.os.version",              ""},
            {"manager.os.arch",                 ""},
            {"manager.os.hostname",             ""},
            {"os.name",                         ""},
            {"os.release",                      ""},
            {"os.version",                      ""},
            {"os.arch",                         ""},
            {"os.hostname",                     ""},
            {"logger.enable",                   ""},
            {"logger.file_name",                ""},
            {"logger.date_format",              "%b %d %H:%M:%S"},
            {"logger.log_level",                ""},
            {"logger.stream_lock",              "NO"},
            {"logger.master_logger",            ""},
            {"module.conf_path",                ""},
            {"module.load_path",                ""},
            {"naming.enable",                   "YES"},
            {"naming.type",                     "corba"},
            {"naming.formats",                  "%h.host_cxt/%n.rtc"},
            {"naming.update.enable",            "YES"},
            {"naming.update.interval",          "10.0"},
            {"timer.enable",                    "YES"},
            {"timer.tick",                      "0.1"},
            {"corba.args",                      ""},
            {"corba.endpoint",                  ""},
            {"corba.id",                        "omniORB"},
            {"corba.nameservers",               ""},
            {"corba.master_manager",            "localhost:2810"},
            {"corba.nameservice.replace_endpoint", "NO"},
//            {"corba.endpoints",                  ""},
            {"exec_cxt.periodic.type",          "PeriodicExecutionContext"},
            {"exec_cxt.periodic.rate",          "1000"},
            {"exec_cxt.evdriven.type",          "EventDrivenExecutionContext"},
            {"example.DummyModule10.config_file",  ""},
            {"example.DummyModule1.config_file",   ""},
            {"example.DummyModule20.config_file", ""},
            {"example.DummyModule2.config_file",  ""},
            {"example.DummyModule11.config_file",  ""},
            {"example.DummyModule21.config_file", ""},
            {"example.DummyModule12.config_file",  ""},
            {"example.DummyModule22.config_file", ""},
            {"",""},
        };

        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
        ::RTM::NVList* list;
        list = pman->get_configuration();
        ::RTM::NVList conf(*list);
        delete list;
        ::CORBA::ULong  len;
        len = conf.length(); 
        CPPUNIT_ASSERT_EQUAL((::CORBA::ULong)73,len);
        for(::CORBA::ULong ic = 0; ic < len; ++ic) 
        {
            CPPUNIT_ASSERT_EQUAL(config[ic].name,
                               ::std::string(conf[ic].name));
            const char* ch;
            if( conf[ic].value >>= ch )
            {
                if(!config[ic].value.empty())
                {
                    CPPUNIT_ASSERT_EQUAL(config[ic].value,
                                         ::std::string(ch)); 
                }
            }
        }
        delete pman;
    }
    /*! 
     * @brief tests for set_configuration()
     *
     *
     *
     */
    void test_set_configuration()
    {
        typedef struct data_struct {
            ::std::string name;
            ::std::string value;
        } DATA_STRUCT;
        DATA_STRUCT config[] =
        {
            {"config.version",                  "1.0.0"},
            {"openrtm.version",                 "OpenRTM-aist-1.0.0"},
            {"manager.naming_formats",          "%n.rtc"},
            {"manager.modules.load_path",       "./,./.libs"},
            {"manager.modules.abs_path_allowed","NO"},
            {"manager.os.release",              "2.6.22-14-generic"},
            {"manager.os.version",              "2008"},
            {"manager.os.arch",                 "64"},
            {"manager.os.hostname",             "ubuntur810"},
        };
        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
        ::RTC::ReturnCode_t ret;
        int len;
        len = sizeof config/sizeof config[0]; 
        for(int ic = 0; ic < len; ++ic) 
        {
            ret = pman->set_configuration(config[ic].name.c_str(), 
                                          config[ic].value.c_str());
            CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
        }

        ::RTM::NVList* list;
        list = pman->get_configuration();
        ::RTM::NVList conf(*list);
        delete list;
        ::CORBA::ULong  leng;
        leng = conf.length(); 
        CPPUNIT_ASSERT_EQUAL((::CORBA::ULong)73,leng);
        for(::CORBA::ULong ic = 0; ic < leng; ++ic) 
        {
            if(config[0].name == ::std::string(conf[ic].name))
            {
                CPPUNIT_ASSERT_EQUAL(config[ic].name,
                                     ::std::string(conf[ic].name));
                const char* ch;
                if( conf[ic].value >>= ch )
                {
                    if(!config[ic].value.empty())
                    {
                        CPPUNIT_ASSERT_EQUAL(config[ic].value,
                                             ::std::string(ch)); 
                    }
                }
            }
        }
        delete pman;
    }

    /*! 
     * @brief tests for shutdown()
     *
     *
     *
     */
    void test_shutdown()
    {

        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();

        try
        {
            ::RTC::ReturnCode_t retcode;
            retcode = pman->shutdown();
            CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, retcode);
            ::coil::sleep(3);
            delete pman;
        }
        catch(...)
        {
	    CPPUNIT_FAIL("Exception thrown.");
        }
        delete pman;
    }
    /*! 
     * @brief tests for get_loadable_modules()
     *
     *
     *
     */
    void test_get_loadable_modules()
    {
        ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();

        //ロード可能なモジュールリストを取得する
        ::RTC::ReturnCode_t ret;
//        ret = pman->set_configuration("manager.modules.load_path", "./.libs");
// rtc.conf に入れないと有効にならないね！

        try
        {
            ret = pman->load_module(".libs/DummyModule1.so","DummyModule1Init");
            CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
            CPPUNIT_ASSERT(isFound(pman->get_loadable_modules(), 
                                   "DummyModule1"));
        }
        catch(...)
        {
	    CPPUNIT_FAIL("Exception thrown.");
        }

        //
        try
        {
            ret = pman->load_module(".libs/DummyModule2.so","DummyModule2Init");
            CPPUNIT_ASSERT_EQUAL(::RTC::RTC_OK, ret);
            CPPUNIT_ASSERT(isFound(pman->get_loadable_modules(), 
                                   "DummyModule2"));
        }
        catch(...)
        {
	    CPPUNIT_FAIL("Exception thrown.");
        }

        //Execute the function
        ::RTM::ModuleProfileList* list;
        list = pman->get_loadable_modules();
        ::RTM::ModuleProfileList modlist(*list);
        delete list;

        //Check returns(ModuleProfileList).
        CPPUNIT_ASSERT_EQUAL((::CORBA::ULong)2, modlist.length());

        CORBA::Long long_ret = NVUtil::find_index(modlist[0].properties,"module_file_name");
        CPPUNIT_ASSERT(long_ret!=-1);

        const char* ch;
        if( modlist[0].properties[long_ret].value >>= ch )
        {
            CPPUNIT_ASSERT_EQUAL(::std::string("DummyModule2.so"), 
                                 ::std::string(ch));
        }
        else
        {
            CPPUNIT_FAIL( "ModuleProfileList is illegal." );
        }

        long_ret = NVUtil::find_index(modlist[1].properties,"module_file_name");
        CPPUNIT_ASSERT(long_ret!=-1);

        if( modlist[1].properties[long_ret].value >>= ch )
        {
            CPPUNIT_ASSERT_EQUAL(::std::string("DummyModule1.so"), 
                                 ::std::string(ch));
        }
        else
        {
            CPPUNIT_FAIL( "ModuleProfileList is illegal." );
        }
        delete pman;
    }


    /*! 
     * @brief tests for fork()
     *
     *
     *
     */
    void test_fork()
    {
      ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
      CPPUNIT_ASSERT(pman->fork() == ::RTC::RTC_OK);
      delete pman;
    }

    /*! 
     * @brief tests for restart()
     *
     *
     *
     */
    void test_restart()
    {
      ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
      CPPUNIT_ASSERT(pman->restart() == ::RTC::RTC_OK);
      delete pman;
    }

    /*! 
     * @brief tests for get_service()
     *
     *
     *
     */
    void test_get_service()
    {
      std::string name("service0");
      ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
      CPPUNIT_ASSERT(CORBA::is_nil(pman->get_service(name.c_str())));
      delete pman;
    }

    /*! 
     * @brief tests for getObjRef()
     *
     *
     *
     */
    void test_getObjRef()
    {
      ::RTM::ManagerServant *pman = new ::RTM::ManagerServant();
      m_objref = pman->getObjRef();
      //CPPUNIT_ASSERT(! CORBA::is_nil(m_objref));
      delete pman;
    }

  };
}; // namespace ManagerServant

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ManagerServant::ManagerServantTests);

#ifdef LOCAL_MAIN
int main(int argc, char* argv[])
{
    CppUnit::TextUi::TestRunner runner;
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry().makeTest());
    CppUnit::Outputter* outputter = 
      new CppUnit::TextOutputter(&runner.result(), std::cout);
    runner.setOutputter(outputter);
    bool retcode = runner.run();
    return !retcode;
}
#endif // MAIN
#endif // ManagerServant_cpp
