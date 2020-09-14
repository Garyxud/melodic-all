// -*- C++ -*-
/*!
 * @file   ConfigurationProxyTests.cpp
 * @brief  ConfigurationProxy test class
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

#ifndef ConfigurationProxy_cpp
#define ConfigurationProxy_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <string>
#include <iostream>
#include <idl/SDOPackageSkel.h>
#include <idl/RTCSkel.h>
#include <ConfigurationProxy.h>
#include <rtm/NVUtil.h>
#include <ISDOService.h>
#include <ISDOSystemElement.h>
#include <IOrganization.h>
#include <doil/corba/CORBAManager.h>


/*!
 * @class ConfigurationProxyTests class
 * @brief ConfigurationProxy test
 */
namespace ConfigurationProxy
{
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

  /*!
   * 
   * 
   *
   */

  class OrganizationServantMock
   : public virtual ::POA_SDOPackage::Organization,
     public virtual ::doil::CORBA::CORBAServantBase
  {
  public:
    OrganizationServantMock(doil::ImplBase* impl)
    : ::doil::CORBA::CORBAServantBase(impl), m_impl(NULL) 
    {
      m_impl = dynamic_cast< ::SDOPackage::Local::IOrganization* >(impl);
    }
    virtual ~OrganizationServantMock(){}
    virtual char* get_organization_id(){return "OrganizationServantMock";}
    virtual ::SDOPackage::OrganizationProperty* get_organization_property()
    {
      ::SDOPackage::OrganizationProperty_var ret 
                            = new ::SDOPackage::OrganizationProperty ();
      return ret._retn();
    }
    virtual ::CORBA::Any* get_organization_property_value(const char* name)
    {
	CORBA::Any_var value;
	value = new CORBA::Any();
	return value._retn();
//      return new ::CORBA::Any();
    }
    virtual ::CORBA::Boolean set_organization_property(
          const ::SDOPackage::OrganizationProperty& organization_property)
    {
	return true;
    }
    virtual ::CORBA::Boolean set_organization_property_value(
          const char* name, const CORBA::Any& value)
    {
	return true;
    }
    virtual ::CORBA::Boolean remove_organization_property(const char* name)
    {
	return true;
    }
    virtual ::SDOPackage::SDOSystemElement_ptr get_owner()
    {
        return m_varOwner._retn();
    }
    virtual ::CORBA::Boolean set_owner(
          ::SDOPackage::SDOSystemElement_ptr sdo)
    { 
        return true; 
    }
    virtual ::SDOPackage::SDOList* get_members()
    {
//        ::SDOPackage::SDOList* ret = new ::SDOPackage::SDOList ();
//	return ret;
	::SDOPackage::SDOList_var sdos;
	sdos = new ::SDOPackage::SDOList(m_memberList);
	return sdos._retn();
    }
    virtual CORBA::Boolean set_members(
          const ::SDOPackage::SDOList& sdos)
    {
	m_memberList = sdos;
	return true;
    }
    virtual CORBA::Boolean add_members(
          const ::SDOPackage::SDOList& sdo_list)
    {
        return true; 
    }
    virtual CORBA::Boolean remove_member(const char* id){return true;}
    virtual ::SDOPackage::DependencyType get_dependency()
    {
        return m_dependency;
    }
    virtual CORBA::Boolean set_dependency(
          ::SDOPackage::DependencyType dependency) 
    { 
	m_dependency = dependency;
        return true; 
    }


  private:
    ::SDOPackage::Local::IOrganization* m_impl;
    ::SDOPackage::SDOSystemElement_var m_varOwner;
    ::SDOPackage::DependencyType m_dependency;
    ::SDOPackage::SDOList m_memberList;
  };

  /*!
   * 
   * 
   *
   */
  class ISDOServiceServantMock
   : public virtual ::POA_SDOPackage::SDOService,
     public virtual ::doil::CORBA::CORBAServantBase
  {
  public:
    ISDOServiceServantMock(doil::ImplBase* impl)
    : ::doil::CORBA::CORBAServantBase(impl)
    {
      m_impl = dynamic_cast< ::SDOPackage::Local::ISDOService* >(impl);
    }
    virtual ~ISDOServiceServantMock(){}


  private:
    ::SDOPackage::Local::ISDOService* m_impl;
  };
  /*!
   * 
   * 
   *
   */
  class ISDOServiceMock
    : public virtual ::SDOPackage::Local::ISDOService
  {
  public:
    ISDOServiceMock()
      : m_refcount(0)
    {}
    virtual ~ISDOServiceMock(){}
    const char* id() {return "ISDOServiceMock";}
    const char* name() {return "ISDOServiceMock";}
    void incRef()
    {
      ++m_refcount;
    }
    void decRef()
    {
      --m_refcount;
      if (m_refcount == 0)
        delete this;
    }

  private:
    std::string m_name;
    int m_refcount;
  };
  /*!
   * 
   * 
   *
   */
  class IOrganizationMock
    : public virtual ::SDOPackage::Local::IOrganization
  {
  public:
    IOrganizationMock()
      : m_refcount(0)
    {}
    virtual ~IOrganizationMock(){}

    virtual ::std::string get_organization_id()
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        return "IOrganizationMock";
    }

    virtual ::SDOPackage::Local::OrganizationProperty get_organization_property()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
       ::SDOPackage::Local::OrganizationProperty ret;
       return ret;
    }

    virtual ::std::string get_organization_property_value(const ::std::string& name)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        return "IOrganizationMock";
    }
    virtual bool set_organization_property(const ::SDOPackage::Local::OrganizationProperty& organization_property)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        return true;       
    }

    virtual bool set_organization_property_value(const ::std::string&name, const ::std::string& value)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        return true;       
    }

    virtual bool remove_organization_property(const ::std::string& name)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        return true;       
    }

    virtual ::SDOPackage::Local::ISDOSystemElement* get_owner()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        return NULL;
    }

    virtual bool set_owner(const ::SDOPackage::Local::ISDOSystemElement* sdo)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        return true;
    }

    virtual ::SDOPackage::Local::SDOList get_members()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        ::SDOPackage::Local::SDOList ret; 
        return ret;
    }

    virtual bool set_members(const ::SDOPackage::Local::SDOList& sdos)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        return true;
    }

    virtual bool add_members(const ::SDOPackage::Local::SDOList& sdo_list)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        return true;
    }

    virtual bool remove_member(const ::std::string& id)
      throw (::SDOPackage::Local::InvalidParameter,
             ::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        return true;
    }

    virtual ::SDOPackage::Local::DependencyType get_dependency()
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        return m_dependency;
    }

    virtual bool set_dependency(::SDOPackage::Local::DependencyType dependency)
      throw (::SDOPackage::Local::NotAvailable,
             ::SDOPackage::Local::InternalError)
    {
        m_dependency = dependency;
        return true;
    }


    const char* id() {return "IOrganizationMock";}
    const char* name() {return "IOrganizationMock";}
    void incRef()
    {
      ++m_refcount;
    }
    void decRef()
    {
      --m_refcount;
      if (m_refcount == 0)
        delete this;
    }

  private:
    std::string m_name;
    int m_refcount;
    ::SDOPackage::Local::DependencyType m_dependency;
   };
  /*!
   * 
   * 
   *
   */
  class ConfigurationMock
    : public virtual ::POA_SDOPackage::Configuration
  {
  protected:
      ::std::vector<std::string> m_log;
  private:
      ::std::string m_name;
      ::std::string m_code;
      ::std::string m_val;
      ::SDOPackage::Parameter m_par;
      ::SDOPackage::StringList m_stlist;
      ::SDOPackage::EnumerationType m_etype;
      ::CORBA::Any m_any;
  public :
      ConfigurationMock(){}
      virtual ~ConfigurationMock()
      {
      }

      void setLogger(Logger* logger)
      {
        m_logger = logger;
      }

    /*! 
     *
     */
     ::CORBA::Boolean set_device_profile(const ::SDOPackage::DeviceProfile& dProfile)
       throw (CORBA::SystemException,
             ::SDOPackage::InvalidParameter, 
             ::SDOPackage::NotAvailable, 
             ::SDOPackage::InternalError)
     {
        if (m_logger != NULL) m_logger->log("set_device_profile");
        return true;
     }
    /*! 
     *
     */
     ::CORBA::Boolean set_service_profile(const ::SDOPackage::ServiceProfile& sProfile)
       throw (CORBA::SystemException,
   	      ::SDOPackage::InvalidParameter, 
              ::SDOPackage::NotAvailable, 
              ::SDOPackage::InternalError)
     {
        if (m_logger != NULL) m_logger->log("set_service_profile");
        return true;
     }
    /*! 
     *
     */
     ::CORBA::Boolean add_organization(::SDOPackage::Organization_ptr org)
       throw (CORBA::SystemException,
   	      ::SDOPackage::InvalidParameter, 
              ::SDOPackage::NotAvailable, 
              ::SDOPackage::InternalError)
     {
        if (m_logger != NULL) m_logger->log("add_organization");
        return true;
     }
    /*! 
     *
     */
     ::CORBA::Boolean remove_service_profile(const char* id)
       throw (CORBA::SystemException,
	      ::SDOPackage::InvalidParameter, 
              ::SDOPackage::NotAvailable, 
              ::SDOPackage::InternalError)
     {
        if (m_logger != NULL)
        {
            m_logger->log("remove_service_profile");
            m_logger->log(id);
        }
        return true;
     }
    /*! 
     *
     */
     ::CORBA::Boolean remove_organization(const char* organization_id)
       throw (CORBA::SystemException,
	      ::SDOPackage::InvalidParameter, 
              ::SDOPackage::NotAvailable, 
              ::SDOPackage::InternalError)
     {
        if (m_logger != NULL)
        { 
            m_logger->log("remove_organization");
            m_logger->log(organization_id);
        }
        return true;
     }
    /*! 
     *
     */
     ::SDOPackage::ParameterList* get_configuration_parameters()
       throw (CORBA::SystemException,
	       ::SDOPackage::NotAvailable, 
               ::SDOPackage::InternalError)
     {
        if (m_logger != NULL)
        {
            m_logger->log("get_configuration_parameters");
        }
        m_name = "test_get_configuration_parameters";
//        m_code = "5";
//        m_any <<= m_code.c_str();
        m_any <<= (short)5;
        m_par.name = m_name.c_str();
        m_par.type = m_any.type();
        char* cp = "999";
        ::SDOPackage::StringList* stlist;
        stlist = new ::SDOPackage::StringList((::CORBA::ULong)1,
                                              (::CORBA::ULong)1,
                                              &cp,
                                              (::CORBA::Boolean)0);
        m_stlist = *stlist;
        m_etype.enumerated_values = m_stlist;
        delete stlist;
        m_par.allowed_values.allowed_enum(m_etype);

        ::SDOPackage::ParameterList_var param;
        param = new ::SDOPackage::ParameterList((::CORBA::ULong)1,
                                                (::CORBA::ULong)1,
                                                 &m_par,
                                                (::CORBA::Boolean)0);

     
        return param._retn();
     }
    /*! 
     *
     */
     ::SDOPackage::NVList* get_configuration_parameter_values()
       throw (CORBA::SystemException,
	      ::SDOPackage::NotAvailable, 
              ::SDOPackage::InternalError)
     {  
        if (m_logger != NULL) 
        {
            m_logger->log("get_configuration_parameter_values");
        }
        
        const char* name_str = "get_configuration_parameter_values_name";
        const char* value_str = "5";
        ::SDOPackage::NVList nv;
        ::NVUtil::appendStringValue(nv,
                                  ::CORBA::string_dup(name_str),
                                  ::CORBA::string_dup(value_str));
        ::SDOPackage::NVList_var nvlist;
        nvlist = new ::SDOPackage::NVList(nv);
        return nvlist._retn();
     }
    /*! 
     *
     */
     ::CORBA::Any* get_configuration_parameter_value(const char* name)
       throw (CORBA::SystemException,
	      ::SDOPackage::InvalidParameter, 
              ::SDOPackage::NotAvailable, 
              ::SDOPackage::InternalError)
     {
        if (m_logger != NULL) 
        {
            m_logger->log("get_configuration_parameter_value");
            m_logger->log(name);
        }
        if (::std::string(name).empty()) 
            throw ::SDOPackage::InvalidParameter("Name is empty.");
        CORBA::Any val;
        val <<= name;

        CORBA::Any_var value;
        value = new CORBA::Any(val);
        return value._retn();
     }
    /*! 
     *
     */
     ::CORBA::Boolean set_configuration_parameter(const char* name,
						  const CORBA::Any& value)
       throw (CORBA::SystemException,
	      ::SDOPackage::InvalidParameter, 
              ::SDOPackage::NotAvailable, 
              ::SDOPackage::InternalError)
     {
        if (m_logger != NULL) 
        {
            m_logger->log("set_configuration_parameter");
            m_logger->log(name);

            const char* ch;
            ::CORBA::ULong ul = 4;
            ::CORBA::Any::to_string to_str(ch, ul);
            value >>= to_str;
            m_logger->log(to_str.val);

        }
        return true;
     }
    /*! 
     *
     */
     ::SDOPackage::ConfigurationSetList* get_configuration_sets()
       throw (CORBA::SystemException,
	      ::SDOPackage::NotAvailable, 
              ::SDOPackage::InternalError)
     {
        const char* id_str = "test_get_configuration_sets_id";
        const char* description_str = 
                         "test_get_configuration_sets_description";
        const char* name_str = "test_get_configuration_sets_nv_name";
        const char* value_str = "5";
        if (m_logger != NULL)
        { 
            m_logger->log("get_configuration_sets");
        }


	::SDOPackage::ConfigurationSetList_var config_sets;
	config_sets 
              = new ::SDOPackage::ConfigurationSetList((CORBA::ULong)0);
        config_sets->length(1);
        config_sets[0].id = ::CORBA::string_dup(id_str);
        config_sets[0].description = ::CORBA::string_dup(description_str);
        ::NVUtil::appendStringValue(config_sets[0].configuration_data,
                                    ::CORBA::string_dup(name_str),
                                    ::CORBA::string_dup(value_str));

	return config_sets._retn();

     }
  
    /*! 
     *
     */
     ::SDOPackage::ConfigurationSet* get_configuration_set(const char* id)
       throw (CORBA::SystemException,
	      ::SDOPackage::NotAvailable, 
              ::SDOPackage::InternalError)
     {
        const char* id_str = "test_get_configuration_set_id";
        const char* description_str = 
                         "test_get_configuration_set_description";
        const char* name_str = "test_get_configuration_set_nv_name";
        const char* value_str = "5";

        if (m_logger != NULL)
        { 
            m_logger->log("get_configuration_set");
            m_logger->log(id);
        }
        if (std::string(id).empty()) 
            throw ::SDOPackage::InternalError("ID is empty");

        ::SDOPackage::ConfigurationSet_var config;
        config = new ::SDOPackage::ConfigurationSet();
//        ::SDOPackage::ConfigurationSet* obj;
//        obj = new ::SDOPackage::ConfigurationSet();
//        ::SDOPackage::ConfigurationSet_ptr config = obj->_this();
        

        config->id.out() = ::CORBA::string_dup(id_str);
        config->description.out() = ::CORBA::string_dup(description_str);
        ::NVUtil::appendStringValue(config->configuration_data,
                                    ::CORBA::string_dup(name_str),
                                    ::CORBA::string_dup(value_str));

	return config._retn();

      }
  
    /*! 
     *
     */
    ::CORBA::Boolean set_configuration_set_values(
             const char* id,
             const ::SDOPackage::ConfigurationSet& configuration_set)
      throw (CORBA::SystemException,
	     ::SDOPackage::InvalidParameter, 
             ::SDOPackage::NotAvailable, 
             ::SDOPackage::InternalError)
    {
        if (m_logger != NULL)
        { 
            m_logger->log("set_configuration_set_values");
            m_logger->log(id);
            m_logger->log(configuration_set.id.in());
            m_logger->log(configuration_set.description.in());
            m_logger->log(configuration_set.configuration_data[0].name.in());
            ::CORBA::Any any;
//            any = ::NVUtil::find(
//                      configuration_set.configuration_data,
//                      configuration_set.configuration_data[0].name.in());
            any = configuration_set.configuration_data[0].value;
            const char* ch;
            ::CORBA::ULong ul = 1;
            ::CORBA::Any::to_string to_str(ch, ul);
            any >>= to_str;
            m_logger->log(to_str.val);
        }
        if (std::string(id).empty()) 
            throw ::SDOPackage::InvalidParameter("ID is empty.");
	return true;
    }
    /*! 
     *
     */
    ::SDOPackage::ConfigurationSet* get_active_configuration_set()
      throw (CORBA::SystemException,
	     ::SDOPackage::NotAvailable, 
             ::SDOPackage::InternalError)
    {
        const char* id_str = "test_get_active_configuration_set_id";
        const char* description_str = 
                         "test_get_active_configuration_set_description";
        const char* name_str = "test_get_active_configuration_set_nv_name";
        const char* value_str = "5";
        if (m_logger != NULL) 
        {
          m_logger->log("get_active_configuration_set");
        }
	::SDOPackage::ConfigurationSet_var config;
	config = new ::SDOPackage::ConfigurationSet();
        config->id.out() = ::CORBA::string_dup(id_str);
        config->description.out() = ::CORBA::string_dup(description_str);
        ::NVUtil::appendStringValue(config->configuration_data,
                                    ::CORBA::string_dup(name_str),
                                    ::CORBA::string_dup(value_str));
	return config._retn();
    }
    /*! 
     *
     */
    ::CORBA::Boolean add_configuration_set(
             const ::SDOPackage::ConfigurationSet& configuration_set)
      throw (CORBA::SystemException,
	     ::SDOPackage::InvalidParameter, 
             ::SDOPackage::NotAvailable, 
             ::SDOPackage::InternalError)
    {
        if (m_logger != NULL) 
        {
            m_logger->log("add_configuration_set");
            m_logger->log(configuration_set.id.in());
            m_logger->log(configuration_set.description.in());
            m_logger->log(configuration_set.configuration_data[0].name.in());
            ::CORBA::Any any;
            any = configuration_set.configuration_data[0].value;
            const char* ch;
            ::CORBA::ULong ul = 1;
            ::CORBA::Any::to_string to_str(ch, ul);
            any >>= to_str;
            m_logger->log(to_str.val);
       }
        return true;
    }
    /*! 
     *
     */
    ::CORBA::Boolean remove_configuration_set(const char* id)
      throw (CORBA::SystemException,
	     ::SDOPackage::InvalidParameter, 
             ::SDOPackage::NotAvailable, 
             ::SDOPackage::InternalError)
    {
        if (m_logger != NULL) 
        {
            m_logger->log("remove_configuration_set");
            m_logger->log(id);
        }
        if (std::string(id).empty())
          throw ::SDOPackage::InvalidParameter("ID is empty.");
        return false;
    }
    /*! 
     *
     */
    ::CORBA::Boolean activate_configuration_set(const char* id)
      throw (CORBA::SystemException,
	     ::SDOPackage::InvalidParameter, 
             ::SDOPackage::NotAvailable, 
             ::SDOPackage::InternalError)
    {
        if (m_logger != NULL) 
        {
            m_logger->log("activate_configuration_set");
            m_logger->log(id);
        }
        if (std::string(id).empty())
          throw ::SDOPackage::InvalidParameter("ID is empty.");
        return true;
    }
  private:
    Logger* m_logger;

  };

  class ConfigurationProxyTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ConfigurationProxyTests);
    CPPUNIT_TEST(test_set_device_profile);
    CPPUNIT_TEST(test_set_service_profile);
    CPPUNIT_TEST(test_add_organization);
    CPPUNIT_TEST(test_remove_service_profile);
    CPPUNIT_TEST(test_remove_organization);
    CPPUNIT_TEST(test_get_configuration_parameters);
    CPPUNIT_TEST(test_get_configuration_parameter_values);
    CPPUNIT_TEST(test_get_configuration_parameter_value);
    CPPUNIT_TEST(test_set_configuration_parameter);
    CPPUNIT_TEST(test_get_configuration_sets);
    CPPUNIT_TEST(test_get_configuration_set);
    CPPUNIT_TEST(test_set_configuration_set_values);
    CPPUNIT_TEST(test_get_active_configuration_set);
    CPPUNIT_TEST(test_add_configuration_set);
    CPPUNIT_TEST(test_remove_configuration_set);
    CPPUNIT_TEST(test_activate_configuration_set);
    CPPUNIT_TEST_SUITE_END();
  
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
  
  public:
  
    /*!
     * @brief Constructor
     */
    ConfigurationProxyTests()
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
    ~ConfigurationProxyTests()
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
     *
     *
     *
     */
    void test_set_device_profile()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("set_device_profile"));
      const ::SDOPackage::Local::DeviceProfile porf;
      ap->set_device_profile(porf);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("set_device_profile"));

      

      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_set_service_profile()
    {
      doil::CORBA::CORBAManager& 
                            mgr(doil::CORBA::CORBAManager::instance());
      std::cout <<"Manager Name==>"<< mgr.name() << std::endl;

      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("set_service_profile"));
      ::SDOPackage::Local::ServiceProfile porf;
      ISDOServiceMock service;

      mgr.registerFactory(service.id(), 
                          doil::New<ISDOServiceServantMock>,
                          doil::Delete<ISDOServiceServantMock>);
      mgr.activateObject(&service);

      porf.service = &service;
      const ::SDOPackage::Local::ServiceProfile _porf = porf;
      ap->set_service_profile(_porf);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("set_service_profile"));

      mgr.deactivateObject(&service);

      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_add_organization()
    {
      doil::CORBA::CORBAManager& 
                            mgr(doil::CORBA::CORBAManager::instance());
      std::cout <<"Manager Name==>"<< mgr.name() << std::endl;

      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("add_organization"));

      IOrganizationMock org;
      mgr.registerFactory(org.id(), 
                          doil::New<OrganizationServantMock>,
                          doil::Delete<OrganizationServantMock>);
      mgr.activateObject(&org);
      const IOrganizationMock _org(org);
      ap->add_organization(&_org);

      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("add_organization"));
      mgr.deactivateObject(&org);

      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_remove_service_profile()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      const ::std::string str = "test_remove_service_profile";
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("remove_service_profile"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str));
      ap->remove_service_profile(str);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("remove_service_profile"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str));

      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_remove_organization()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);
      const ::std::string str = "test_remove_organization";

      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("remove_organization"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str));
      ap->remove_organization(str);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("remove_organization"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str));

      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_get_configuration_parameters()
    {
std::cout<<"test_get_configuration_parameters"<<std::endl;
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      ::SDOPackage::Local::ParameterList pl;
      CPPUNIT_ASSERT_EQUAL(0, 
                        logger.countLog("get_configuration_parameters"));
      pl = ap->get_configuration_parameters();
      CPPUNIT_ASSERT_EQUAL(1, 
                        logger.countLog("get_configuration_parameters"));
      CPPUNIT_ASSERT(pl[0].name == 
                             "test_get_configuration_parameters");


      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_get_configuration_parameter_values()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      ::SDOPackage::Local::NVList nvlist;
      CPPUNIT_ASSERT_EQUAL(0, 
                  logger.countLog("get_configuration_parameter_values"));
      nvlist = ap->get_configuration_parameter_values();
      CPPUNIT_ASSERT_EQUAL(1, 
                  logger.countLog("get_configuration_parameter_values"));

     
      CPPUNIT_ASSERT(nvlist[0].name == 
                             "get_configuration_parameter_values_name");
      CPPUNIT_ASSERT(nvlist[0].value == "5");

      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_get_configuration_parameter_value()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      const ::std::string str = "test_get_configuration_parameter_value";
      ::std::string ret;
      CPPUNIT_ASSERT_EQUAL(0, 
                  logger.countLog("get_configuration_parameter_value"));
      ret = ap->get_configuration_parameter_value(str);
      CPPUNIT_ASSERT_EQUAL(1, 
                  logger.countLog("get_configuration_parameter_value"));

      CPPUNIT_ASSERT(ret == str);
      delete ap;
      CORBA::release(ref);

    }
    /*! 
     *
     *
     *
     */
    void test_set_configuration_parameter()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      const ::std::string str1 = "configuration_parameter";
      const ::std::string str2 = "5555";
      CPPUNIT_ASSERT_EQUAL(0, 
                        logger.countLog("set_configuration_parameter"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str1));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str2));
      ap->set_configuration_parameter(str1,str2);
      CPPUNIT_ASSERT_EQUAL(1, 
                        logger.countLog("set_configuration_parameter"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str1));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str2));


      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_get_configuration_sets()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);
      ::SDOPackage::Local::ConfigurationSetList configlist;

      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("get_configuration_sets"));
      configlist = ap->get_configuration_sets();
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("get_configuration_sets"));

      CPPUNIT_ASSERT(configlist[0].id == 
                         "test_get_configuration_sets_id");
      CPPUNIT_ASSERT(configlist[0].description == 
                         "test_get_configuration_sets_description");
      CPPUNIT_ASSERT(configlist[0].configuration_data[0].name == 
                         "test_get_configuration_sets_nv_name");
      CPPUNIT_ASSERT(configlist[0].configuration_data[0].value ==  "5");


      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_get_configuration_set()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      const ::std::string str = "get_configuration_set_config_id";
      ::SDOPackage::Local::ConfigurationSet confset;

      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("get_configuration_set"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str));
      confset = ap->get_configuration_set(str);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("get_configuration_set"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str));

      CPPUNIT_ASSERT(confset.id == 
                         "test_get_configuration_set_id");
      CPPUNIT_ASSERT(confset.description == 
                         "test_get_configuration_set_description");
      CPPUNIT_ASSERT(confset.configuration_data[0].name == 
                         "test_get_configuration_set_nv_name");
      CPPUNIT_ASSERT(confset.configuration_data[0].value ==  "5");

      delete ap;
      CORBA::release(ref);

    }
    /*! 
     *
     *
     *
     */
    void test_set_configuration_set_values()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      const ::std::string str = "set_configuration_set_values_config_id";
      const ::std::string id_str = "test_set_configuration_set_valuest_id";
      const ::std::string description_str = 
                          "test_set_configuration_set_values_description";
      const ::std::string name_str = 
                          "test_set_configuration_set_values_nv_name";
      const ::std::string value_str = "5";

      ::SDOPackage::Local::NameValue nv; 
      nv.name  = name_str;
      nv.value = value_str;
      ::SDOPackage::Local::NVList nvlist;
      nvlist.push_back(nv);

      ::SDOPackage::Local::ConfigurationSet configuration_set;
      configuration_set.id = id_str;
      configuration_set.description = description_str;
      configuration_set.configuration_data = nvlist; 

      CPPUNIT_ASSERT_EQUAL(0, 
                        logger.countLog("set_configuration_set_values"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str));
      ap->set_configuration_set_values(str,configuration_set);
      CPPUNIT_ASSERT_EQUAL(1, 
                        logger.countLog("set_configuration_set_values"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str));

      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_get_active_configuration_set()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      ::SDOPackage::Local::ConfigurationSet confset;
      CPPUNIT_ASSERT_EQUAL(0, 
                        logger.countLog("get_active_configuration_set"));
      confset = ap->get_active_configuration_set();
      CPPUNIT_ASSERT_EQUAL(1, 
                        logger.countLog("get_active_configuration_set"));

      CPPUNIT_ASSERT(confset.id == 
                         "test_get_active_configuration_set_id");
      CPPUNIT_ASSERT(confset.description == 
                         "test_get_active_configuration_set_description");
      CPPUNIT_ASSERT(confset.configuration_data[0].name == 
                         "test_get_active_configuration_set_nv_name");
      CPPUNIT_ASSERT(confset.configuration_data[0].value ==  "5");

      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_add_configuration_set()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      const ::std::string id_str = "test_add_configuration_set_id";
      const ::std::string description_str = 
                                 "test_add_configuration_set_description";
      const ::std::string name_str = "test_add_configuration_set_nv_name";
      const ::std::string value_str = "5";

      ::SDOPackage::Local::NameValue nv; 
      nv.name  = name_str;
      nv.value = value_str;
      ::SDOPackage::Local::NVList nvlist;
      nvlist.push_back(nv);

      ::SDOPackage::Local::ConfigurationSet configuration_set;
      configuration_set.id = id_str;
      configuration_set.description = description_str;
      configuration_set.configuration_data = nvlist; 

      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("add_configuration_set"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(id_str));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(description_str));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(name_str));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(value_str));
      ap->add_configuration_set(configuration_set);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("add_configuration_set"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(id_str));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(description_str));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(name_str));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(value_str));
      delete ap;
      CORBA::release(ref);

    }
    /*! 
     *
     *
     *
     */
    void test_remove_configuration_set()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      const ::std::string str = "test_remove_configuration_set";
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog("remove_configuration_set"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str));
      ap->remove_configuration_set(str);
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog("remove_configuration_set"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str));

      delete ap;
      CORBA::release(ref);
    }
    /*! 
     *
     *
     *
     */
    void test_activate_configuration_set()
    {
      ConfigurationMock* obj = new ConfigurationMock();
      ::CORBA::Object_ptr ref = obj->_this();
      if(::CORBA::is_nil(ref))
      {
         std::cout<<"ref is nil.Abort test."<<std::endl;
         delete obj;
         return;
      }
      ::SDOPackage::CORBA::ConfigurationProxy* ap 
                 = new ::SDOPackage::CORBA::ConfigurationProxy(ref);

      Logger logger;
      obj->setLogger(&logger);

      const ::std::string str = "test_activate_configuration_set";
      CPPUNIT_ASSERT_EQUAL(0, 
                           logger.countLog("activate_configuration_set"));
      CPPUNIT_ASSERT_EQUAL(0, logger.countLog(str));
      ap->activate_configuration_set(str);
      CPPUNIT_ASSERT_EQUAL(1, 
                           logger.countLog("activate_configuration_set"));
      CPPUNIT_ASSERT_EQUAL(1, logger.countLog(str));

      delete ap;
      CORBA::release(ref);
    }
    /* test case */
    void test_case0()
    {
    }
  };
}; // namespace ConfigurationProxy

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ConfigurationProxy::ConfigurationProxyTests);

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
#endif // ConfigurationProxy_cpp
