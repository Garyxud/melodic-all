// -*- C++ -*-
/*!
 * @file   SdoConfigurationTests.cpp
 * @brief  SdoConfiguration test class
 * @date   $Date: 2008/04/23 10:43:41 $
 * @author Shinji Kurihara
 *         Noriaki Ando <n-ando@aist.go.jp>
 * 
 * Copyright (C) 2006
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

/*
 * $Log: SdoConfigurationTests.cpp,v $
 * Revision 1.3  2008/04/23 10:43:41  arafune
 * Modified / Added some tests.
 *
 * Revision 1.2  2008/04/17 13:21:45  arafune
 * Modified some tests, and added new tests.
 *
 * Revision 1.1  2007/12/20 07:50:18  arafune
 * *** empty log message ***
 *
 * Revision 1.2  2007/01/24 16:04:18  n-ando
 * The SdoConfiguration's ctor. was changed.
 *
 * Revision 1.1  2006/11/27 08:26:07  n-ando
 * TestSuites are devided into each directory.
 *
 * Revision 1.3  2006/11/10 07:13:44  kurihara
 * A test after SdoConfiguration class revision.
 *
 * Revision 1.2  2006/11/09 09:29:47  kurihara
 * A test after SdoConfiguration class revision.
 *
 * Revision 1.1  2006/11/01 11:23:35  kurihara
 * test program for SdoConfiguration class.
 *
 */

#ifndef SdoConfiguration_cpp
#define SdoConfiguration_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <vector>
#include <string>

#include <rtm/CORBA_SeqUtil.h>
#include <rtm/SdoConfiguration.h>
#include <rtm/RTObject.h>
#include <rtm/Manager.h>
#include <rtm/SdoOrganization.h>

/*!
 * @class SdoConfigurationTests class
 * @brief SdoConfiguration test
 */
namespace SdoConfiguration
{
  using namespace SDOPackage;
  using namespace std;
	
  struct ServiceProfileFinder
  {
    ServiceProfileFinder(const std::string& id) : _id(id) { }
			
    bool operator()(const ServiceProfile& svcProf)
    {
      return (_id == std::string(svcProf.id));
    }
			
    std::string _id;
  };

  class SdoConfigurationTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(SdoConfigurationTests);

    CPPUNIT_TEST(test_set_device_profile_and_getDeviceProfile);
    CPPUNIT_TEST(test_set_service_profile_and_getServiceProfile);
    CPPUNIT_TEST(test_getServiceProfiles);
    CPPUNIT_TEST(test_remove_service_profile);
    CPPUNIT_TEST(test_add_organization_and_getOrganizations);
    CPPUNIT_TEST(test_remove_organization);
    CPPUNIT_TEST(test_add_configuration_set_and_get_configuration_set);
    CPPUNIT_TEST(test_remove_configuration_set);
    CPPUNIT_TEST(test_set_configuration_set_values);
    CPPUNIT_TEST(test_activate_configuration_set_and_get_active_configuration_set);
	//CPPUNIT_TEST(test_get_configuration_parameters);       //未実装のため未テスト
	//CPPUNIT_TEST(test_get_configuration_parameter_values); //未実装のため未テスト
	//CPPUNIT_TEST(test_get_configuration_parameter_value);  //未実装のため未テスト
	//CPPUNIT_TEST(test_set_configuration_parameter);        //未実装のため未テスト

    CPPUNIT_TEST_SUITE_END();
	
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
		
  public:
    /*!
     * @brief Constructor
     */
    SdoConfigurationTests()
    {
      int argc(0);
      char** argv(NULL);
      m_pORB = CORBA::ORB_init(argc, argv);
      m_pPOA = PortableServer::POA::_narrow(
		    m_pORB->resolve_initial_references("RootPOA"));
      m_pPOA->the_POAManager()->activate();
    }
    
    /*!
     * @brief Destructor
     */
    ~SdoConfigurationTests()
    {
    }
		
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
      usleep(100000);
    }
		
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    { 
    }
		
    /*!
     * set_device_profile()メソッドとgetDeviceProfile()メソッドのテスト
     * 
     * - set_device_profile()で設定したDeviceProfileを、getDeviceProfile()で正しく取得できるか？
     */
    void test_set_device_profile_and_getDeviceProfile()
    {
//      std::cout << "test_set_device_profile_and_getDeviceProfile() IN" << std::endl;
      coil::Properties cfgAdminProp;
      RTC::ConfigAdmin cfgAdmin(cfgAdminProp);
      SDOPackage::Configuration_impl* sdoCfg = new Configuration_impl(cfgAdmin);
			
      // DeviceProfileを準備する
      SDOPackage::DeviceProfile devProf;
      devProf.device_type = "DEVICE_TYPE";
      devProf.manufacturer = "MANUFACTURER";
      devProf.model = "MODEL";
      devProf.version = "VERSION";
      {
	SDOPackage::NVList properties;
	properties.length(2);
	properties[0].name = "name 0";
	properties[0].value <<= CORBA::Float(3.14159);
	properties[1].name = "name 1";
	properties[1].value <<= CORBA::Float(2.71828);
	devProf.properties = properties;
      }
			
      // set_device_profile()を呼出して、準備したDeviceProfileを設定する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->set_device_profile(devProf));
			
      // getDeviceProfile()で設定されているDeviceProfileを取り出し、設定したものと一致することを確認する
      const SDOPackage::DeviceProfile devProfRet = sdoCfg->getDeviceProfile();
      CPPUNIT_ASSERT_EQUAL(std::string("DEVICE_TYPE"),
			   std::string(devProfRet.device_type));
      CPPUNIT_ASSERT_EQUAL(std::string("MANUFACTURER"),
			   std::string(devProfRet.manufacturer));
      CPPUNIT_ASSERT_EQUAL(std::string("MODEL"),
			   std::string(devProfRet.model));
      CPPUNIT_ASSERT_EQUAL(std::string("VERSION"),
			   std::string(devProfRet.version));
      CPPUNIT_ASSERT_EQUAL(std::string("name 0"),
			   std::string(devProfRet.properties[0].name));
      {
	CORBA::Float value; devProfRet.properties[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(CORBA::Float(3.14159), value);
      }
      CPPUNIT_ASSERT_EQUAL(std::string("name 1"),
			   std::string(devProfRet.properties[1].name));
      {
	CORBA::Float value; devProfRet.properties[1].value >>= value;
	CPPUNIT_ASSERT_EQUAL(CORBA::Float(2.71828), value);
      }
      sdoCfg->_remove_ref();
//      std::cout << "test_set_device_profile_and_getDeviceProfile() OUT" << std::endl;
    }
		
    /*!
     * @brief set_service_profile()メソッドとgetServiceProfile()メソッドのテスト
     * 
     * - set_service_profile()で設定したServiceProfileを、getServiceProfile()で正しく取得できるか？
     */
    void test_set_service_profile_and_getServiceProfile()
    {
//      std::cout << "test_set_service_profile_and_getServiceProfile() IN" << std::endl;
      coil::Properties cfgAdminProp;
      RTC::ConfigAdmin cfgAdmin(cfgAdminProp);
      SDOPackage::Configuration_impl* sdoCfg = new Configuration_impl(cfgAdmin);
			
      // ServiceProfileを準備する
      SDOPackage::ServiceProfile svcProf;
      svcProf.id = "ID";
      svcProf.interface_type = "INTERFACE_TYPE";
      {
	SDOPackage::NVList properties;
	properties.length(2);
	properties[0].name = "name 0";
	properties[0].value <<= CORBA::Float(3.14159);
	properties[1].name = "name 1";
	properties[1].value <<= CORBA::Float(2.71828);
	svcProf.properties = properties;
      }
			
      // ServiceProfileを設定する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_service_profile(svcProf));
			
      // getServiceProfile()でServiceProfileを取得し、設定したものと一致しているか確認する
      const SDOPackage::ServiceProfile svcProfRet = sdoCfg->getServiceProfile("ID");
      CPPUNIT_ASSERT_EQUAL(std::string("ID"), std::string(svcProfRet.id));
      CPPUNIT_ASSERT_EQUAL(std::string("INTERFACE_TYPE"),
			   std::string(svcProfRet.interface_type));
      CPPUNIT_ASSERT_EQUAL(std::string("name 0"),
			   std::string(svcProfRet.properties[0].name));
      {
	CORBA::Float value; svcProfRet.properties[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(CORBA::Float(3.14159), value);
      }
      CPPUNIT_ASSERT_EQUAL(std::string("name 1"),
			   std::string(svcProfRet.properties[1].name));
      {
	CORBA::Float value; svcProfRet.properties[1].value >>= value;
	CPPUNIT_ASSERT_EQUAL(CORBA::Float(2.71828), value);
      }
      sdoCfg->_remove_ref();
//      std::cout << "test_set_service_profile_and_getServiceProfile() OUT" << std::endl;
    }
		
    /*!
     * @brief getServiceProfiles()メソッドのテスト
     * 
     * - 登録されている複数のServiceProfileを、getServiceProfiles()で正しく取得できるか？
     */
    void test_getServiceProfiles()
    {
//      std::cout << "test_getServiceProfiles() IN" << std::endl;
      coil::Properties cfgAdminProp;
      RTC::ConfigAdmin cfgAdmin(cfgAdminProp);
      SDOPackage::Configuration_impl* sdoCfg = new Configuration_impl(cfgAdmin);
			
      // ServiceProfileを準備する
      SDOPackage::ServiceProfile svcProf0;
      svcProf0.id = "ID 0";
      svcProf0.interface_type = "INTERFACE_TYPE 0";
      {
	SDOPackage::NVList properties;
	properties.length(2);
	properties[0].name = "name 0-0";
	properties[0].value <<= CORBA::Float(3.14159);
	properties[1].name = "name 0-1";
	properties[1].value <<= CORBA::Float(2.71828);
	svcProf0.properties = properties;
      }
    	
      SDOPackage::ServiceProfile svcProf1;
      svcProf1.id = "ID 1";
      svcProf1.interface_type = "INTERFACE_TYPE 1";
      {
	SDOPackage::NVList properties;
	properties.length(2);
	properties[0].name = "name 1-0";
	properties[0].value <<= CORBA::Float(1.41421356);
	properties[1].name = "name 1-1";
	properties[1].value <<= CORBA::Float(1.7320508);
	svcProf1.properties = properties;
      }
			
      // ServiceProfileを設定する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_service_profile(svcProf0));
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_service_profile(svcProf1));
			
      // getServiceProfiles()で設定されているServiceProfile群を取得する
      const SDOPackage::ServiceProfileList svcProfList = sdoCfg->getServiceProfiles();
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2), svcProfList.length());
			
      // 設定したServiceProfileと一致しているか？
      {
	CORBA::ULong idx = CORBA_SeqUtil::find(svcProfList, ServiceProfileFinder("ID 0"));
				
	CPPUNIT_ASSERT_EQUAL(std::string("ID 0"),
			     std::string(svcProfList[idx].id));
	CPPUNIT_ASSERT_EQUAL(std::string("INTERFACE_TYPE 0"),
			     std::string(svcProfList[idx].interface_type));
	CPPUNIT_ASSERT_EQUAL(std::string("name 0-0"),
			     std::string(svcProfList[idx].properties[0].name));
	{
	  CORBA::Float value; svcProfList[idx].properties[0].value >>= value;
	  CPPUNIT_ASSERT_EQUAL(CORBA::Float(3.14159), value);
	}
	CPPUNIT_ASSERT_EQUAL(std::string("name 0-1"),
			     std::string(svcProfList[idx].properties[1].name));
	{
	  CORBA::Float value; svcProfList[idx].properties[1].value >>= value;
	  CPPUNIT_ASSERT_EQUAL(CORBA::Float(2.71828), value);
	}
      }

      {
	CORBA::ULong idx = CORBA_SeqUtil::find(svcProfList, ServiceProfileFinder("ID 1"));
				
	CPPUNIT_ASSERT_EQUAL(std::string("ID 1"),
			     std::string(svcProfList[idx].id));
	CPPUNIT_ASSERT_EQUAL(std::string("INTERFACE_TYPE 1"),
			     std::string(svcProfList[idx].interface_type));
	CPPUNIT_ASSERT_EQUAL(std::string("name 1-0"),
			     std::string(svcProfList[idx].properties[0].name));
	{
	  CORBA::Float value; svcProfList[idx].properties[0].value >>= value;
	  CPPUNIT_ASSERT_EQUAL(CORBA::Float(1.41421356), value);
	}
	CPPUNIT_ASSERT_EQUAL(std::string("name 1-1"),
			     std::string(svcProfList[idx].properties[1].name));
	{
	  CORBA::Float value; svcProfList[idx].properties[1].value >>= value;
	  CPPUNIT_ASSERT_EQUAL(CORBA::Float(1.7320508), value);
	}
      }
      sdoCfg->_remove_ref();
//      std::cout << "test_getServiceProfiles() OUT" << std::endl;
    }
    
    /*!
     * @brief remove_service_profile()メソッドのテスト
     * 
     * - 指定したIDを持つServiceProfileを正しく登録解除できるか？
     */
    void test_remove_service_profile()
    {
//      std::cout << "test_remove_service_profile() IN" << std::endl;
      coil::Properties cfgAdminProp;
      RTC::ConfigAdmin cfgAdmin(cfgAdminProp);
      SDOPackage::Configuration_impl* sdoCfg = new Configuration_impl(cfgAdmin);
			
      // ServiceProfileを準備する
      SDOPackage::ServiceProfile svcProf0;
      svcProf0.id = "ID 0";
      svcProf0.interface_type = "INTERFACE_TYPE 0";
      {
	SDOPackage::NVList properties;
	properties.length(2);
	properties[0].name = "name 0-0";
	properties[0].value <<= CORBA::Float(3.14159);
	properties[1].name = "name 0-1";
	properties[1].value <<= CORBA::Float(2.71828);
	svcProf0.properties = properties;
      }
    	
      SDOPackage::ServiceProfile svcProf1;
      svcProf1.id = "ID 1";
      svcProf1.interface_type = "INTERFACE_TYPE 1";
      {
	SDOPackage::NVList properties;
	properties.length(2);
	properties[0].name = "name 1-0";
	properties[0].value <<= CORBA::Float(1.41421356);
	properties[1].name = "name 1-1";
	properties[1].value <<= CORBA::Float(1.7320508);
	svcProf1.properties = properties;
      }
			
      // ServiceProfileを設定する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_service_profile(svcProf0));
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_service_profile(svcProf1));
			
      // 設定したServiceProfileのうち、片方を登録解除する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->remove_service_profile("ID 0"));
			
      // getServiceProfiles()で全ServiceProfileを取得し、登録解除したものが含まれないことを確認する
      const SDOPackage::ServiceProfileList svcProfList = sdoCfg->getServiceProfiles();
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(1), svcProfList.length());
      CPPUNIT_ASSERT_EQUAL(CORBA::Long(-1),
			   CORBA_SeqUtil::find(svcProfList, ServiceProfileFinder("ID 0")));
			
      // 登録解除していないものは、依然として含まれているか？
      CPPUNIT_ASSERT_EQUAL(CORBA::Long(0),
			   CORBA_SeqUtil::find(svcProfList, ServiceProfileFinder("ID 1")));

      sdoCfg->_remove_ref();
//      std::cout << "test_remove_service_profile() OUT" << std::endl;
    }
    
    /* 
     * @brief add_organization()メソッドとgetOrganizations()メソッドのテスト
     * 
     * - add_organization()でOrganization_ptrインスタンスを登録できるか？
     * - getOrganizations()で登録されているOrganization_ptrインスタンス群を取得できるか？
     */
    void test_add_organization_and_getOrganizations()
    {
//      std::cout << "test_add_organization_and_getOrganizations() IN" << std::endl;
      coil::Properties cfgAdminProp;
      RTC::ConfigAdmin cfgAdmin(cfgAdminProp);
      SDOPackage::Configuration_impl* sdoCfg = new Configuration_impl(cfgAdmin);
			
      // Organizationを2つ登録する
      SDOPackage::Organization_var org1;
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true),
			   sdoCfg->add_organization(org1._retn()));

      SDOPackage::Organization_var org2;
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true),
			   sdoCfg->add_organization(org2._retn()));
      
      // 取得されるOrganizationの数は、意図どおり2つか？
      SDOPackage::OrganizationList orgList = sdoCfg->getOrganizations();
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2), orgList.length());

      sdoCfg->_remove_ref();
//      std::cout << "test_add_organization_and_getOrganizations() OUT" << std::endl;
    }
    
    /*
     * @brief remove_organization()のテスト
     * - add_organization()で登録し、remove_organization()で正しく削除できるか？
     */
    void test_remove_organization()
    {
//      std::cout << "test_remove_organization() IN" << std::endl;
      coil::Properties cfgAdminProp;
      RTC::ConfigAdmin cfgAdmin(cfgAdminProp);
      SDOPackage::Configuration_impl* sdoCfg = new Configuration_impl(cfgAdmin);
      RTC::RTObject_impl* rtobj;
      SDOPackage::Organization_impl* m_pOi;
      RTC::Manager& mgr(RTC::Manager::instance());
      rtobj = new ::RTC::RTObject_impl(&mgr);
      m_pOi = new Organization_impl(rtobj->getObjRef());
			
      // Organizationを登録する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true),
			   sdoCfg->add_organization(m_pOi->getObjRef()));

      // organization_idを取得する
      std::string id(m_pOi->get_organization_id());

      // 登録したOrganizationを削除する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true),
			   sdoCfg->remove_organization(id.c_str()));

      // 取得されるOrganizationの数は、意図どおり0件か？
      SDOPackage::OrganizationList orgList = sdoCfg->getOrganizations();
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(0), orgList.length());

      m_pOi->_remove_ref();
      rtobj->_remove_ref();
      sdoCfg->_remove_ref();
//      std::cout << "test_remove_organization() OUT" << std::endl;
    }
				
    /*
     * @brief get_configuration_parameters()のテスト
     */
    void test_get_configuration_parameters()
    {
      // テスト対象であるSDOPackage::Confirutaion_impl::get_configuration_parameters()が
      // 未実装であるため、本テストも未実装である。
//      std::cout << "test_get_configuration_parameters() IN" << std::endl;
//      std::cout << "test_get_configuration_parameters() OUT" << std::endl;
    }
		
    /*!
     * @brief get_configuration_parameter_values()のテスト
     */
    void test_get_configuration_parameter_values()
    {
      // テスト対象であるSDOPackage::Confirutaion_impl::get_configuration_parameter_values()が
      // 未実装であるため、本テストも未実装である。
//      std::cout << "test_get_configuration_parameter_values() IN" << std::endl;
//      std::cout << "test_get_configuration_parameter_values() OUT" << std::endl;
    }
		
    /*!
     * @brief get_configuration_parameter_value()のテスト
     */
    void test_get_configuration_parameter_value()
    {
      // テスト対象であるSDOPackage::Confirutaion_impl::get_configuration_parameter_value()が
      // 未実装であるため、本テストも未実装である。
//      std::cout << "test_get_configuration_parameter_value() IN" << std::endl;
//      std::cout << "test_get_configuration_parameter_value() OUT" << std::endl;
    }
		
    /*!
     * @brief set_configuration_parameter()のテスト
     */
    void test_set_configuration_parameter()
    {
      // テスト対象であるSDOPackage::Confirutaion_impl::set_configuration_parameter()が
      // 未実装であるため、本テストも未実装である。
//      std::cout << "test_set_configuration_parameter() IN" << std::endl;
//      std::cout << "test_set_configuration_parameter() OUT" << std::endl;
    }
		
    /*!
     * @brief add/get_configuration_set()メソッドのテスト
     * 
     * - ConfigurationSetをadd_configuration_set()で正常に登録できるか？
     * - add_configuration_set()で登録したConfigurationSetを、get_configuration_set()で正しく取得できるか？
     */
    void test_add_configuration_set_and_get_configuration_set()
    {
//      std::cout << "test_add_configuration_set_and_get_configuration_set() IN" << std::endl;
      coil::Properties cfgAdminProp;
      RTC::ConfigAdmin cfgAdmin(cfgAdminProp);
      SDOPackage::Configuration_impl* sdoCfg = new Configuration_impl(cfgAdmin);
			
      // ConfigurationSetを準備する
      SDOPackage::ConfigurationSet cfgSet0;
      cfgSet0.id = "ID 0";
      cfgSet0.description = "DESCRIPTION 0";
      cfgSet0.configuration_data.length(2);
      cfgSet0.configuration_data[0].name = "NAME 0-0";
      cfgSet0.configuration_data[0].value <<= "3.14159";
      cfgSet0.configuration_data[1].name = "NAME 0-1";
      cfgSet0.configuration_data[1].value <<= "2.71828";
			
      SDOPackage::ConfigurationSet cfgSet1;
      cfgSet1.id = "ID 1";
      cfgSet1.description = "DESCRIPTION 1";
      cfgSet1.configuration_data.length(2);
      cfgSet1.configuration_data[0].name = "NAME 1-0";
      cfgSet1.configuration_data[0].value <<= "1.41421356";
      cfgSet1.configuration_data[1].name = "NAME 1-1";
      cfgSet1.configuration_data[1].value <<= "1.7320508";
			
      // 準備したConfigurationSetを登録する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_configuration_set(cfgSet0));
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_configuration_set(cfgSet1));
			
      // 登録したConfigurationSetを正しく取得できるか？
      SDOPackage::ConfigurationSet* cfgSetRet0 = sdoCfg->get_configuration_set("ID 0");
      CPPUNIT_ASSERT_EQUAL(std::string("ID 0"),
			   std::string(cfgSetRet0->id));
//Deleted this test, because description was not used.
//      CPPUNIT_ASSERT_EQUAL(std::string("DESCRIPTION 0"),
//			   std::string(cfgSetRet0->description));
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2),
			   cfgSetRet0->configuration_data.length());
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 0-0"),
			   std::string(cfgSetRet0->configuration_data[0].name));
      {
	const char* value; cfgSetRet0->configuration_data[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("3.14159"), std::string(value));
      }
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 0-1"),
			   std::string(cfgSetRet0->configuration_data[1].name));
      {
	const char* value; cfgSetRet0->configuration_data[1].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("2.71828"), std::string(value));
      }

      SDOPackage::ConfigurationSet* cfgSetRet1 = sdoCfg->get_configuration_set("ID 1");
      CPPUNIT_ASSERT_EQUAL(std::string("ID 1"),
			   std::string(cfgSetRet1->id));
//Deleted this test, because description was not used.
//      CPPUNIT_ASSERT_EQUAL(std::string("DESCRIPTION 1"),
//			   std::string(cfgSetRet1->description));
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2),
			   cfgSetRet1->configuration_data.length());
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 1-0"),
			   std::string(cfgSetRet1->configuration_data[0].name));
      {
	const char* value; cfgSetRet1->configuration_data[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("1.41421356"), std::string(value));
      }
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 1-1"),
			   std::string(cfgSetRet1->configuration_data[1].name));
      {
	const char* value; cfgSetRet1->configuration_data[1].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("1.7320508"), std::string(value));
      }

      sdoCfg->_remove_ref();
//      std::cout << "test_add_configuration_set_and_get_configuration_set() OUT" << std::endl;
    }
		
    /*!
     * @brief remove_configuration_set()メソッドのテスト
     * 
     * - 登録済みのConfigurationSetを正しく登録解除できるか？
     * - 登録されていないIDを指定した場合、意図どおり例外がスローされるか？
     */
    void test_remove_configuration_set()
    {
//      std::cout << "test_remove_configuration_set() IN" << std::endl;
      coil::Properties cfgAdminProp;
      RTC::ConfigAdmin cfgAdmin(cfgAdminProp);
      SDOPackage::Configuration_impl* sdoCfg = new Configuration_impl(cfgAdmin);
			
      // ConfigurationSetを準備する
      SDOPackage::ConfigurationSet cfgSet0;
      cfgSet0.id = "ID 0";
      cfgSet0.description = "DESCRIPTION 0";
      cfgSet0.configuration_data.length(2);
      cfgSet0.configuration_data[0].name = "NAME 0-0";
      cfgSet0.configuration_data[0].value <<= "3.14159";
      cfgSet0.configuration_data[1].name = "NAME 0-1";
      cfgSet0.configuration_data[1].value <<= "2.71828";
			
      SDOPackage::ConfigurationSet cfgSet1;
      cfgSet1.id = "ID 1";
      cfgSet1.description = "DESCRIPTION 1";
      cfgSet1.configuration_data.length(2);
      cfgSet1.configuration_data[0].name = "NAME 1-0";
      cfgSet1.configuration_data[0].value <<= "1.41421356";
      cfgSet1.configuration_data[1].name = "NAME 1-1";
      cfgSet1.configuration_data[1].value <<= "1.7320508";
			
      // 準備したConfigurationSetを登録する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_configuration_set(cfgSet0));
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_configuration_set(cfgSet1));
			
      // 登録したうち、片方のConfigurationSetを登録解除する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->remove_configuration_set("ID 0"));
			
      // 登録解除したConfigurationSetを指定して、get_configuration_set()呼出を試みて、
      // 意図どおりに例外がスローされるか？
      try
	{
	  sdoCfg->get_configuration_set("ID 0");
	  CPPUNIT_FAIL("ID 0 was not removed.");
	}
      catch (SDOPackage::InvalidParameter expected) {}
      catch (...) {}
			
      // 空のidを指定して、get_configuration_set()呼出を試みて、
      // 意図どおりに例外がスローされるか？
      try
	{
	  sdoCfg->get_configuration_set("");
	  CPPUNIT_FAIL("ID Not set.");
	}
      catch (SDOPackage::InvalidParameter expected) {}
      catch (...) {}
			
      // 登録解除していないConfigurationSetは、依然として取得できるか？
      SDOPackage::ConfigurationSet* cfgSetRet = sdoCfg->get_configuration_set("ID 1");
      CPPUNIT_ASSERT_EQUAL(std::string("ID 1"), std::string(cfgSetRet->id));
			
      // 存在しないIDを指定して登録解除を試みた場合に、意図どおりに例外がスローされるか？
      try
	{
	  sdoCfg->remove_configuration_set("inexist ID");
	  CPPUNIT_FAIL("Exception not thrown.");
	}
      catch (SDOPackage::InvalidParameter expected) {}
      catch (...) {}

      sdoCfg->_remove_ref();
//      std::cout << "test_remove_configuration_set() OUT" << std::endl;
    }
		
    /*!
     * @brief set_configuration_set_values()メソッドのテスト
     * 
     * - 登録済みのConfigurationSetのIDを指定して、正しくConfigurationSetを更新できるか？
     * - 存在しないIDを指定した場合に、意図どおりに例外がスローされるか？
     */
    void test_set_configuration_set_values()
    {
//      std::cout << "test_set_configuration_set_values() IN" << std::endl;
      coil::Properties cfgAdminProp;
      RTC::ConfigAdmin cfgAdmin(cfgAdminProp);
      SDOPackage::Configuration_impl* sdoCfg = new Configuration_impl(cfgAdmin);
			
      // ConfigurationSetを準備する
      SDOPackage::ConfigurationSet cfgSet0;
      cfgSet0.id = "ID 0";
      cfgSet0.description = "DESCRIPTION 0";
      cfgSet0.configuration_data.length(2);
      cfgSet0.configuration_data[0].name = "NAME 0-0";
      cfgSet0.configuration_data[0].value <<= "3.14159";
      cfgSet0.configuration_data[1].name = "NAME 0-1";
      cfgSet0.configuration_data[1].value <<= "2.71828";
			
      SDOPackage::ConfigurationSet cfgSet1;
      cfgSet1.id = "ID 1";
      cfgSet1.description = "DESCRIPTION 1";
      cfgSet1.configuration_data.length(2);
      cfgSet1.configuration_data[0].name = "NAME 1-0";
      cfgSet1.configuration_data[0].value <<= "1.41421356";
      cfgSet1.configuration_data[1].name = "NAME 1-1";
      cfgSet1.configuration_data[1].value <<= "1.7320508";

      SDOPackage::ConfigurationSet cfgSet1_Modified;
      cfgSet1_Modified.id = "ID 1";
      cfgSet1_Modified.description = "DESCRIPTION 1 M";
      cfgSet1_Modified.configuration_data.length(2);
      cfgSet1_Modified.configuration_data[0].name = "NAME 1-0";
      cfgSet1_Modified.configuration_data[0].value <<= "2.23620679";
      cfgSet1_Modified.configuration_data[1].name = "NAME 1-1";
      cfgSet1_Modified.configuration_data[1].value <<= "2.44948974";

      // 準備したConfigurationSetを登録する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_configuration_set(cfgSet0));
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_configuration_set(cfgSet1));
			
      // 登録したConfigurationSetのうち片方を、set_configuration_set_values()で更新する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true),
			   sdoCfg->set_configuration_set_values(cfgSet1_Modified));
			
      // 更新したConfigurationSetを、正しく取得できるか？
      SDOPackage::ConfigurationSet* cfgSetRet = sdoCfg->get_configuration_set("ID 1");
      CPPUNIT_ASSERT_EQUAL(std::string("ID 1"), std::string(cfgSetRet->id));

//Deleted this test, because description was not used.
//      CPPUNIT_ASSERT_EQUAL(std::string("DESCRIPTION 1 M"),
//			   std::string(cfgSetRet->description));
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2),
			   cfgSetRet->configuration_data.length());

      {
	const char* value; cfgSetRet->configuration_data[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("2.23620679"), std::string(value));
      }
      {
	const char* value; cfgSetRet->configuration_data[1].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("2.44948974"), std::string(value));
      }
			
      // 空のIDを指定してset_configuration_set_values()を呼出し、
      // 意図どおり例外がスローされるか？
      try
	{
	  cfgSet1_Modified.id = "";
	  sdoCfg->set_configuration_set_values(cfgSet1_Modified);
	  CPPUNIT_FAIL("Exception not thrown.");
	}
      catch (SDOPackage::InvalidParameter expected) {
	}

      sdoCfg->_remove_ref();
//      std::cout << "test_set_configuration_set_values() OUT" << std::endl;
    }
		
    /*!
     * @brief activate_configuration_set()メソッド、get_active_configuration_set()メソッド、
     * get_configuration_sets()メソッドのテスト
     * 
     */
    void test_activate_configuration_set_and_get_active_configuration_set()
    {
//      std::cout << "test_activate_configuration_set_and_get_active_configuration_set() IN" << std::endl;
      coil::Properties cfgAdminProp;
      RTC::ConfigAdmin cfgAdmin(cfgAdminProp);
      SDOPackage::Configuration_impl* sdoCfg = new Configuration_impl(cfgAdmin);
	
      // ConfigurationSetを準備する
      SDOPackage::ConfigurationSet cfgSet0;
      cfgSet0.id = "ID 0";
      cfgSet0.description = "DESCRIPTION 0";
      cfgSet0.configuration_data.length(2);
      cfgSet0.configuration_data[0].name = "NAME 0-0";
      cfgSet0.configuration_data[0].value <<= "3.14159";
      cfgSet0.configuration_data[1].name = "NAME 0-1";
      cfgSet0.configuration_data[1].value <<= "2.71828";
			
      SDOPackage::ConfigurationSet cfgSet1;
      cfgSet1.id = "ID 1";
      cfgSet1.description = "DESCRIPTION 1";
      cfgSet1.configuration_data.length(2);
      cfgSet1.configuration_data[0].name = "NAME 1-0";
      cfgSet1.configuration_data[0].value <<= "1.41421356";
      cfgSet1.configuration_data[1].name = "NAME 1-1";
      cfgSet1.configuration_data[1].value <<= "1.7320508";

      // 準備したConfigurationSetを登録する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_configuration_set(cfgSet0));
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->add_configuration_set(cfgSet1));
			
      // ConfigurationSet リストを取得する
      ConfigurationSetList_var config_sets(sdoCfg->get_configuration_sets());

      // 取得した件数と内容が一致しているか？
      CPPUNIT_ASSERT(config_sets->length() == 2);
      CPPUNIT_ASSERT_EQUAL(std::string("ID 0"),
			   std::string(config_sets[0].id));
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 0-0"),
			   std::string(config_sets[0].configuration_data[0].name));
      {
	const char* value; config_sets[0].configuration_data[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("3.14159"), std::string(value));
      }
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 0-1"),
			   std::string(config_sets[0].configuration_data[1].name));
      {
	const char* value; config_sets[0].configuration_data[1].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("2.71828"), std::string(value));
      }

      CPPUNIT_ASSERT_EQUAL(std::string("ID 1"),
			   std::string(config_sets[1].id));
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 1-0"),
			   std::string(config_sets[1].configuration_data[0].name));
      {
	const char* value; config_sets[1].configuration_data[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("1.41421356"), std::string(value));
      }
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 1-1"),
			   std::string(config_sets[1].configuration_data[1].name));
      {
	const char* value; config_sets[1].configuration_data[1].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("1.7320508"), std::string(value));
      }

      // "ID 0"のほうをアクティブ化する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->activate_configuration_set("ID 0"));
			
      // アクティブなConfigurationSetを取得し、それがアクティブ化したものと一致するか？
      SDOPackage::ConfigurationSet* cfgSetRet0 = sdoCfg->get_active_configuration_set();
      CPPUNIT_ASSERT_EQUAL(std::string("ID 0"), std::string(cfgSetRet0->id));

//Deleted this test, because description was not used.
//      CPPUNIT_ASSERT_EQUAL(std::string("DESCRIPTION 0"),
//			   std::string(cfgSetRet0->description));
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2),
			   cfgSetRet0->configuration_data.length());
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 0-0"),
			   std::string(cfgSetRet0->configuration_data[0].name));
      {
	const char* value; cfgSetRet0->configuration_data[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("3.14159"), std::string(value));
      }
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 0-1"),
			   std::string(cfgSetRet0->configuration_data[1].name));
      {
	const char* value; cfgSetRet0->configuration_data[1].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("2.71828"), std::string(value));
      }

      // "ID 1"のほうをアクティブ化する
      CPPUNIT_ASSERT_EQUAL(CORBA::Boolean(true), sdoCfg->activate_configuration_set("ID 1"));
			
      // アクティブなConfigurationSetを取得し、それがアクティブ化したものと一致するか？
      SDOPackage::ConfigurationSet* cfgSetRet1 = sdoCfg->get_active_configuration_set();
      CPPUNIT_ASSERT_EQUAL(std::string("ID 1"), std::string(cfgSetRet1->id));

//Deleted this test, because description was not used.
//      CPPUNIT_ASSERT_EQUAL(std::string("DESCRIPTION 1"),
//			   std::string(cfgSetRet1->description));
      CPPUNIT_ASSERT_EQUAL(CORBA::ULong(2),
			   cfgSetRet1->configuration_data.length());
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 1-0"),
			   std::string(cfgSetRet1->configuration_data[0].name));
      {
	const char* value; cfgSetRet1->configuration_data[0].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("1.41421356"), std::string(value));
      }
      CPPUNIT_ASSERT_EQUAL(std::string("NAME 1-1"),
			   std::string(cfgSetRet1->configuration_data[1].name));
      {
	const char* value; cfgSetRet1->configuration_data[1].value >>= value;
	CPPUNIT_ASSERT_EQUAL(std::string("1.7320508"), std::string(value));
      }
			
      // 存在しないIDを指定してactivate_configuration_set()を呼出し、意図どおりの例外がスローされるか？
      try
	{
	  sdoCfg->activate_configuration_set("inexist ID");
	  CPPUNIT_FAIL("Exception not thrown.");
	}
      catch (SDOPackage::InvalidParameter expected) {}

      sdoCfg->_remove_ref();
//      std::cout << "test_activate_configuration_set_and_get_active_configuration_set() OUT" << std::endl;
    }

  };
}; // namespace SdoConfiguration

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(SdoConfiguration::SdoConfigurationTests);

#ifdef LOCAL_MAIN
int main(int argc, char* argv[])
{

  FORMAT format = TEXT_OUT;
  int target = 0;
  std::string xsl;
  std::string ns;
  std::string fname;
  std::ofstream ofs;

  int i(1);
  while (i < argc)
    {
      std::string arg(argv[i]);
      std::string next_arg;
      if (i + 1 < argc) next_arg = argv[i + 1];
      else              next_arg = "";

      if (arg == "--text") { format = TEXT_OUT; break; }
      if (arg == "--xml")
	{
	  if (next_arg == "")
	    {
	      fname = argv[0];
	      fname += ".xml";
	    }
	  else
	    {
	      fname = next_arg;
	    }
	  format = XML_OUT;
	  ofs.open(fname.c_str());
	}
      if ( arg == "--compiler"  ) { format = COMPILER_OUT; break; }
      if ( arg == "--cerr"      ) { target = 1; break; }
      if ( arg == "--xsl"       )
	{
	  if (next_arg == "") xsl = "default.xsl"; 
	  else                xsl = next_arg;
	}
      if ( arg == "--namespace" )
	{
	  if (next_arg == "")
	    {
	      std::cerr << "no namespace specified" << std::endl;
	      exit(1); 
	    }
	  else
	    {
	      xsl = next_arg;
	    }
	}
      ++i;
    }
  CppUnit::TextUi::TestRunner runner;
  if ( ns.empty() )
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry().makeTest());
  else
    runner.addTest(CppUnit::TestFactoryRegistry::getRegistry(ns).makeTest());
  CppUnit::Outputter* outputter = 0;
  std::ostream* stream = target ? &std::cerr : &std::cout;
  switch ( format )
    {
    case TEXT_OUT :
      outputter = new CppUnit::TextOutputter(&runner.result(),*stream);
      break;
    case XML_OUT :
      std::cout << "XML_OUT" << std::endl;
      outputter = new CppUnit::XmlOutputter(&runner.result(),
					    ofs, "shift_jis");
      static_cast<CppUnit::XmlOutputter*>(outputter)->setStyleSheet(xsl);
      break;
    case COMPILER_OUT :
      outputter = new CppUnit::CompilerOutputter(&runner.result(),*stream);
      break;
    }
  runner.setOutputter(outputter);
  runner.run();
  return 0; // runner.run() ? 0 : 1;
}
#endif // MAIN
#endif // SdoConfiguration_cpp
