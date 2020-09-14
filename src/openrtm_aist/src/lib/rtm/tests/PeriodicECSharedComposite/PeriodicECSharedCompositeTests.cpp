// -*- C++ -*-
/*!
 * @file   PeriodicECSharedCompositeTests.cpp
 * @brief  PeriodicECSharedComposite test class
 * @date   $Date: 2010/02/10 13:19:07 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id: PeriodicECSharedCompositeTests.cpp 1466 2009-07-17 09:18:14Z hakuta $
 *
 */

/*
 * $Log: PeriodicECSharedCompositeTests.cpp,v $
 *
 */

#ifndef PeriodicECSharedComposite_cpp
#define PeriodicECSharedComposite_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/idl/RTCSkel.h>
#include <rtm/idl/OpenRTMSkel.h>
#include <rtm/RTC.h>
#include <rtm/RTObject.h>
#include <rtm/PeriodicExecutionContext.h>
#include <rtm/PeriodicECSharedComposite.h>
#include <rtm/Manager.h>
#include <rtm/SdoOrganization.h>

/*!
 * @class PeriodicECSharedCompositeTests class
 * @brief PeriodicECSharedComposite test
 */
namespace PeriodicECSharedComposite
{
  using namespace SDOPackage;

  class PeriodicECSharedCompositeTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(PeriodicECSharedCompositeTests);

    CPPUNIT_TEST(test_PeriodicECOrganization);
    CPPUNIT_TEST(test_PeriodicECSharedComposite);

    CPPUNIT_TEST_SUITE_END();
	
  private:
    CORBA::ORB_ptr m_pORB;
    PortableServer::POA_ptr m_pPOA;
	
  public:

    /*!
     * @brief Constructor
     */
    PeriodicECSharedCompositeTests()
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
    virtual ~PeriodicECSharedCompositeTests()
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
     * @brief add_members(), set_members(), remove_member(), removeAllMembers(), updateDelegatedPorts() メソッドのテスト
     * 
     */
    void test_PeriodicECOrganization()
    {
      CORBA::Boolean bret;
      SDOPackage::SDO_var sdo;
      SDOPackage::SDOList slist;

      RTC::Manager& mgr(RTC::Manager::instance());
      RTC::RTObject_impl* rto = new RTC::RTObject_impl(&mgr);
      SDOPackage::PeriodicECOrganization* peco = new SDOPackage::PeriodicECOrganization(rto);

      short sflg = -1;
      try {
        // set_members()
        sdo = SDOPackage::SDO::_duplicate(rto->getObjRef());
        rto->setInstanceName("obj1");
        CORBA_SeqUtil::push_back(slist, sdo);
        bret = peco->set_members(slist);
        CPPUNIT_ASSERT(bret);
        sflg = 0;
      }
      catch (InvalidParameter ip) {
        //std::cout << "InvalidParameter exception." << std::endl;
        sflg = 1;
      }
      catch (NotAvailable na) {
        //std::cout << "NotAvailable exception." << std::endl;
        sflg = 2;
      }
      catch (InternalError ie) {
        //std::cout << "InternalError exception." << std::endl;
        sflg = 3;
      }
      catch (...) {
        //std::cout << "othrer exception." << std::endl;
        sflg = 9;
      }
      CPPUNIT_ASSERT(sflg == 0);

      sflg = -1;
      try {
        // remove_member()
        bret = peco->remove_member("obj1");
        CPPUNIT_ASSERT(bret);
        sflg = 0;
      }
      catch (InvalidParameter ip) {
        //std::cout << "InvalidParameter exception." << std::endl;
        sflg = 1;
      }
      catch (NotAvailable na) {
        //std::cout << "NotAvailable exception." << std::endl;
        sflg = 2;
      }
      catch (InternalError ie) {
        //std::cout << "InternalError exception." << std::endl;
        sflg = 3;
      }
      catch (...) {
        //std::cout << "othrer exception." << std::endl;
        sflg = 9;
      }
      CPPUNIT_ASSERT(sflg == 0);

      // updateDelegatedPorts()
      peco->updateDelegatedPorts();

      // 削除後の例外チェック
      sflg = -1;
      try {
        // remove_member()
        bret = peco->remove_member("obj1");
        sflg = 0;
      }
      catch (InvalidParameter ip) {
        //std::cout << "InvalidParameter exception." << std::endl;
        sflg = 1;
      }
      catch (NotAvailable na) {
        //std::cout << "NotAvailable exception." << std::endl;
        sflg = 2;
      }
      catch (InternalError ie) {
        //std::cout << "InternalError exception." << std::endl;
        sflg = 3;
      }
      catch (...) {
        //std::cout << "othrer exception." << std::endl;
        sflg = 9;
      }
      CPPUNIT_ASSERT(sflg == 1);

      sflg = -1;
      try {
        // add_members()
        sdo = SDOPackage::SDO::_duplicate(rto->getObjRef());
        rto->setInstanceName("obj2");
        CORBA_SeqUtil::push_back(slist, sdo);
        bret = peco->add_members(slist);
        CPPUNIT_ASSERT(bret);
        sflg = 0;
      }
      catch (InvalidParameter ip) {
        //std::cout << "InvalidParameter exception." << std::endl;
        sflg = 1;
      }
      catch (NotAvailable na) {
        //std::cout << "NotAvailable exception." << std::endl;
        sflg = 2;
      }
      catch (InternalError ie) {
        //std::cout << "InternalError exception." << std::endl;
        sflg = 3;
      }
      catch (...) {
        //std::cout << "othrer exception." << std::endl;
        sflg = 9;
      }
      CPPUNIT_ASSERT(sflg == 0);

      sflg = -1;
      try {
        // removeAllMembers()
        peco->removeAllMembers();
        sflg = 0;
      }
      catch (InvalidParameter ip) {
        //std::cout << "InvalidParameter exception." << std::endl;
        sflg = 1;
      }
      catch (NotAvailable na) {
        //std::cout << "NotAvailable exception." << std::endl;
        sflg = 2;
      }
      catch (InternalError ie) {
        //std::cout << "InternalError exception." << std::endl;
        sflg = 3;
      }
      catch (...) {
        //std::cout << "othrer exception." << std::endl;
        sflg = 9;
      }
      CPPUNIT_ASSERT(sflg == 0);

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(peco));
      delete peco;
      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(rto));
      delete rto;
    }

    /*!
     * @brief onInitialize(), onActivated(), onDeactivated(), onReset(), onFinalize() メソッドのテスト
     * 
     */
    void test_PeriodicECSharedComposite()
    {
      RTC::ReturnCode_t ret;
      RTC::Manager& mgr(RTC::Manager::instance());
      RTC::PeriodicECSharedComposite* pec = new RTC::PeriodicECSharedComposite(&mgr);
      RTC::UniqueId id = 0;

      // onInitialize()
      ret = pec->onInitialize();
      CPPUNIT_ASSERT(ret == RTC::RTC_OK);
      coil::usleep(100000);

      // onActivated()
      ret = pec->onActivated(id);
      CPPUNIT_ASSERT(ret == RTC::RTC_OK);
      coil::usleep(100000);

      // onDeactivated()
      ret = pec->onDeactivated(id);
      CPPUNIT_ASSERT(ret == RTC::RTC_OK);
      coil::usleep(100000);

      // onReset()
      ret = pec->onReset(id);
      CPPUNIT_ASSERT(ret == RTC::RTC_OK);
      coil::usleep(100000);

      // onFinalize()
      ret = pec->onFinalize();
      CPPUNIT_ASSERT(ret == RTC::RTC_OK);
      coil::usleep(10000);

      m_pPOA->deactivate_object(*m_pPOA->servant_to_id(pec));
      delete pec;
    }

  };

}; // namespace PeriodicECSharedComposite

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(PeriodicECSharedComposite::PeriodicECSharedCompositeTests);

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
#endif // PeriodicECSharedComposite_cpp
