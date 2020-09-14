// -*- C++ -*-
/*!
 * @file   ConnectorListenerTests.cpp
 * @brief  ConnectorListener test class
 * @date   $Date: 2010/02/09 04:32:00 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id: ConnectorListenerTests.cpp 1758 2010-01-22 14:04:54Z hakuta $
 *
 */

/*
 * $Log: ConnectorListenerTests.cpp,v $
 */

#ifndef ConnectorListener_cpp
#define ConnectorListener_cpp

#include <iostream>

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/DataPortSkel.h>
#include <rtm/RTC.h>
#include <rtm/CdrBufferBase.h>
#include <coil/Properties.h>
#include <rtm/ConnectorListener.h>

// ConnectorDataListenerType count
#define cdl_len 10
// ConnectorListenerType count
#define cl_len 7

/*!
 * @class ConnectorListenerTests class
 * @brief ConnectorListener test
 */

namespace ConnectorListener
{
  // ConnectorDataListenerType
  static const char* str_cdl[] =
  {
    "ON_BUFFER_WRITE",
    "ON_BUFFER_FULL",
    "ON_BUFFER_WRITE_TIMEOUT",
    "ON_BUFFER_OVERWRITE",
    "ON_BUFFER_READ", 
    "ON_SEND", 
    "ON_RECEIVED",
    "ON_RECEIVER_FULL", 
    "ON_RECEIVER_TIMEOUT", 
    "ON_RECEIVER_ERROR"
  };

  // ConnectorListenerType
  static const char* str_cl[] =
  {
    "ON_BUFFER_EMPTY",
    "ON_BUFFER_READ_TIMEOUT",
    "ON_SENDER_EMPTY", 
    "ON_SENDER_TIMEOUT", 
    "ON_SENDER_ERROR", 
    "ON_CONNECT",
    "ON_DISCONNECT"
  };

  static int cdl_count;
  static int cl_count;

  class DataListener
    : public RTC::ConnectorDataListenerT<RTC::TimedLong>
  {
  public:
    DataListener(const char* name) : m_name(name)
    {
      ++cdl_count;
    }
    virtual ~DataListener()
    {
      --cdl_count;
    }

    virtual void operator()(const RTC::ConnectorInfo& info,
                            const RTC::TimedLong& data)
    {
      m_data = data;
      std::cout << "-------------------------------------------" << std::endl;
      std::cout << "Data Listener: " << m_name << std::endl;
//      std::cout << "    Data: " << data.data << std::endl;
      std::cout << "-------------------------------------------" << std::endl;
    }

    RTC::TimedLong get_data()
    {
       RTC::TimedLong ret;
       ret = m_data;
       m_data.data = 0;
       return ret;
    };

    std::string m_name;
    RTC::TimedLong m_data;
  };

  class ConnListener
    : public RTC::ConnectorListener
  {
  public:
    ConnListener(const char* name) : m_name(name)
    {
      ++cl_count;
    }
    virtual ~ConnListener()
    {
      --cl_count;
    }

    virtual void operator()(const RTC::ConnectorInfo& info)
    {
      std::cout << "-------------------------------------------" << std::endl;
      std::cout << "Connector Listener: " << m_name << std::endl;
      std::cout << "-------------------------------------------" << std::endl;
    };

    std::string m_name;
  };


  class ConnectorListenerTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(ConnectorListenerTests);

    CPPUNIT_TEST(test_ConnectorDataListener);
    CPPUNIT_TEST(test_ConnectorListener);

    CPPUNIT_TEST_SUITE_END();
  
  private:
    RTC::ConnectorListeners listeners;
    DataListener *datalisteners[cdl_len];
    ConnListener *connlisteners[cl_len];

  public:
  
    /*!
     * @brief Constructor
     */
    ConnectorListenerTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~ConnectorListenerTests()
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
     * @brief ConnectorDataListener test
     */
    void test_ConnectorDataListener()
    {
      //ConnectorDataListeners
      for (int i(0); i<cdl_len; ++i)
        {
          datalisteners[i] = new DataListener(str_cdl[i]);
        }

      //ConnectorInfo
      coil::Properties prop;
      coil::vstring ports;
      RTC::ConnectorInfo info("name", "id", ports, prop);
      RTC::ConnectorInfo info_big("name", "id", ports, prop);
      info_big.properties.setProperty("serializer.cdr.endian", "big");

      // addListener()
      for (int i(0); i<cdl_len; ++i)
        {
          listeners.connectorData_[i].addListener(datalisteners[i], true);
        }

      // notify() endian: little
      for (int i(0); i<cdl_len; ++i)
        {
          RTC::TimedLong rtd;
          cdrMemoryStream cdr;
          RTC::TimedLong td;
          td.data = i;
          cdr.setByteSwapFlag(true);
          td >>= cdr;
          listeners.connectorData_[i].notify(info, cdr);
          rtd = datalisteners[i]->get_data();
          CPPUNIT_ASSERT_EQUAL(i, (int)rtd.data);
        }
      
      // notify() endian: big
      for (int i(0); i<cdl_len; ++i)
        {
          RTC::TimedLong rtd;
          cdrMemoryStream cdr;
          RTC::TimedLong td;
          td.data = i;
          cdr.setByteSwapFlag(false);
          td >>= cdr;
          listeners.connectorData_[i].notify(info_big, cdr);
          rtd = datalisteners[i]->get_data();
          CPPUNIT_ASSERT_EQUAL(i, (int)rtd.data);
        }

      // removeListener()
      for (int i(0); i<cdl_len; ++i)
        {
          listeners.connectorData_[i].removeListener(datalisteners[i]);
        }

      // notify() endian: little
      for (int i(0); i<cdl_len; ++i)
        {
          RTC::TimedLong rtd;
          cdrMemoryStream cdr;
          RTC::TimedLong td;
          td.data = i;
          cdr.setByteSwapFlag(true);
          td >>= cdr;
          listeners.connectorData_[i].notify(info, cdr);
          rtd = datalisteners[i]->get_data();
          CPPUNIT_ASSERT_EQUAL(0, (int)rtd.data);
        }

        CPPUNIT_ASSERT_EQUAL(0, cdl_count);

        // デストラクタでdelete しているので不要。
        // delete datalisteners;
    }

    /*!
     * @brief ConnectorListener test
     */
    void test_ConnectorListener()
    {
      //ConnectorListeners
      for (int i(0); i<cl_len; ++i)
        {
          connlisteners[i] = new ConnListener(str_cl[i]);
        }

      //ConnectorInfo
      coil::Properties prop;
      coil::vstring ports;
      RTC::ConnectorInfo info("name", "id", ports, prop);

      // addListener()
      for (int i(0); i<cl_len; ++i)
        {
          listeners.connector_[i].addListener(connlisteners[i], true);
        }

      // notify()
      for (int i(0); i<cl_len; ++i)
        {
          listeners.connector_[i].notify(info);
        }
      
      // removeListener()
      for (int i(0); i<cl_len; ++i)
        {
          listeners.connector_[i].removeListener(connlisteners[i]);
        }

      // notify() endian: little
      for (int i(0); i<cl_len; ++i)
        {
          listeners.connector_[i].notify(info);
        }
        CPPUNIT_ASSERT_EQUAL(0, cl_count);

        // デストラクタでdelete しているので不要。
        // delete datalisteners;
    }

  };
}; // namespace ConnectorListener

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(ConnectorListener::ConnectorListenerTests);

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
#endif // ConnectorListener_cpp
