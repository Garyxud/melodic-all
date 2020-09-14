// -*- C++ -*-
/*!
 * @file   PublisherPeriodicTests.cpp
 * @brief  PublisherPeriodic test class
 * @date   $Date: 2008/01/28 13:52:19 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$
 *
 */

/*
 * $Log: PublisherPeriodicTests.cpp,v $
 * Revision 1.2  2008/01/28 13:52:19  arafune
 * Some tests were added.
 *
 * Revision 1.1  2007/12/20 07:50:17  arafune
 * *** empty log message ***
 *
 * Revision 1.2  2007/01/12 14:54:45  n-ando
 * The constructor's signature was changed.
 * InPortConsumer base class is now abstruct class. It needs concrete class.
 *
 * Revision 1.1  2006/12/18 06:51:55  n-ando
 * The first commitment.
 *
 *
 */

#ifndef PublisherPeriodic_cpp
#define PublisherPeriodic_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <iostream>
#include <coil/Properties.h>
#include <coil/Time.h>
#include <rtm/InPortConsumer.h>
#include <rtm/InPortCorbaCdrConsumer.h>
#include <rtm/PublisherPeriodic.h>
#include <rtm/CdrRingBuffer.h>
#include <rtm/idl/ManagerSkel.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/ConnectorListener.h>

/*!
 * @class PublisherPeriodicTests class
 * @brief PublisherPeriodic test
 */
namespace PublisherPeriodic
{
  int m_OnCheck = 0;

  /*!
   * 
   */
  class DataListener
    : public RTC::ConnectorDataListenerT<RTC::TimedLong>
  {
  public:
    DataListener(const char* name) : m_name(name) {}
    virtual ~DataListener()
    {
    }

    virtual void operator()(const RTC::ConnectorInfo& info,
                            const RTC::TimedLong& data)
    {
      std::cout << "------------------------------" << std::endl;
      std::cout << "Listener: " << m_name << std::endl;
      std::cout << "    Data: " << data.data << std::endl;
      std::cout << "------------------------------" << std::endl;
    };
    std::string m_name;
  };

  /*!
   * 
   * 
   */
  class PublisherPeriodicMock
    : public RTC::PublisherPeriodic
  {
  public:
    PublisherPeriodicMock(void)
    {
        ;
    }
    virtual ~PublisherPeriodicMock(void)
    {
        ;
    }
  };
  /*!
   * 
   * 
   */
  class InPortCorbaCdrConsumerMock
    : public RTC::InPortCorbaCdrConsumer
  {
  public:
      /*!
       * 
       * 
       */
      InPortCorbaCdrConsumerMock(void)
      {
          m_buffer = new RTC::CdrRingBuffer();
          m_test_mode = 0;
      }
      /*!
       * 
       * 
       */
      virtual ~InPortCorbaCdrConsumerMock()
      {
          delete m_buffer;
      }
      /*!
       * 
       * 
       */
      virtual ReturnCode put(const cdrMemoryStream& data)
      {
          if(m_test_mode == 0)
          {
              if (m_buffer->full())
              {
                   return RTC::PublisherPeriodic::SEND_FULL;
              }

              RTC::BufferStatus::Enum ret = m_buffer->write(data);

              //Listener check
              if(m_OnCheck == 0) {
                  switch(ret)
                  {
                       case RTC::BufferStatus::BUFFER_OK:
                           return RTC::PublisherPeriodic::PORT_OK;
                           break;
                       case RTC::BufferStatus::BUFFER_ERROR:
                           return RTC::PublisherPeriodic::PORT_ERROR;
                           break;
                       case RTC::BufferStatus::BUFFER_FULL:
                           return RTC::PublisherPeriodic::SEND_FULL;
                           break;
                       case RTC::BufferStatus::BUFFER_EMPTY:
                           return RTC::PublisherPeriodic::BUFFER_EMPTY;
                           break;
                       case RTC::BufferStatus::TIMEOUT:
                           return RTC::PublisherPeriodic::SEND_TIMEOUT;
                           break;
                       default:
                           return RTC::PublisherPeriodic::UNKNOWN_ERROR;
                   }
                   return RTC::PublisherPeriodic::UNKNOWN_ERROR;
                  }
              else if(m_OnCheck == 1) {
                   return RTC::PublisherPeriodic::PORT_OK;
              }
              else if(m_OnCheck == 2) {
                   return RTC::PublisherPeriodic::PORT_ERROR;
              }
              else if(m_OnCheck == 3) {
                   return RTC::PublisherPeriodic::SEND_FULL;
              }
              else if(m_OnCheck == 4) {
                   return RTC::PublisherPeriodic::SEND_TIMEOUT;
              }
              else if(m_OnCheck == 5) {
                   return RTC::PublisherPeriodic::UNKNOWN_ERROR;
              }
              else if(m_OnCheck == 6) {
                   return RTC::PublisherPeriodic::CONNECTION_LOST;
              }
          }
          else if(m_test_mode == 1)
          {
               std::string str("test");
               throw str;
          }
          else
          {
          }
      }
      /*!
       * 
       * 
       */
      cdrMemoryStream get_m_put_data(void)
      {
          cdrMemoryStream cdr;
          m_buffer->read(cdr);

          return cdr;
      }
      /*!
       * 
       * 
       */
      int get_m_put_data_len(void)
      {
          int ic;
          ic = (int)m_buffer->readable();

          return ic;
      }

      /*!
       * 
       * 
       */
      void set_m_mode(int mode)
      {
          m_test_mode = mode;
      }
  private:
       RTC::CdrBufferBase* m_buffer;
       ::OpenRTM::CdrData  m_put_data;
       int m_test_mode;
  };
  class MockConsumer : public RTC::InPortConsumer
  {
  public:
	
    MockConsumer() : RTC::InPortConsumer()
    {
      clearLastTime();
    }
		
    virtual ~MockConsumer() {}
		
    virtual void push()
    {
      timeval now;
      coil::gettimeofday(&now, NULL);
			
      if (! isLastTimeCleared())
	{
	  long interval =
	    (now.tv_sec - _lastTime.tv_sec) * 1000000
	    + (now.tv_usec - _lastTime.tv_usec);
				
	  _intervalTicks.push_back(interval);
	}
			
      _lastTime = now;
    }
		
    virtual RTC::InPortConsumer* clone() const
    {
      MockConsumer* clone = new MockConsumer();
      copy(_intervalTicks.begin(), _intervalTicks.end(), clone->_intervalTicks.begin());
      clone->_lastTime = _lastTime;
			
      return clone;
    }

    virtual bool subscribeInterface(const SDOPackage::NVList&)
    {
      return true;
    }
		
    virtual void unsubscribeInterface(const SDOPackage::NVList&)
    {
      return;
    }
	
    virtual const std::vector<long>& getIntervalTicks() const
    {
      return _intervalTicks;
    }
		
    virtual int getCount() const
    {
      return static_cast<int>(_intervalTicks.size());
    }
		
    virtual void init(coil::Properties& prop)
    {
    }
    virtual InPortConsumer::ReturnCode put(const cdrMemoryStream& data)
    {
        return (InPortConsumer::ReturnCode)0 ;
    }
    virtual void publishInterfaceProfile(SDOPackage::NVList& properties)
    {
    }
  private:
	
    std::vector<long> _intervalTicks;
    timeval _lastTime;
		
  private:
	
    void clearLastTime()
    {
      _lastTime.tv_sec = 0;
      _lastTime.tv_usec = 0;
    }
		
    bool isLastTimeCleared()
    {
      return (_lastTime.tv_sec == 0) && (_lastTime.tv_usec == 0);
    }
  };
	
  class CounterConsumer : public RTC::InPortConsumer
  {
  public:
	
    CounterConsumer(CounterConsumer* component = NULL)
      : RTC::InPortConsumer(), _count(0), _component(component)	{}
		
    virtual ~CounterConsumer() {}
		
    virtual void push()
    {
      _count++;
			
      if (_component != NULL)
	{
	  _component->push();
	}
    }
		
    virtual RTC::InPortConsumer* clone() const
    {
      CounterConsumer* clone = new CounterConsumer();
      clone->_count = _count;
      clone->_component = _component;
      return clone;
    }

    virtual bool subscribeInterface(const SDOPackage::NVList&)
    {
      return true;
    }
		
    virtual void unsubscribeInterface(const SDOPackage::NVList&)
    {
      return;
    }

    virtual int getCount() const
    {
      return _count;
    }
	
    virtual void init(coil::Properties& prop)
    {
    }
    virtual InPortConsumer::ReturnCode put(const cdrMemoryStream& data)
    {
        return (InPortConsumer::ReturnCode)0 ;
    }
    virtual void publishInterfaceProfile(SDOPackage::NVList& properties)
    {
    }
  private:
	
    int _count;
    CounterConsumer* _component;
  };

  class PublisherPeriodicTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(PublisherPeriodicTests);
		
    //CPPUNIT_TEST(test_init);  // OK
    CPPUNIT_TEST(test_setConsumer);
    CPPUNIT_TEST(test_setBuffer);
    //CPPUNIT_TEST(test_activate_deactivate_isActive);  // OK
    CPPUNIT_TEST(test_pushAll);
    CPPUNIT_TEST(test_pushAll_2);
    CPPUNIT_TEST(test_pushFifo);
    CPPUNIT_TEST(test_pushFifo_2);
    CPPUNIT_TEST(test_pushSkip);
    CPPUNIT_TEST(test_pushSkip_2);
    CPPUNIT_TEST(test_pushNew);
    CPPUNIT_TEST(test_write);
		
    CPPUNIT_TEST_SUITE_END();
		
  public:
    RTC::ConnectorListeners m_listeners;
	
    /*!
     * @brief Constructor
     */
    PublisherPeriodicTests()
    {
    }
		
    /*!
     * @brief Destructor
     */
    ~PublisherPeriodicTests()
    {
    }
		
    /*!
     * @brief Test initialization
     */
    virtual void setUp()
    {
//      coil::usleep(1000000);
    }
    
    /*!
     * @brief Test finalization
     */
    virtual void tearDown()
    { 
    }
		
    /*!
     * @brief init()メソッドのテスト
     * 
     */
    void test_init(void)
    {

        PublisherPeriodicMock publisher;
        RTC::PublisherBase::ReturnCode retcode;
        coil::Properties prop;

        //Propertiesが空の状態ではエラーになることを確認する
        retcode = publisher.init(prop);
        //coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::INVALID_ARGS, retcode);

        prop.setProperty("publisher.push_policy","new");
        prop.setProperty("thread_type","bar");
        prop.setProperty("measurement.exec_time","default");
        prop.setProperty("measurement.period_count","1");
        prop.setProperty("publisher.push_rate","10.0");

        //thread_type が不正の場合 INVALID_ARGS を返すことを確認する。
        retcode = publisher.init(prop);
        //coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, retcode);

        //以下のpropertiesの設定で動作することを確認する。
        prop.setProperty("publisher.push_policy","all");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","10");
        retcode = publisher.init(prop);
        //coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, retcode);

        prop.setProperty("publisher.push_policy","fifo");
        prop.setProperty("publisher.skip_count","1");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","disable");
        prop.setProperty("measurement.exec_count","1");
        prop.setProperty("measurement.period_time","disable");
        prop.setProperty("measurement.period_count","1");
        prop.setProperty("publisher.push_rate","20");
        retcode = publisher.init(prop);
        //coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, retcode);

        prop.setProperty("publisher.push_policy","skip");
        prop.setProperty("publisher.skip_count","-1");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","bar");
        prop.setProperty("measurement.exec_count","-1");
        prop.setProperty("measurement.period_time","bar");
        prop.setProperty("measurement.period_count","-1");
        prop.setProperty("publisher.push_rate","10");
        retcode = publisher.init(prop);
        //coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, retcode);

        prop.setProperty("publisher.push_policy","new");
        prop.setProperty("publisher.skip_count","foo");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","foo");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","foo");
        prop.setProperty("publisher.push_rate","10");
        retcode = publisher.init(prop);
        //coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, retcode);

        prop.setProperty("publisher.push_policy","bar");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","10");
        retcode = publisher.init(prop);
        //coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, retcode);

        prop.setProperty("publisher.push_policy","all");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","0");
        retcode = publisher.init(prop);
        //coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::INVALID_ARGS, retcode);

        prop.setProperty("publisher.push_policy","all");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","-1");
        retcode = publisher.init(prop);
        //coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::INVALID_ARGS, retcode);

    }
    /*!
     * @brief setConsumer()メソッドのテスト
     * 
     */
    void test_setConsumer(void)
    {
        RTC::InPortCorbaCdrConsumer* consumer0 
                                    = new RTC::InPortCorbaCdrConsumer();
        RTC::InPortCorbaCdrConsumer* consumer1 
                                    = new RTC::InPortCorbaCdrConsumer();
        RTC::PublisherPeriodic publisher;

        //NULLを渡した場合INVALID_ARGSとなることを確認する。
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::INVALID_ARGS, 
                             publisher.setConsumer(NULL));

        //
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, 
                             publisher.setConsumer(consumer0));

        //
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, 
                             publisher.setConsumer(consumer1));

        delete consumer1;
        delete consumer0;
    }
    /*!
     * @brief setBuffer()メソッドのテスト
     * 
     */
    void test_setBuffer(void)
    {
        RTC::CdrBufferBase* buffer0 = new RTC::CdrRingBuffer();
        RTC::CdrBufferBase* buffer1 = new RTC::CdrRingBuffer();
        RTC::PublisherPeriodic publisher;

        //NULLを渡した場合INVALID_ARGSとなることを確認する。
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::INVALID_ARGS, 
                             publisher.setBuffer(NULL));

        //
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, 
                             publisher.setBuffer(buffer0));

        //
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, 
                             publisher.setBuffer(buffer1));

        delete buffer1;
        delete buffer0;
    }
    /*!
     * @brief activate(),deactivate(),isActiveメソッドのテスト
     * 
     */
    void test_activate_deactivate_isActive(void)
    {
        RTC::InPortCorbaCdrConsumer* consumer 
                                    = new RTC::InPortCorbaCdrConsumer();
        RTC::PublisherPeriodic publisher;
        publisher.setConsumer(consumer);

        //init() せずに activate() をコールした場合をi
        //PRECONDITION_NOT_MET 返すことを確認する。
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PRECONDITION_NOT_MET, 
                             publisher.activate());

        //init() せずに deactivate() をコールした場合をi
        //PRECONDITION_NOT_MET 返すことを確認する。
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PRECONDITION_NOT_MET, 
                             publisher.deactivate());

        //init()
        coil::Properties prop;
        prop.setProperty("publisher.push_policy","all");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","20");
        publisher.init(prop);
        coil::usleep(10000);

        
        //setBuffer() せずに activate() をコールした場合をi
        //PRECONDITION_NOT_MET 返すことを確認する。
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PRECONDITION_NOT_MET, 
                             publisher.activate());

        //setBuffer()
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        publisher.setBuffer(buffer);


        CPPUNIT_ASSERT_EQUAL(false, 
                             publisher.isActive());
        
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, 
                             publisher.activate());

        CPPUNIT_ASSERT_EQUAL(true, 
                             publisher.isActive());

        //既に activate されている場合は 
        //activateすると
        //PORT_OK を返すことを確認する。
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, 
                             publisher.activate());

        CPPUNIT_ASSERT_EQUAL(true, 
                             publisher.isActive());
        
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, 
                             publisher.deactivate());

        CPPUNIT_ASSERT_EQUAL(false, 
                             publisher.isActive());
        
        //activate されていない状態で、
        //deactivateすると
        //PORT_OK を返すことを確認する。
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK, 
                             publisher.deactivate());
        
        CPPUNIT_ASSERT_EQUAL(false, 
                             publisher.isActive());
        
        delete buffer;
        delete consumer;
    }
    /*!
     * @brief write(), pushAll() メソッドのテスト
     *
     * -provider 側のバッファ full 状態でもデータ抜けががないことを確認する。
     */
    void test_pushAll(void)
    {
        InPortCorbaCdrConsumerMock* consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherPeriodicMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","all");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","10");
        publisher.init(prop);
        coil::usleep(10000);

        //ConnectorInfo
        coil::vstring ports;
        RTC::ConnectorInfo info("name", "id", ports, prop);

        //ConnectorListeners
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE].addListener(
                                   new DataListener("ON_BUFFER_WRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_FULL].addListener(
                                   new DataListener("ON_BUFFER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE_TIMEOUT].addListener(
                                   new DataListener("ON_BUFFER_WRITE_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_OVERWRITE].addListener(
                                   new DataListener("ON_BUFFER_OVERWRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_READ].addListener(
                                   new DataListener("ON_BUFFER_READ"), true);
        m_listeners.connectorData_[RTC::ON_SEND].addListener(
                                   new DataListener("ON_SEND"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVED].addListener(
                                   new DataListener("ON_RECEIVED"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_FULL].addListener(
                                   new DataListener("ON_RECEIVER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_TIMEOUT].addListener(
                                   new DataListener("ON_RECEIVER_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_ERROR].addListener(
                                   new DataListener("ON_RECEIVER_ERROR"), true);

        // setListener
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::INVALID_ARGS, 
                             publisher.setListener(info, 0));
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::PORT_OK, 
                             publisher.setListener(info, &m_listeners));

        publisher.setConsumer(consumer);
        publisher.setBuffer(buffer);
        publisher.activate();

        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = icc;
            td >>= cdr;

            CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));

        }

        coil::usleep(150000);
        //provider 側のバッファ full の状態でコール(full)
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 8;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 9;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }

        //provider 側のバッファから 4 件取得
        //(full ではない状態にする )
        for(int icc(0);icc<4;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc, (long)rtd.data);
        }

        coil::usleep(150000);
        //provider 側のバッファ full ではない状態でコール
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 10;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 11;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }

        //データ抜けががないことを確認する。
        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc+4, (long)rtd.data);
        }
        publisher.deactivate();
        
        delete buffer;
        delete consumer;
        
    }
    /*!
     * @brief write(), pushAll() メソッドのテスト
     *
     * -
     */
    void test_pushAll_2(void)
    {
        InPortCorbaCdrConsumerMock* consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherPeriodicMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","all");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","10");
        publisher.init(prop);
        coil::usleep(10000);

        //ConnectorInfo
        coil::vstring ports;
        RTC::ConnectorInfo info("name", "id", ports, prop);

        //ConnectorListeners
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE].addListener(
                                   new DataListener("ON_BUFFER_WRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_FULL].addListener(
                                   new DataListener("ON_BUFFER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE_TIMEOUT].addListener(
                                   new DataListener("ON_BUFFER_WRITE_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_OVERWRITE].addListener(
                                   new DataListener("ON_BUFFER_OVERWRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_READ].addListener(
                                   new DataListener("ON_BUFFER_READ"), true);
        m_listeners.connectorData_[RTC::ON_SEND].addListener(
                                   new DataListener("ON_SEND"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVED].addListener(
                                   new DataListener("ON_RECEIVED"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_FULL].addListener(
                                   new DataListener("ON_RECEIVER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_TIMEOUT].addListener(
                                   new DataListener("ON_RECEIVER_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_ERROR].addListener(
                                   new DataListener("ON_RECEIVER_ERROR"), true);

        // setListener
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::INVALID_ARGS, 
                             publisher.setListener(info, 0));
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::PORT_OK, 
                             publisher.setListener(info, &m_listeners));

        publisher.setConsumer(consumer);
        publisher.setBuffer(buffer);
        publisher.activate();

        for(int icc(0);icc<16;++icc)
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = icc;
            td >>= cdr;

            RTC::PublisherBase::ReturnCode ret;
            ret = publisher.write(cdr,0,0);
            if(icc<9)
            {
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                     ret);
            }
            else
            {
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::BUFFER_FULL,
                                     ret);
            }
            coil::usleep(150000);

        }

        //consumer と provider 両方の buffer が full 状態のため、
        // この write データは抜ける。
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 16;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }

        //データを確認する。
        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc, (long)rtd.data);
        }

        coil::usleep(300000);
        //データを確認する。
        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc+8, (long)rtd.data);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 17;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }
        //データを確認する。
        {
        cdrMemoryStream data;
        data = consumer->get_m_put_data();
        CORBA::ULong inlen = data.bufSize();
        CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

        RTC::TimedLong rtd;
        rtd <<= data;
        CPPUNIT_ASSERT_EQUAL((long)17, (long)rtd.data);
        }

        coil::usleep(150000);
        publisher.deactivate();
        
        delete buffer;
        delete consumer;
        
    }
    /*!
     * @brief pushFifo()メソッドのテスト
     * 
     */
    void test_pushFifo(void)
    {
        InPortCorbaCdrConsumerMock* consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherPeriodicMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","fifo");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","10");
        publisher.init(prop);
        coil::usleep(10000);

        //ConnectorInfo
        coil::vstring ports;
        RTC::ConnectorInfo info("name", "id", ports, prop);

        //ConnectorListeners
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE].addListener(
                                   new DataListener("ON_BUFFER_WRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_FULL].addListener(
                                   new DataListener("ON_BUFFER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE_TIMEOUT].addListener(
                                   new DataListener("ON_BUFFER_WRITE_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_OVERWRITE].addListener(
                                   new DataListener("ON_BUFFER_OVERWRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_READ].addListener(
                                   new DataListener("ON_BUFFER_READ"), true);
        m_listeners.connectorData_[RTC::ON_SEND].addListener(
                                   new DataListener("ON_SEND"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVED].addListener(
                                   new DataListener("ON_RECEIVED"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_FULL].addListener(
                                   new DataListener("ON_RECEIVER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_TIMEOUT].addListener(
                                   new DataListener("ON_RECEIVER_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_ERROR].addListener(
                                   new DataListener("ON_RECEIVER_ERROR"), true);

        // setListener
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::INVALID_ARGS, 
                             publisher.setListener(info, 0));
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::PORT_OK, 
                             publisher.setListener(info, &m_listeners));

        publisher.setConsumer(consumer);
        publisher.setBuffer(buffer);
        publisher.activate();

        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = icc;
            td >>= cdr;

            CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));

        coil::usleep(150000);
        }

        //provider 側のバッファ full の状態でコール(full)
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 8;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 9;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }

        //provider 側のバッファから 4 件取得
        //(full ではない状態にする )
        for(int icc(0);icc<4;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc, (long)rtd.data);
        }
        coil::usleep(220000); //Waits to complete writing 8 and 9. 

        //provider 側のバッファ full ではない状態でコール
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 10;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 11;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 12;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 13;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }

        //データ抜けががないことを確認する。
        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc+4, (long)rtd.data);
        }
        coil::usleep(400000);  //Meanwhile, 12 and 13 are forwarded. 
        for(int icc(0);icc<2;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc+12, (long)rtd.data);
        }


        coil::usleep(10000);
        publisher.deactivate();
        delete buffer;
        delete consumer;
        
    }
    /*!
     * @brief write(), pushFifo() メソッドのテスト
     *
     * -
     */
    void test_pushFifo_2(void)
    {
        InPortCorbaCdrConsumerMock* consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherPeriodicMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","fifo");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","10");
        publisher.init(prop);
        coil::usleep(10000);

        //ConnectorInfo
        coil::vstring ports;
        RTC::ConnectorInfo info("name", "id", ports, prop);

        //ConnectorListeners
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE].addListener(
                                   new DataListener("ON_BUFFER_WRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_FULL].addListener(
                                   new DataListener("ON_BUFFER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE_TIMEOUT].addListener(
                                   new DataListener("ON_BUFFER_WRITE_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_OVERWRITE].addListener(
                                   new DataListener("ON_BUFFER_OVERWRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_READ].addListener(
                                   new DataListener("ON_BUFFER_READ"), true);
        m_listeners.connectorData_[RTC::ON_SEND].addListener(
                                   new DataListener("ON_SEND"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVED].addListener(
                                   new DataListener("ON_RECEIVED"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_FULL].addListener(
                                   new DataListener("ON_RECEIVER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_TIMEOUT].addListener(
                                   new DataListener("ON_RECEIVER_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_ERROR].addListener(
                                   new DataListener("ON_RECEIVER_ERROR"), true);

        // setListener
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::INVALID_ARGS, 
                             publisher.setListener(info, 0));
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::PORT_OK, 
                             publisher.setListener(info, &m_listeners));

        publisher.setConsumer(consumer);
        publisher.setBuffer(buffer);
        publisher.activate();

        //consumer と provider 両方の buffer を full 状態にする
        for(int icc(0);icc<16;++icc)
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = icc;
            td >>= cdr;

            RTC::PublisherBase::ReturnCode ret;
            ret = publisher.write(cdr,0,0);
            if(icc<9)
            {
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                     ret);
            }
            else
            {
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::BUFFER_FULL,
                                     ret);
            }
            coil::usleep(150000);

        }

        //consumer と provider 両方の buffer が full 状態のため、
        // この write データは抜ける。
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 16;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(100000);
        }

        //データを確認する。
        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc, (long)rtd.data);
        }

        coil::usleep(800000);
        // この write データは転送される。
        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = 17+icc;
            td >>= cdr;
            RTC::PublisherBase::ReturnCode ret;
            ret = publisher.write(cdr,0,0);
            if(icc<1)
            {
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                     ret);
            }
            //else
            //{
            //    CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::BUFFER_FULL,
            //                         ret);
            //}
            coil::usleep(100000);
        }
        coil::usleep(80000);
        //データを確認する。
        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc+8, (long)rtd.data);
        }
        coil::usleep(100000);
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 25;
        td >>= cdr;
        //CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
        //                         publisher.write(cdr,0,0));
        coil::usleep(100000);
        }
        coil::usleep(800000);
        //データを確認する。
        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc+17, (long)rtd.data);
        }


        coil::usleep(100000);
        publisher.deactivate();
        
        delete buffer;
        delete consumer;
        
    }
    /*!
     * @brief pushSklip()メソッドのテスト
     * 
     */
    void test_pushSkip(void)
    {
        InPortCorbaCdrConsumerMock* consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherPeriodicMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","skip");
        prop.setProperty("publisher.skip_count","1");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","10.0");
        publisher.init(prop);
        coil::usleep(10000);

        //ConnectorInfo
        coil::vstring ports;
        RTC::ConnectorInfo info("name", "id", ports, prop);

        //ConnectorListeners
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE].addListener(
                                   new DataListener("ON_BUFFER_WRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_FULL].addListener(
                                   new DataListener("ON_BUFFER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE_TIMEOUT].addListener(
                                   new DataListener("ON_BUFFER_WRITE_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_OVERWRITE].addListener(
                                   new DataListener("ON_BUFFER_OVERWRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_READ].addListener(
                                   new DataListener("ON_BUFFER_READ"), true);
        m_listeners.connectorData_[RTC::ON_SEND].addListener(
                                   new DataListener("ON_SEND"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVED].addListener(
                                   new DataListener("ON_RECEIVED"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_FULL].addListener(
                                   new DataListener("ON_RECEIVER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_TIMEOUT].addListener(
                                   new DataListener("ON_RECEIVER_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_ERROR].addListener(
                                   new DataListener("ON_RECEIVER_ERROR"), true);

        // setListener
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::INVALID_ARGS, 
                             publisher.setListener(info, 0));
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::PORT_OK, 
                             publisher.setListener(info, &m_listeners));

        publisher.setConsumer(consumer);
        publisher.setBuffer(buffer);
        publisher.activate();

        for(int icc(0);icc<16;++icc)
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = icc;
            td >>= cdr;

            CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));

            coil::usleep(150000);
        }

        //provider 側のバッファ full の状態でコール(full)
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 16;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 17;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }

        //provider 側のバッファから 4 件取得
        //(full ではない状態にする )
        for(int icc(0);icc<4;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc*2+1, (long)rtd.data);
        }

        coil::usleep(450000);
        //provider 側のバッファ full ではない状態でコール
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 18;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 19;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }

        //データ抜けががないことを確認する。
        CPPUNIT_ASSERT_EQUAL(6,
                              consumer->get_m_put_data_len());
        for(int icc(0);icc<6;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc*2+9, (long)rtd.data);
        }
       
        coil::usleep(100000);
        publisher.deactivate();
        delete buffer;
        delete consumer;
    }
    /*!
     * @brief write(), pushSkip() メソッドのテスト
     *
     * -
     */
    void test_pushSkip_2(void)
    {
        InPortCorbaCdrConsumerMock* consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherPeriodicMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","skip");
        prop.setProperty("publisher.skip_count","1");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","10.0");
        publisher.init(prop);
        coil::usleep(10000);

        //ConnectorInfo
        coil::vstring ports;
        RTC::ConnectorInfo info("name", "id", ports, prop);

        //ConnectorListeners
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE].addListener(
                                   new DataListener("ON_BUFFER_WRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_FULL].addListener(
                                   new DataListener("ON_BUFFER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE_TIMEOUT].addListener(
                                   new DataListener("ON_BUFFER_WRITE_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_OVERWRITE].addListener(
                                   new DataListener("ON_BUFFER_OVERWRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_READ].addListener(
                                   new DataListener("ON_BUFFER_READ"), true);
        m_listeners.connectorData_[RTC::ON_SEND].addListener(
                                   new DataListener("ON_SEND"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVED].addListener(
                                   new DataListener("ON_RECEIVED"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_FULL].addListener(
                                   new DataListener("ON_RECEIVER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_TIMEOUT].addListener(
                                   new DataListener("ON_RECEIVER_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_ERROR].addListener(
                                   new DataListener("ON_RECEIVER_ERROR"), true);

        // setListener
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::INVALID_ARGS, 
                             publisher.setListener(info, 0));
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::PORT_OK, 
                             publisher.setListener(info, &m_listeners));

        publisher.setConsumer(consumer);
        publisher.setBuffer(buffer);
        publisher.activate();

        //consumer と provider 両方の buffer を full 状態にする
        for(int icc(0);icc<25;++icc)
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = icc;
            td >>= cdr;

            RTC::PublisherBase::ReturnCode ret;
            ret = publisher.write(cdr,0,0);
            if(icc<18)
            {
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                     ret);
            }
            else
            {
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::BUFFER_FULL,
                                     ret);
            }
            coil::usleep(150000);

        }

        //consumer と provider 両方の buffer が full 状態のため、
        // この write データは抜ける。
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 25;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }

        //データを確認する。
        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc*2+1, (long)rtd.data);
        }

        coil::usleep(150000);
        //データを確認する。
        int len =consumer->get_m_put_data_len();
        CPPUNIT_ASSERT_EQUAL(4,len);
        for(int icc(0);icc<len;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)icc*2+17, (long)rtd.data);
        }
        coil::usleep(150000);
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 26;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 27;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(150000);
        }
        //データを確認する。
        {

        cdrMemoryStream data;
        data = consumer->get_m_put_data();
        CORBA::ULong inlen = data.bufSize();
        CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

        RTC::TimedLong rtd;
        rtd <<= data;
        CPPUNIT_ASSERT_EQUAL((long)26, (long)rtd.data);
        }

        coil::usleep(100000);
        publisher.deactivate();
        
        delete buffer;
        delete consumer;
        
    }
    /*!
     * @brief pushNew()メソッドのテスト
     * 
     */
    void test_pushNew(void)
    {
        InPortCorbaCdrConsumerMock* consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherPeriodicMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","new");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","10");
        publisher.init(prop);
        coil::usleep(10000);

        //ConnectorInfo
        coil::vstring ports;
        RTC::ConnectorInfo info("name", "id", ports, prop);

        //ConnectorListeners
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE].addListener(
                                   new DataListener("ON_BUFFER_WRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_FULL].addListener(
                                   new DataListener("ON_BUFFER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE_TIMEOUT].addListener(
                                   new DataListener("ON_BUFFER_WRITE_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_OVERWRITE].addListener(
                                   new DataListener("ON_BUFFER_OVERWRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_READ].addListener(
                                   new DataListener("ON_BUFFER_READ"), true);
        m_listeners.connectorData_[RTC::ON_SEND].addListener(
                                   new DataListener("ON_SEND"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVED].addListener(
                                   new DataListener("ON_RECEIVED"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_FULL].addListener(
                                   new DataListener("ON_RECEIVER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_TIMEOUT].addListener(
                                   new DataListener("ON_RECEIVER_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_ERROR].addListener(
                                   new DataListener("ON_RECEIVER_ERROR"), true);

        // setListener
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::INVALID_ARGS, 
                             publisher.setListener(info, 0));
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::PORT_OK, 
                             publisher.setListener(info, &m_listeners));

        publisher.setConsumer(consumer);
        publisher.setBuffer(buffer);
        publisher.activate();

        //8件のデータは転送されない
        //最新データの7は転送される。
        for(int icc(0);icc<8;++icc)
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = icc;
            td >>= cdr;

            CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));

        }

  
        coil::usleep(150000);
        //provider 側のバッファから取得
        //
        int len = consumer->get_m_put_data_len() -1;
        for(int icc(0);icc<len;++icc)
        {
            cdrMemoryStream data;
            data = consumer->get_m_put_data();
            CORBA::ULong inlen = data.bufSize();
            CPPUNIT_ASSERT_EQUAL(12,(int)inlen);
        }
        coil::usleep(150000);
        //最新データが転送されていることを確認する。
        {
        cdrMemoryStream data;
        data = consumer->get_m_put_data();
        CORBA::ULong inlen = data.bufSize();
        CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

        RTC::TimedLong rtd;
        rtd <<= data;
        CPPUNIT_ASSERT_EQUAL((long)7, (long)rtd.data);
        }

        coil::usleep(100000);
        publisher.deactivate();
        delete buffer;
        delete consumer;
    }
    /*!
     * @brief write()メソッドのテスト
     * 
     * - 手順を無視して write した場合
     */
    void test_write(void)
    {
        InPortCorbaCdrConsumerMock* consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherPeriodicMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","all");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        prop.setProperty("publisher.push_rate","10");
        publisher.init(prop);
        coil::usleep(10000);

        //ConnectorInfo
        coil::vstring ports;
        RTC::ConnectorInfo info("name", "id", ports, prop);

        //ConnectorListeners
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE].addListener(
                                   new DataListener("ON_BUFFER_WRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_FULL].addListener(
                                   new DataListener("ON_BUFFER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_WRITE_TIMEOUT].addListener(
                                   new DataListener("ON_BUFFER_WRITE_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_OVERWRITE].addListener(
                                   new DataListener("ON_BUFFER_OVERWRITE"), true);
        m_listeners.connectorData_[RTC::ON_BUFFER_READ].addListener(
                                   new DataListener("ON_BUFFER_READ"), true);
        m_listeners.connectorData_[RTC::ON_SEND].addListener(
                                   new DataListener("ON_SEND"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVED].addListener(
                                   new DataListener("ON_RECEIVED"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_FULL].addListener(
                                   new DataListener("ON_RECEIVER_FULL"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_TIMEOUT].addListener(
                                   new DataListener("ON_RECEIVER_TIMEOUT"), true);
        m_listeners.connectorData_[RTC::ON_RECEIVER_ERROR].addListener(
                                   new DataListener("ON_RECEIVER_ERROR"), true);

        // setListener
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::INVALID_ARGS, 
                             publisher.setListener(info, 0));
        CPPUNIT_ASSERT_EQUAL(RTC::DataPortStatus::PORT_OK, 
                             publisher.setListener(info, &m_listeners));

        //
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 123;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PRECONDITION_NOT_MET,
                                 publisher.write(cdr,0,0));
        coil::usleep(100000);
        }

        //
        publisher.setBuffer(buffer);
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 123;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PRECONDITION_NOT_MET,
                                 publisher.write(cdr,0,0));
        coil::usleep(100000);
        }

        //
        publisher.setConsumer(consumer);
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 123;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherPeriodic::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(100000);
        }

        //Listener callback check
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 777;
        td >>= cdr;
        //m_OnCheck = 1;  // PORT_OK:onReceived()
        //publisher.write(cdr,0,0);
        //coil::usleep(10000);
        m_OnCheck = 2;  // PORT_ERROR:onReceiverError()
        publisher.write(cdr,0,0);
        coil::usleep(10000);
        m_OnCheck = 3;  // SEND_FULL:onReceiverFull()
        publisher.write(cdr,0,0);
        coil::usleep(100000);
        m_OnCheck = 4;  // SEND_TIMEOUT:onReceiverTimeout()
        publisher.write(cdr,0,0);
        coil::usleep(100000);
        m_OnCheck = 5;  // UNKNOWN_ERROR:onReceiverError()
        publisher.write(cdr,0,0);
        coil::usleep(100000);
        m_OnCheck = 6;  // CONNECTION_LOST:onReceiverError()
        publisher.write(cdr,0,0);
        coil::usleep(100000);

        delete buffer;
        delete consumer;
    }
    /*!
     * @brief デストラクタのテスト
     * 
     * - デストラクタ呼出により、release()メソッドを呼び出さずともPublisherの動作が問題なく停止するか？
     */
/*
    void test_destructor()
    {
      CounterConsumer* consumer1 = new CounterConsumer();

      { // Publisherのインスタンススコープ開始
	CounterConsumer* consumer2 = new CounterConsumer(consumer1);
	coil::Properties prop;
	prop.setProperty("dataport.push_rate", "10"); // 10 [Hz]
	RTC::PublisherPeriodic publisher(consumer2, prop);
	// 5 [sec]だけ動作させる
	coil::usleep(5000000);
				
      } // デストラクタを呼び出す（スコープを終了させる）
			
      coil::usleep(1000000); // 完全停止するまで待つ
			
      // この時点での呼出回数を記録する
      int countReleased = consumer1->getCount();
			
      // さらにConsumerがコールバックされ得る時間を与える
      coil::usleep(5000000); // 5 [sec]
			
      // この時点での呼出回数を取得し、先に記録しておいた回数から変化がない
      // （つまり、Publisherの動作が停止している）ことを確認する
      int countSleeped = consumer1->getCount();
      CPPUNIT_ASSERT_EQUAL(countReleased, countSleeped);
    }
*/
		
    /*!
     * @brief release()メソッドのテスト
     * 
     * - release()メソッド呼出によりPublisherの動作が停止するか？
     */
/*
    void test_release()
    {
      CounterConsumer* consumer = new CounterConsumer();
      coil::Properties prop;
      prop.setProperty("dataport.push_rate", "10"); // 10 [Hz]
      RTC::PublisherPeriodic publisher(consumer, prop);
			
      // 5 [sec]だけ動作させる
      coil::usleep(5000000);
			
      // Publisherの動作を停止させる
      publisher.release();
      coil::usleep(1000000); // 完全停止するまで待つ
			
      // この時点での呼出回数を記録する
      int countReleased = consumer->getCount();
			
      // さらにConsumerがコールバックされ得る時間を与える
      coil::usleep(5000000); // 5 [sec]
			
      // この時点での呼出回数を取得し、先に記録しておいた回数から変化がない
      // （つまり、Publisherの動作が停止している）ことを確認する
      int countSleeped = consumer->getCount();
      CPPUNIT_ASSERT_EQUAL(countReleased, countSleeped);
    }
*/
		
    /*!
     * @brief PublisherによるConsumer呼出間隔精度のテスト
     * 
     * - Publisherに指定した時間間隔で、正しくConsumerがコールバックされるか？
     */
/*
    void test_interval_accuracy()
    {
      MockConsumer* consumer = new MockConsumer();
      coil::Properties prop;
      prop.setProperty("dataport.push_rate", "10"); // 10 [Hz]
      RTC::PublisherPeriodic publisher(consumer, prop);
			
      // 5 [sec]だけ動作させる
      coil::usleep(5000000);
			
      // Publisherの動作を停止させる
      publisher.release();
      coil::usleep(1000000); // 完全停止するまで待つ
			
      // 指定した時間間隔で正しくConsumerがコールバックされているか？
      long permissibleTickMin = static_cast<long>(100000 * 0.9);
      long permissibleTickMax = static_cast<long>(100000 * 1.1);
      const std::vector<long> intervalTicks = consumer->getIntervalTicks();
      CPPUNIT_ASSERT(intervalTicks.size() > 0);

      for (std::vector<long>::size_type i = 0; i < intervalTicks.size(); i++)
	{
	  long tick = intervalTicks[i];
	  CPPUNIT_ASSERT((permissibleTickMin <= tick) && (tick <= permissibleTickMax));
	}
    }
*/
  };
}; // namespace PublisherPeriodic

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(PublisherPeriodic::PublisherPeriodicTests);

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
#endif // PublisherPeriodic_cpp
