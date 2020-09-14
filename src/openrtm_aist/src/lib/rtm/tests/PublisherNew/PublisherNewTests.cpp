// -*- C++ -*-
/*!
 * @file   PublisherNewTests.cpp
 * @brief  PublisherNew test class
 * @date   $Date: 2008/01/23 09:19:31 $
 * @author Noriaki Ando <n-ando@aist.go.jp>
 *
 * $Id$
 *
 */

/*
 * $Log: PublisherNewTests.cpp,v $
 * Revision 1.2  2008/01/23 09:19:31  arafune
 * added some tests.
 *
 * Revision 1.1  2007/12/20 07:50:16  arafune
 * *** empty log message ***
 *
 * Revision 1.2  2007/01/12 14:54:39  n-ando
 * The constructor's signature was changed.
 * InPortConsumer base class is now abstruct class. It needs concrete class.
 *
 * Revision 1.1  2006/12/18 06:51:43  n-ando
 * The first commitment.
 *
 *
 */

#ifndef PublisherNew_cpp
#define PublisherNew_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <coil/Properties.h>
#include <rtm/InPortConsumer.h>
#include <rtm/PublisherNew.h>
#include <coil/Time.h>
#include <rtm/CORBA_SeqUtil.h>
#include <rtm/NVUtil.h>
#include <rtm/InPortCorbaCdrConsumer.h>
#include <rtm/CdrRingBuffer.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/ConnectorListener.h>

/*!
 * @class PublisherNewTests class
 * @brief PublisherNew test
 */
namespace PublisherNew
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
  class PublisherNewMock
    : public RTC::PublisherNew
  {
  public:
    PublisherNewMock(void)
    {
        ;
    }
    virtual ~PublisherNewMock(void)
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
                   return RTC::PublisherNew::SEND_FULL;
              }

              RTC::BufferStatus::Enum ret = m_buffer->write(data);

              //Listener check
              if(m_OnCheck == 0) {
                  switch(ret)
                  {
                       case RTC::BufferStatus::BUFFER_OK:
                           return RTC::PublisherNew::PORT_OK;
                           break;
                       case RTC::BufferStatus::BUFFER_ERROR:
                           return RTC::PublisherNew::PORT_ERROR;
                           break;
                       case RTC::BufferStatus::BUFFER_FULL:
                           return RTC::PublisherNew::SEND_FULL;
                           break;
                       case RTC::BufferStatus::BUFFER_EMPTY:
                           return RTC::PublisherNew::BUFFER_EMPTY;
                           break;
                       case RTC::BufferStatus::TIMEOUT:
                           return RTC::PublisherNew::SEND_TIMEOUT;
                           break;
                       default:
                           return RTC::PublisherNew::UNKNOWN_ERROR;
                   }
                   return RTC::PublisherNew::UNKNOWN_ERROR;
              }
              else if(m_OnCheck == 1) {
                   return RTC::PublisherNew::PORT_OK;
              }
              else if(m_OnCheck == 2) {
                   return RTC::PublisherNew::PORT_ERROR;
              }
              else if(m_OnCheck == 3) {
                   return RTC::PublisherNew::SEND_FULL;
              }
              else if(m_OnCheck == 4) {
                   return RTC::PublisherNew::SEND_TIMEOUT;
              }
              else if(m_OnCheck == 5) {
                   return RTC::PublisherNew::UNKNOWN_ERROR;
              }
              else if(m_OnCheck == 6) {
                   return RTC::PublisherNew::CONNECTION_LOST;
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
	
  class PublisherNewTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(PublisherNewTests);

    //CPPUNIT_TEST(test_init);  // OK
    CPPUNIT_TEST(test_setConsumer);
    CPPUNIT_TEST(test_setBuffer);
    CPPUNIT_TEST(test_activate_deactivate_isActive);
    CPPUNIT_TEST(test_pushAll);
    CPPUNIT_TEST(test_pushAll_2);
    CPPUNIT_TEST(test_pushFifo);
    CPPUNIT_TEST(test_pushFifo_2);
    CPPUNIT_TEST(test_pushSkip);
    CPPUNIT_TEST(test_pushSkip_2);
    CPPUNIT_TEST(test_pushNew);
    CPPUNIT_TEST(test_write);

    CPPUNIT_TEST_SUITE_END();
		
  private:

  public:
    RTC::ConnectorListeners m_listeners;

    /*!
     * @brief Constructor
     */
    PublisherNewTests()
    {
    }
		
    /*!
     * @brief Destructor
     */
    ~PublisherNewTests()
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
        PublisherNewMock publisher;
        RTC::PublisherBase::ReturnCode retcode;
        coil::Properties prop;

        //Propertiesが空の状態でも正常に動作することを確認する
        retcode = publisher.init(prop);
        coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, retcode);

        prop.setProperty("publisher.push_policy","new");
        prop.setProperty("thread_type","bar");
        prop.setProperty("measurement.exec_time","default");
        prop.setProperty("measurement.period_count","1");

        //thread_type が不正の場合 INVALID_ARGS を返すことを確認する。
        retcode = publisher.init(prop);
        coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::INVALID_ARGS, retcode);

        //以下のpropertiesの設定で動作することを確認する。
        prop.setProperty("publisher.push_policy","all");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        retcode = publisher.init(prop);
        coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, retcode);

        prop.setProperty("publisher.push_policy","fifo");
        prop.setProperty("publisher.skip_count","1");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","disable");
        prop.setProperty("measurement.exec_count","1");
        prop.setProperty("measurement.period_time","disable");
        prop.setProperty("measurement.period_count","1");
        retcode = publisher.init(prop);
        coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, retcode);

        prop.setProperty("publisher.push_policy","skip");
        prop.setProperty("publisher.skip_count","-1");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","bar");
        prop.setProperty("measurement.exec_count","-1");
        prop.setProperty("measurement.period_time","bar");
        prop.setProperty("measurement.period_count","-1");
        retcode = publisher.init(prop);
        coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, retcode);

        prop.setProperty("publisher.push_policy","new");
        prop.setProperty("publisher.skip_count","foo");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","foo");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","foo");
        retcode = publisher.init(prop);
        coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, retcode);

        prop.setProperty("publisher.push_policy","bar");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
        retcode = publisher.init(prop);
        coil::usleep(10000);
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, retcode);

    }
    /*!
     * @brief setConsumer()メソッドのテスト
     * 
     */
    void test_setConsumer(void)
    {
        RTC::InPortCorbaCdrConsumer *consumer0 
                                    = new RTC::InPortCorbaCdrConsumer();
        RTC::InPortCorbaCdrConsumer *consumer1 
                                    = new RTC::InPortCorbaCdrConsumer();
        RTC::PublisherNew publisher;

        //NULLを渡した場合INVALID_ARGSとなることを確認する。
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::INVALID_ARGS, 
                             publisher.setConsumer(NULL));

        //
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, 
                             publisher.setConsumer(consumer0));

        //
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, 
                             publisher.setConsumer(consumer1));

        delete consumer0;
        delete consumer1;
    }
    /*!
     * @brief setBuffer()メソッドのテスト
     * 
     */
    void test_setBuffer(void)
    {
        RTC::CdrBufferBase* buffer0 = new RTC::CdrRingBuffer();
        RTC::CdrBufferBase* buffer1 = new RTC::CdrRingBuffer();
        RTC::PublisherNew publisher;

        //NULLを渡した場合INVALID_ARGSとなることを確認する。
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::INVALID_ARGS, 
                             publisher.setBuffer(NULL));

        //
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, 
                             publisher.setBuffer(buffer0));

        //
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, 
                             publisher.setBuffer(buffer1));

        delete buffer0;
        delete buffer1;
    }
    /*!
     * @brief activate(),deactivate(),isActiveメソッドのテスト
     * 
     */
    void test_activate_deactivate_isActive(void)
    {
        RTC::InPortCorbaCdrConsumer *consumer 
                                    = new RTC::InPortCorbaCdrConsumer();
        RTC::PublisherNew publisher;
        publisher.setConsumer(consumer);

        CPPUNIT_ASSERT_EQUAL(false, 
                             publisher.isActive());
        
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, 
                             publisher.activate());

        CPPUNIT_ASSERT_EQUAL(true, 
                             publisher.isActive());
        
        //既に activate されている場合は 
        //activateすると
        //PORT_OK を返すことを確認する。
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, 
                             publisher.activate());

        CPPUNIT_ASSERT_EQUAL(true, 
                             publisher.isActive());
        
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, 
                             publisher.deactivate());

        CPPUNIT_ASSERT_EQUAL(false, 
                             publisher.isActive());
        
        //activate されていない状態で、
        //deactivateすると
        //PORT_OK を返すことを確認する。
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK, 
                             publisher.deactivate());
        
        CPPUNIT_ASSERT_EQUAL(false, 
                             publisher.isActive());
        
        coil::usleep(10000);
        delete consumer;
    }
    /*!
     * @brief write(), pushAll() メソッドのテスト
     *
     * -provider 側のバッファ full 状態でもデータ抜けががないことを確認する。
     */
    void test_pushAll(void)
    {
        InPortCorbaCdrConsumerMock *consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherNewMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","all");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
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

            CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));

        }

        //provider 側のバッファ full の状態でコール(full)
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 8;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 9;
        td >>= cdr;
        RTC::PublisherBase::ReturnCode ret = publisher.write(cdr,0,0);
        bool bret = false;
        if( (ret == RTC::PublisherNew::PORT_OK) || 
            (ret == RTC::PublisherNew::BUFFER_FULL) ) bret = true;
        CPPUNIT_ASSERT(bret);
        coil::usleep(10000);
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

        //provider 側のバッファ full ではない状態でコール
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 10;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 11;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
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
        
        coil::usleep(10000);
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
        InPortCorbaCdrConsumerMock *consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherNewMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","all");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
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
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                     ret);
            }
            else
            {
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                     ret);
            }
            coil::usleep(10000);

        }

        //consumer と provider 両方の buffer が full 状態のため、
        // この weite データは抜ける。
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 16;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
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

        //consumer の buffer が full 状態のため、
        // この weite データは抜ける。
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 17;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
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
            CPPUNIT_ASSERT_EQUAL((long)icc+8, (long)rtd.data);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 18;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        //データを確認する。
        {
        cdrMemoryStream data;
        data = consumer->get_m_put_data();
        CORBA::ULong inlen = data.bufSize();
        CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

        RTC::TimedLong rtd;
        rtd <<= data;
        CPPUNIT_ASSERT_EQUAL((long)18, (long)rtd.data);
        }

        coil::usleep(10000);
        publisher.deactivate();
        
        coil::usleep(10000);
        delete buffer;
        delete consumer;
        
    }
    /*!
     * @brief pushFifo()メソッドのテスト
     * 
     */
    void test_pushFifo(void)
    {
        InPortCorbaCdrConsumerMock *consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherNewMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","fifo");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
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

            CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));

            coil::usleep(10000);
        }

        //provider 側のバッファ full の状態でコール(full)
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 8;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(50000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 9;
        td >>= cdr;
        //CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
        //                         publisher.write(cdr,0,0));
        publisher.write(cdr,0,0);
        coil::usleep(10000);
        }

        //provider 側のバッファから 4 件取得
        //(full ではない状態にする )
        coil::usleep(10000);
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

        //provider 側のバッファ full ではない状態でコール
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 10;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(30000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 11;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 12;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 13;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
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

        //この時点で consumer 側のバッファにデータが 2 件格納されている状態

        coil::usleep(10000);
        publisher.deactivate();
        coil::usleep(10000);
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
        InPortCorbaCdrConsumerMock *consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherNewMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","fifo");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
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
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                     ret);
            }
            else
            {
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                     ret);
            }
            coil::usleep(10000);

        }

        //consumer と provider 両方の buffer が full 状態のため、
        // この weite データは抜ける。
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 16;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
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

        //consumer の buffer が full 状態のため、
        // この weite データは抜ける。
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 17;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        // この weite データは転送される。
        for(int icc(0);icc<7;++icc)
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = (18+icc);
            td >>= cdr;
            CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
            coil::usleep(10000);
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
            CPPUNIT_ASSERT_EQUAL((long)icc+8, (long)rtd.data);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 26;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        //データを確認する。
        {
        cdrMemoryStream data;
        data = consumer->get_m_put_data();
        CORBA::ULong inlen = data.bufSize();
        CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)18, (long)rtd.data);
        }


        coil::usleep(10000);
        publisher.deactivate();
        
        coil::usleep(10000);
        delete buffer;
        delete consumer;
        
    }
    /*!
     * @brief pushSklip()メソッドのテスト
     * 
     */
    void test_pushSkip(void)
    {
        InPortCorbaCdrConsumerMock *consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherNewMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","skip");
        prop.setProperty("publisher.skip_count","1");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
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

            //CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
            //                     publisher.write(cdr,0,0));
            publisher.write(cdr,0,0);
            coil::usleep(10000);
        }

        //provider 側のバッファ full の状態でコール(full)
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 8;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 9;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
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

        //provider 側のバッファ full ではない状態でコール
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 10;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 11;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }

        //データ抜けががないことを確認する。
        for(int icc(0);icc<2;++icc)
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
        coil::usleep(10000);
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
        InPortCorbaCdrConsumerMock *consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherNewMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","skip");
        prop.setProperty("publisher.skip_count","1");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
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
        for(int icc(0);icc<24;++icc)
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = icc;
            td >>= cdr;

            RTC::PublisherBase::ReturnCode ret;
            ret = publisher.write(cdr,0,0);
            if(icc<18)
            {
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                     ret);
            }
            else
            {
                CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                     ret);
            }
            coil::usleep(10000);

        }

        //consumer と provider 両方の buffer が full 状態のため、
        // この weite データは抜ける。
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 24;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
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

        //consumer の buffer が full 状態のため、
        // この weite データは抜ける。
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 25;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::BUFFER_FULL,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
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
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 26;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 27;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        //データを確認する。
        {
        cdrMemoryStream data;
        data = consumer->get_m_put_data();
        CORBA::ULong inlen = data.bufSize();
        CPPUNIT_ASSERT_EQUAL(12,(int)inlen);

            RTC::TimedLong rtd;
            rtd <<= data;
            CPPUNIT_ASSERT_EQUAL((long)27, (long)rtd.data);
        }

        coil::usleep(10000);
        publisher.deactivate();
        
        coil::usleep(10000);
        delete buffer;
        delete consumer;
        
    }
    /*!
     * @brief pushNew()メソッドのテスト
     * 
     */
    void test_pushNew(void)
    {
        InPortCorbaCdrConsumerMock *consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherNewMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","new");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
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
        for(int icc(0);icc<7;++icc)
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = icc;
            td >>= cdr;

            CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));

        }

        coil::usleep(10000);
        {
            cdrMemoryStream cdr;
            RTC::TimedLong td;
            td.data = 7;
            td >>= cdr;

            CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));

        }

        coil::usleep(10000);
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

        coil::usleep(1000000);  // ここは長めにしないと落ちます。
        publisher.deactivate();
        coil::usleep(1000000);  // ここは長めにしないと落ちます。
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
        InPortCorbaCdrConsumerMock *consumer 
                                    = new InPortCorbaCdrConsumerMock();
        RTC::CdrBufferBase* buffer = new RTC::CdrRingBuffer();
        PublisherNewMock publisher;

        coil::Properties prop;
        prop.setProperty("publisher.push_policy","all");
        prop.setProperty("publisher.skip_count","0");
        prop.setProperty("thread_type","default");
        prop.setProperty("measurement.exec_time","enable");
        prop.setProperty("measurement.exec_count","0");
        prop.setProperty("measurement.period_time","enable");
        prop.setProperty("measurement.period_count","0");
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

        // consumer not set check
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 101;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PRECONDITION_NOT_MET,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        publisher.setBuffer(buffer);
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 102;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PRECONDITION_NOT_MET,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }

        // consumer set
        publisher.setConsumer(consumer);
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 103;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }
        publisher.activate();

        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 104;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
        }

        // 例外スローチェック : OK
        //consumer->set_m_mode(1);
        {
        cdrMemoryStream cdr;
        RTC::TimedLong td;
        td.data = 105;
        td >>= cdr;
        CPPUNIT_ASSERT_EQUAL(RTC::PublisherNew::PORT_OK,
                                 publisher.write(cdr,0,0));
        coil::usleep(10000);
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
        coil::usleep(10000);
        m_OnCheck = 4;  // SEND_TIMEOUT:onReceiverTimeout()
        publisher.write(cdr,0,0);
        coil::usleep(10000);
        m_OnCheck = 5;  // UNKNOWN_ERROR:onReceiverError()
        publisher.write(cdr,0,0);
        coil::usleep(10000);
        m_OnCheck = 6;  // CONNECTION_LOST:onReceiverError()
        publisher.write(cdr,0,0);
        coil::usleep(10000);

        delete consumer;
        delete buffer;

    }
    /*!
     * @brief update()メソッドのテスト
     * 
     * - 「Publisherのupdate()メソッド呼出間隔」>「Consumerのpush()メソッド処理時間」の場合に、update()呼出からpush()呼出までの時間間隔が、所定時間内に収まっているか？
     * 
     * @attention リアルタイム性が保証されているわけでもなく、仕様上も呼出までの時間を明記しているわけではないので、ここでの所定時間はテスト作成者の主観によるものに過ぎない。
     */
/*
    void test_update_large_interval()
    {
      long sleepTick = 100000; // 0.1[sec]
      long intervalTick = sleepTick * 10;
			
      MockConsumer* consumer = new MockConsumer(sleepTick);
      coil::Properties prop;
      RTC::PublisherNew publisher(consumer, prop);
			
      for (int i = 0; i < 10; i++)
	{
	  consumer->setDelayStartTime();
	  publisher.update();
	  coil::usleep(intervalTick);
	}
			
      // Consumer呼出が完了するであろうタイミングまで待つ
      coil::usleep(5000000); // 5 [sec]
			
      // update()呼出からpush()呼出までの時間間隔が、所定時間内に収まっているか？
      // （リアルタイム性が保証されているわけでもなく、仕様上も呼出までの時間を明記しているわけではないので、
      // ここでの所定時間はテスト作成者の主観によるものに過ぎない。）
      long permissibleDelay = sleepTick + 100000;
      const std::vector<long>& delayTicks = consumer->getDelayTicks();
      for (std::vector<long>::size_type i = 0; i < delayTicks.size(); i++)
	{
	  //std::cout << "delay tick = " << delayTicks[i] << std::endl;
	  CPPUNIT_ASSERT(delayTicks[i] < permissibleDelay);
	}
      publisher.release();
    }
*/		
    /*!
     * @brief update()メソッドのテスト
     * 
     * - 「Publisherのupdate()メソッド呼出間隔」<「Consumerのpush()メソッド処理時間」の場合に、update()呼出が溜ってしまうことなく、update()呼出からpush()呼出までの時間間隔が、所定時間内に収まっているか？
     * 
     * @attention リアルタイム性が保証されているわけでもなく、仕様上も呼出までの時間を明記しているわけではないので、ここでの所定時間はテスト作成者の主観によるものに過ぎない。
     */
/*
    void test_update_small_interval()
    {
      long sleepTick = 100000; // 0.1[sec]
      long intervalTick = sleepTick / 10;
			
      MockConsumer* consumer = new MockConsumer(sleepTick);
      coil::Properties prop;
      RTC::PublisherNew publisher(consumer, prop);
			
      for (int i = 0; i < 1000; i++)
	{
	  consumer->setDelayStartTime();
	  publisher.update();
	  coil::usleep(intervalTick);
	}
			
      // Consumer呼出が完了するであろうタイミングまで待つ
      coil::usleep(5000000); // 5 [sec]
			
      // update()呼出からpush()呼出までの時間間隔が、所定時間内に収まっているか？
      // （リアルタイム性が保証されているわけでもなく、仕様上も呼出までの時間を明記しているわけではないので、
      // ここでの所定時間はテスト作成者の主観によるものに過ぎない。）
      long permissibleDelay = sleepTick + 100000;
      const std::vector<long>& delayTicks = consumer->getDelayTicks();
      for (std::vector<long>::size_type i = 0; i < delayTicks.size(); i++)
	{
	  //std::cout << "delay tick = " << delayTicks[i] << std::endl;
	  CPPUNIT_ASSERT(delayTicks[i] < permissibleDelay);
	}
      publisher.release();
    }
*/
		
    /*!
     * @brief release()メソッドのテスト
     * 
     * - release()メソッド呼出によりPublisherの動作を確実に停止できるか？
     */
/*
    void test_release()
    {
      MockConsumer* consumer = new MockConsumer(1000000); // 1 [sec]
      coil::Properties prop;
      RTC::PublisherNew publisher(consumer, prop);
			
      // update()を呼出して、Consumerを呼び出させる
      publisher.update();
      coil::usleep(3000000); // Consumerを呼出す時間を与える
      publisher.release();
			
      CPPUNIT_ASSERT_EQUAL(1, consumer->getCount());
			
      // 再度update()を呼出し、Consumerを呼出しうる時間を与える。
      // （実際には、前段のrelease()によりPublisherが停止済みであり、
      // update()呼出は何ら影響を与えないことを予期している。）
      publisher.update();
      coil::usleep(3000000);

      // Consumer呼出回数が変わっていないこと、つまりPublisherの動作が停止していることを確認する
      CPPUNIT_ASSERT_EQUAL(1, consumer->getCount());
    }
*/
		
  };
}; // namespace PublisherNew

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(PublisherNew::PublisherNewTests);

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
#endif // PublisherNew_cpp
