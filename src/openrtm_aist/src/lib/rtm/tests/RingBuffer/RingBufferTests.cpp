// -*- C++ -*-
/*!
 * @file   RingBufferTests.cpp
 * @brief  RingBuffer test class
 * @date   $Date: 2008/01/24 01:52:14 $
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
 * $Log: RingBufferTests.cpp,v $
 * Revision 1.3  2008/01/24 01:52:14  tsakamoto
 * *** empty log message ***
 *
 * Revision 1.2  2008/01/11 11:27:11  arafune
 * *** empty log message ***
 *
 * Revision 1.1  2007/12/20 07:50:17  arafune
 * *** empty log message ***
 *
 * Revision 1.2  2006/12/02 18:53:08  n-ando
 * Some tests were added.
 *
 * Revision 1.1  2006/11/27 08:37:03  n-ando
 * TestSuites are devided into each directory.
 *
 *
 */

#ifndef RingBuffer_cpp
#define RingBuffer_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

#include <string>
#include <sstream>
#include <unistd.h>
#include <rtm/RingBuffer.h>

//#define DEBUG

#ifdef __QNX__
using std::abort;
using std::perror;
#endif

static const int NLOOP = 0x00000fff;	// 読み書きのループ回数
typedef ::RTC::BufferStatus::Enum ReturnCode;
int isBlockTest = false;
ReturnCode g_ret = RTC::BufferStatus::BUFFER_OK;

struct thread_arg
{
    thread_arg(RTC::RingBuffer<int>& buff, int loop=NLOOP)
        : m_buff(buff), m_n_loop(loop)
    {}
    RTC::RingBuffer<int>& m_buff;
    int m_n_loop;
};
// 読み込みスレッド
void* reader(void* arg)
{
  thread_arg* t_arg = (thread_arg*)arg;
  RTC::RingBuffer<int>* buff = (RTC::RingBuffer<int>*)&(t_arg->m_buff);
  
  ReturnCode ret;
  for (int i = 0; i < t_arg->m_n_loop; i++) {
	int output = -1;
	ret = (*buff).read(output);
#if PRINTOUT
	std::cout << "read() : " << output << std::endl;
#endif
	if (ret == RTC::BufferStatus::TIMEOUT) {
	  g_ret = ret;
	  break;
	}
	if (isBlockTest) {
	  if (output != i) {
		std::cout << "ERROR!!: output=" << output;
		std::cout << ", correct data=" << i;
		std::cout << ", return code=" << ret << std::endl;
		abort();
	  }
	}
  }
  return NULL;
}

// 書き込みスレッド
void* writer(void* arg)
{
  thread_arg* t_arg = (thread_arg*)arg;
  RTC::RingBuffer<int>* buff = (RTC::RingBuffer<int>*)&(t_arg->m_buff);
  
  
  for (int i = 0; i < t_arg->m_n_loop; i++) {
	int input = i;
	(*buff).write(input);
#if PRINTOUT
	std::cout << "write(" << input << ")" << std::endl;
#endif
  }
  return NULL;
}


namespace RingBuffer
{
  /*!
   * @class RingBufferTests class
   * @brief RingBuffer test
   */
  class RingBufferTests
    : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(RingBufferTests);

    CPPUNIT_TEST(test_length);
    CPPUNIT_TEST(test_isFull);
    CPPUNIT_TEST(test_isEmpty);
    CPPUNIT_TEST(test_init);
    CPPUNIT_TEST(test_write_read);
    CPPUNIT_TEST(test_write_read_with_small_length);
    CPPUNIT_TEST(test_isNew);
    CPPUNIT_TEST(test_reset);
    CPPUNIT_TEST(test_wptr_put);
    CPPUNIT_TEST(test_advanceWptr);
    CPPUNIT_TEST(test_rptr_get);
    CPPUNIT_TEST(test_advanceRptr);

	CPPUNIT_TEST(test_owrite_rback); // full: overwrite, empty: readback, buff length 1,8
	CPPUNIT_TEST(test_owrite_dnothing); // full: overwrite, empty: do_nothing, buff length 1,8
	CPPUNIT_TEST(test_owrite_block); // full: overwrite, empty: block, buff length 1,8

	CPPUNIT_TEST(test_dnothing_rback); // full: do_nothing, empty: readback, buff length 1,8
	CPPUNIT_TEST(test_dnothing_dnothing); // full: do_nothing, empty: do_nothing, buff length 1,8
	CPPUNIT_TEST(test_dnothing_block); // full: do_nothing, empty: block, buff length 1,8
	CPPUNIT_TEST(test_block_rback); // full: block, empty: readback, buff length 1,8
	CPPUNIT_TEST(test_block_dnothing); // full: block, empty: do_nothing, buff length 1,8
	CPPUNIT_TEST(test_block_block_wr); // full: block, empty: block, buff length 1,8

	CPPUNIT_TEST(test_block_block_rw); // full: block, empty: block, read -> write

    CPPUNIT_TEST_SUITE_END();
		
  private:
	void do_test(RTC::RingBuffer<int>& buff, int loop=NLOOP, bool read_first=false)
	{
	  pthread_t tr, tw;
          thread_arg arg(buff,loop);

	  if (read_first) {
		// 読み込みスレッドの開始
		if (pthread_create(&tr, NULL , reader, static_cast<void *>(&buff)) != 0) {
		  perror("pthread_create(r)");
		  return;
		}
		// 書き込みスレッドの開始
		if (pthread_create(&tw, NULL , writer, static_cast<void *>(&buff)) != 0) {
		  perror("pthread_create(w)");
		  return;
		}
	  }
	  else {
		// 書き込みスレッドの開始
//		if (pthread_create(&tw, NULL , writer, static_cast<void *>(&buff)) != 0) {
		if (pthread_create(&tw, NULL , writer, &arg) != 0) {
		  perror("pthread_create(w)");
		  return;
		}
		// 読み込みスレッドの開始
//		if (pthread_create(&tr, NULL , reader, static_cast<void *>(&buff)) != 0) {
		if (pthread_create(&tr, NULL , reader, &arg) != 0) {
		  perror("pthread_create(r)");
		  return;
		}
	  }
	  
	  // スレッドの合流
	  pthread_join(tw, NULL);
	  pthread_join(tr, NULL);
	}
		
  public:
    /*!
     * @brief Constructor
     */
    RingBufferTests()
    {
    }
		
    /*!
     * @brief Destructor
     */
    ~RingBufferTests()
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
     * @brief length()メソッドのテスト
     * 
     * - コンストラクタで指定されたバッファ長が正しく取得できるか？
     */
    void test_length()
    {
      RTC::RingBuffer<int> buff(123);
      CPPUNIT_ASSERT(buff.length() == 123);

      size_t len(456);
      buff.length(len);
      CPPUNIT_ASSERT(buff.length() == len);
    }

    /*!
     * @brief isEmpty()メソッドのテスト
     * 
     * - バッファ初期化直後、空ではないと判定されるか？
     * - 最後にデータが読み取られた後、新しいデータが書き込みされていない場合、
     *   空と判定されるか？
     * - 最後にデータが読み取られた後、新しいデータが書き込みされた場合、
     *   空ではないと判定されるか？
     */
    void test_isEmpty()
    {
      long int length = 10;
      RTC::RingBuffer<int> buff(length);
			
      // (1) バッファ初期化直後、空ではないと判定されるか？
      int initialValue = 12345;
      for (unsigned int i(0); i < buff.length(); ++i)
        {
          buff.write(initialValue);
        }
      CPPUNIT_ASSERT_EQUAL(false, buff.empty());			
			
      // (2) 最後にデータが読み取られた後、新しいデータが書き込みされて
      // いない場合、空ではないと判定されるか？
      int readValue;
      CPPUNIT_ASSERT_EQUAL(::RTC::BufferStatus::BUFFER_OK, buff.read(readValue));
      CPPUNIT_ASSERT_EQUAL(false, buff.empty());
			
      // (3) 最後にデータが読み取られた後、新しいデータが書き込みされた
      // 場合、空ではないと判定されるか？
      int writeValue = 98765;
      CPPUNIT_ASSERT_EQUAL(RTC::BufferBase<int>::BUFFER_OK, buff.write(writeValue));
      CPPUNIT_ASSERT_EQUAL(false, buff.empty());
    }
		
		
    /*!
     * @brief isFull()メソッドのテスト
     * 
     * - バッファが空の場合、フル判定は偽となるか？
     * - 全バッファにデータが書き込まれている状態で、フル判定は真となるか？
     * - バッファに幾分データが書き込まれている状態で、フル判定は偽となるか？
     */
    void test_isFull()
    {
      // (1) バッファが空の場合、フル判定は偽となるか？
      int length1 = 10;
      RTC::RingBuffer<int> buff1(length1);
      CPPUNIT_ASSERT_EQUAL(false, buff1.full());
			
      // (2) 全バッファにデータが書き込まれている状態で、フル判定は真
      // となるか？
      int length2 = 10;
      RTC::RingBuffer<int> buff2(length2);
      for (int i = 0; i < length2; i++) {
	buff2.write(i);
      }
      CPPUNIT_ASSERT_EQUAL(true, buff2.full());
			
      // (3) バッファに幾分データが書き込まれている状態で、フル判定は偽
      // となるか？
      int length3 = 10;
      RTC::RingBuffer<int> buff3(length3);
      for (int i = 0; i < length3 / 2; i++) {
	buff3.write(i);
      }
      CPPUNIT_ASSERT_EQUAL(false, buff3.full());
    }
				
    /*!
     * @brief init()メソッドのテスト
     * 
     * - あらかじめデータで初期化した後、設定したデータを正しく読み出せるか？
     */
    void test_init()
    {
      // バッファを作成して、init()で初期化する
      long int length = 10;
      RTC::RingBuffer<int> buff(length);
			
      int value = 12345;
      for (unsigned int i(0); i < buff.length(); ++i)
        {
          buff.write(value);
        }
			
      // 設定したデータを正しく読み出せるか？
      int expected = 12345;
      for (long int i = 0; i < length; i++) {
	int actual;
	buff.read(actual);
	CPPUNIT_ASSERT_EQUAL(expected, actual);
      }
    }

    /*!
     * @brief write()メソッドおよびread()メソッドのテスト
     * 
     * - バッファ空状態で１データ書込・読出を行い、書き込んだデータを正しく
     *   読み出せるか？
     * - 全バッファにデータが書き込まれている状態で１データ書込・読出を行い、
     *   書き込んだデータを正しく読み出せるか？
     * - 全バッファに幾分データが書き込まれている状態で１データ書込・読出を
     *   行い、書き込んだデータを正しく読み出せるか？
     */
    void test_write_read()
    {
      // (1) バッファ空状態で１データ書込・読出を行い、書き込んだデータ
      // を正しく読み出せるか？バッファ作成し、空のままにする
      long int length1 = 3;
      RTC::RingBuffer<int> buff1(length1);
      coil::Properties prop;
      prop["write.full_policy"] = "block";
      prop["write.timeout"] = "5.0";
      prop["read.empty_policy"] = "block";
      prop["read.timeout"] = "5.0";

      buff1.init(prop);
      // １データ書込・読出を行う
      for (int writeValue = 0; writeValue < 100; writeValue++) {
	// 書込み
        if (buff1.full())
          {
            std::cout << "### FULL ###" << std::endl;
            int readValue;

            if (writeValue % 5 == 0)
              {
                while (!buff1.empty())
                  {
                    std::cout << "read timeout: " << 5 << std::endl;
                    buff1.read(readValue, 5);
                    std::cout << "read: " << readValue << std::endl;
                  }
                std::cout << "read timeout: " << 5 << std::endl;
                std::cout << "waiting 5 sec" << std::endl;
                std::cout << "read ret: " << RTC::BufferStatus::toString(buff1.read(readValue, 5)) << std::endl;
                std::cout << "read: " << readValue << std::endl;
              }
            else
              {
                buff1.read(readValue);
                std::cout << "read: " << readValue << std::endl;
              }

            if (buff1.full())
              {
                std::cout << "??? still full" << std::endl;
              }
            else
              {
                std::cout << "buffer full was blown over." << std::endl;
              }
          }
        if (buff1.empty())
          {
            std::cout << "### EMPTY ###" << std::endl;
          }

        //        std::cout << "timeout-> " << writeValue << std::endl;
        std::cout << "write ret: " << RTC::BufferStatus::toString(buff1.write(writeValue, writeValue)) << std::endl;
				
        //        std::cout << "readable " << buff1.readable() << std::endl;
        //        std::cout << "writable " << buff1.writable() << std::endl;

	// 読出し
	int readValue;
	buff1.get(readValue);
			
        std::cout << writeValue << " == " << readValue << std::endl;

        //        buff1.read(readValue)
	// 書き込んだデータを正しく読み出せたか？
        //	CPPUNIT_ASSERT_EQUAL(writeValue, readValue);
        sleep(1);
      }
      return;
      // (2) 全バッファにデータが書き込まれている状態で１データ書込・読
      // 出を行い、書き込んだデータを正しく読み出せるか？バッファ作成し、
      // フル状態にする
      long int length2 = 10;
      RTC::RingBuffer<int> buff2(length2);
      for (int i = 0; i < length2; i++) {
	buff2.write(i + 123);
      }
			
      // １データ書込・読出を行う
      for (int writeValue = 0; writeValue < 100; writeValue++) {
	// 書込み
	buff2.write(writeValue);
				
	// 読出し
	int readValue;
	buff2.read(readValue);
				
	// 書き込んだデータを正しく読み出せたか？
	CPPUNIT_ASSERT_EQUAL(writeValue, readValue);
      }
			
      // (3) バッファに幾分データが書き込まれている状態で１データ書込・
      // 読出を行い、書き込んだデータを正しく読み出せるか？
      long int length3 = 10;
      RTC::RingBuffer<int> buff3(length3);
      for (int i = 0; i < length3 / 2; i++) {
	buff3.write(i + 123);
      }
			
      // １データ書込・読出を行う
      for (int writeValue = 0; writeValue < 100; writeValue++) {
	// 書込み
	buff3.write(writeValue);
				
	// 読出し
	int readValue;
	buff3.read(readValue);
				
	// 書き込んだデータを正しく読み出せたか？
	CPPUNIT_ASSERT_EQUAL(writeValue, readValue);
      }
    }
		
    /*!
     * @brief write()メソッドおよびread()メソッドのテスト（バッファ長２の場合）
     * 
     * - バッファ空状態で１データ書込・読出を行い、書き込んだデータを正しく
     *   読み出せるか？
     * - 全バッファにデータが書き込まれている状態で１データ書込・読出を行い、
     *   書き込んだデータを正しく読み出せるか？
     * - バッファに幾分データが書き込まれている状態で１データ書込・読出を行い、
     *   書き込んだデータを正しく読み出せるか？
     */
    void test_write_read_with_small_length()
    {
      // (1) バッファ空状態で１データ書込・読出を行い、書き込んだデータ
      // を正しく読み出せるか？バッファ作成し、空のままにする
      long int length1 = 2;
      RTC::RingBuffer<int> buff1(length1);
			
      // １データ書込・読出を行う
      for (int writeValue = 0; writeValue < 100; writeValue++) {
	// 書込み
	buff1.write(writeValue);
				
	// 読出し
	int readValue;
	buff1.read(readValue);
				
	// 書き込んだデータを正しく読み出せたか？
	CPPUNIT_ASSERT_EQUAL(writeValue, readValue);
      }
			
      // (2) 全バッファにデータが書き込まれている状態で１データ書込・読
      // 出を行い、書き込んだデータを正しく読み出せるか？バッファ作成し、
      // フル状態にする
      long int length2 = 2;
      RTC::RingBuffer<int> buff2(length2);
      coil::Properties prop;
      prop["write.full_policy"] = "overwrite";
      buff2.init(prop);

      for (int i = 0; i < length2; i++) {
	buff2.write(i + 123);
      }
			
      // １データ書込・読出を行う
//      for (int writeValue = 0; writeValue < 100; writeValue++) {
//	// 書込み
//	buff2.write(writeValue);
//				
//	// 読出し
//	int readValue;
//	buff2.read(readValue);
//				
//	// 書き込んだデータを正しく読み出せたか？
//	CPPUNIT_ASSERT_EQUAL(writeValue+1+123, readValue);
//     }

	// 書込み
        /* policyがoverwriteでfull状態で書き込むと、古いデータを上書きし、読み込み側のポインタをインクリメントする。*/
	buff2.write(0);
				
        CPPUNIT_ASSERT_EQUAL(true, buff2.full());
	// 読出し
	int readValue;
        /* 読み出して読み込み側のポインタをインクメントするためfull状態ではなくなる */
	buff2.read(readValue);

        CPPUNIT_ASSERT_EQUAL(false, buff2.full());
	// 書き込んだデータを正しく読み出せたか？
	CPPUNIT_ASSERT_EQUAL(1+123, readValue);
			
      // (3) バッファに幾分データが書き込まれている状態で１データ書込・読出を行い、書き込んだデータを正しく読み出せるか？
      long int length3 = 2;
      RTC::RingBuffer<int> buff3(length3);
      coil::Properties prop3;
      prop3["write.full_policy"] = "overwrite";
      buff3.init(prop3);

      for (int i = 0; i < 1; i++) {
	buff3.write(i + 123);
      }
      {	
	// 書込み
	buff3.write(-1);
				
	// 読出し
	int readValue;
	buff3.read(readValue);
				
	// 書き込んだデータを正しく読み出せたか？
	CPPUNIT_ASSERT_EQUAL(123, readValue);
      }
      // １データ書込・読出を行う
      for (int writeValue = 0; writeValue < 100; writeValue++) {
	// 書込み
	buff3.write(writeValue);
				
	// 読出し
	int readValue;
	buff3.read(readValue);
				
	// 書き込んだデータを正しく読み出せたか？
	CPPUNIT_ASSERT_EQUAL(writeValue-1, readValue);
      }
    }
		
    /*!
     * @brief isNew()メソッドのテスト
     * 
     * - バッファが空の状態で、isNew判定が偽になるか？
     * - 全バッファにデータが書き込まれている状態で、
     *   データ書込後のisNew判定が真になるか？
     * - 全バッファにデータが書き込まれている状態で、データ書込し、
     *   そのデータ読出を行った後のisNew判定が偽になるか？
     * - バッファに幾分データが書き込まれている状態で、データ書込後のisNew
     *   判定が真になるか？
     * - バッファに幾分データが書き込まれている状態で、データ書込し、
     *   そのデータ読出を行った後のisNew判定が偽になるか？
     */
    void test_isNew()
    {
      // (1) バッファが空の状態で、isNew判定が偽になるか？
      long int length1 = 10;
      RTC::RingBuffer<int> buff1(length1);
      CPPUNIT_ASSERT_EQUAL(true, buff1.empty());
			
      // 全バッファにデータが書き込まれている状態で...
      long int length2 = 10;
      RTC::RingBuffer<int> buff2(length2);
			
      for (int i = 0; i < length2; i++) {
	// (2) ...データ書込後のisNew判定が真になるか？
	int writeValue = i + 123;
	buff2.write(writeValue);
	CPPUNIT_ASSERT_EQUAL(false, buff2.empty());
				
	// (3) ...データ書込し、そのデータ読出を行った後のisNew判定が偽
	// になるか？
	int readValue;
	buff2.read(readValue);
	CPPUNIT_ASSERT_EQUAL(true, buff2.empty());
      }
			
      // バッファに幾分データが書き込まれている状態で...
      long int length3 = 10;
      RTC::RingBuffer<int> buff3(length3);
      for (int i = 0; i < length3 / 2; i++) {
	buff3.write(i + 456);
      }
			
      for (int i = 0; i < length3; i++) {
	// (4) ...データ書込後のisNew判定が真になるか？
	int writeValue = i + 123;
	buff3.write(writeValue);
	CPPUNIT_ASSERT_EQUAL(false, buff3.empty());
				
	// (5) ...データ書込し、そのデータ読出を行った後のisNew判定が偽
	// になるか？
	int readValue;
	buff3.read(readValue);
	CPPUNIT_ASSERT_EQUAL(false, buff3.empty());
      }
    }
    /*!
     * @brief reset()メソッドのテスト
     * 
     */
    void test_reset()
    {
      int idata[10] = {123,456,789,321,654,987,1234,3456,5678,7890};
      RTC::RingBuffer<int> buff(10);

      for(int ic(0);ic<8;++ic)
      {
          buff.put(idata[ic]);
          buff.advanceWptr();
      }
      buff.advanceRptr(3);
      CPPUNIT_ASSERT_EQUAL(buff.get(), idata[3]);
      CPPUNIT_ASSERT_EQUAL(buff.readable(), (size_t)5);

      buff.reset();
      CPPUNIT_ASSERT(buff.empty());
      CPPUNIT_ASSERT_EQUAL(buff.get(), idata[0]);
      buff.put(idata[9]);
      CPPUNIT_ASSERT_EQUAL(buff.get(), idata[9]);
      CPPUNIT_ASSERT_EQUAL(buff.readable(), (size_t)0);
      
    }	
    /*!
     * @brief wptr(),put()メソッドのテスト
     * 
     */
    void test_wptr_put()
    {
      int idata[10] = {123,456,789,321,654,987,1234,3456,5678,7890};
      RTC::RingBuffer<int> buff(10);
      for(int ic(0);ic<10;++ic)
      {
          buff.put(idata[ic]);
          buff.advanceWptr();
      }
      buff.reset();
      for(int ic(0);ic<10;++ic)
      {
          CPPUNIT_ASSERT_EQUAL(idata[ic],*buff.wptr(ic));
      }
      for(int ic(0);ic<10;++ic)
      {
          CPPUNIT_ASSERT_EQUAL(idata[(-ic+10)%10],*buff.wptr(-ic));
      }
      buff.advanceWptr(5);
      for(int ic(0);ic<10;++ic)
      {
          CPPUNIT_ASSERT_EQUAL(idata[(5+ic)%10],*buff.wptr(ic));
      }
      for(int ic(0);ic<10;++ic)
      {
          CPPUNIT_ASSERT_EQUAL(idata[(5-ic+10)%10],*buff.wptr(-ic));
      }
        
    }
    /*!
     * @brief advanceWptr()メソッドのテスト
     * 
     */
    void test_advanceWptr()
    {
        int idata[10] = {123,456,789,321,654,987,1234,3456,5678,7890};
        RTC::RingBuffer<int> buff(10);
        CPPUNIT_ASSERT_EQUAL(buff.advanceWptr(-5),::RTC::BufferStatus::PRECONDITION_NOT_MET);
        CPPUNIT_ASSERT_EQUAL(buff.advanceWptr(5),::RTC::BufferStatus::BUFFER_OK);
        CPPUNIT_ASSERT_EQUAL(buff.advanceWptr(8),::RTC::BufferStatus::PRECONDITION_NOT_MET);
        CPPUNIT_ASSERT_EQUAL(buff.advanceWptr(-5),::RTC::BufferStatus::BUFFER_OK);
        buff.reset();
        for(int ic(0);ic<10;++ic)
        {
            buff.put(idata[ic]);
            buff.advanceWptr();
        }
        buff.reset();
        CPPUNIT_ASSERT_EQUAL(buff.advanceWptr(5),::RTC::BufferStatus::BUFFER_OK);
        buff.advanceRptr(5);
        CPPUNIT_ASSERT_EQUAL(buff.advanceWptr(-5),::RTC::BufferStatus::PRECONDITION_NOT_MET);
        CPPUNIT_ASSERT_EQUAL(buff.advanceWptr(8),::RTC::BufferStatus::BUFFER_OK);
        CPPUNIT_ASSERT_EQUAL(idata[3],*buff.wptr());
        CPPUNIT_ASSERT_EQUAL((size_t)8,buff.readable());
        CPPUNIT_ASSERT_EQUAL(buff.advanceWptr(-5),::RTC::BufferStatus::BUFFER_OK);
        CPPUNIT_ASSERT_EQUAL(idata[8],*buff.wptr());
        CPPUNIT_ASSERT_EQUAL((size_t)3,buff.readable());
    }
    /*!
     * @brief rptr(),get()メソッドのテスト
     * 
     */
    void test_rptr_get()
    {
        int idata[10] = {123,456,789,321,654,987,1234,3456,5678,7890};
        RTC::RingBuffer<int> buff(10);
        for(int ic(0);ic<10;++ic)
        {  
            buff.put(idata[ic]);
            buff.advanceWptr();
        }
        buff.reset();
        for(int ic(0);ic<10;++ic)
        {
            CPPUNIT_ASSERT_EQUAL(idata[ic],*buff.rptr(ic));
        }
        for(int ic(0);ic<10;++ic)
        { 
            CPPUNIT_ASSERT_EQUAL(idata[(-ic+10)%10],*buff.rptr(-ic));
        }
        buff.advanceWptr(5);
        buff.advanceRptr(5);
        for(int ic(0);ic<10;++ic)
        {
            CPPUNIT_ASSERT_EQUAL(idata[(5+ic)%10],*buff.rptr(ic));
        }
        for(int ic(0);ic<10;++ic)
        {
            CPPUNIT_ASSERT_EQUAL(idata[(5-ic+10)%10],*buff.rptr(-ic));
        }
        buff.reset();
        buff.advanceWptr(10);
        for(int ic(0);ic<10;++ic)
        {
            CPPUNIT_ASSERT_EQUAL(idata[ic],buff.get());
            int ret;
            buff.get(ret);
            CPPUNIT_ASSERT_EQUAL(idata[ic],ret);
            buff.advanceRptr();
        }
    }
    /*!
     * @brief advanceRptr()メソッドのテスト
     * 
     */
    void test_advanceRptr()
    {
        int idata[10] = {123,456,789,321,654,987,1234,3456,5678,7890};
        RTC::RingBuffer<int> buff(10);
        buff.advanceWptr(5);
        CPPUNIT_ASSERT_EQUAL(buff.advanceRptr(-6),::RTC::BufferStatus::PRECONDITION_NOT_MET);
        CPPUNIT_ASSERT_EQUAL(buff.advanceRptr(5),::RTC::BufferStatus::BUFFER_OK);
        CPPUNIT_ASSERT_EQUAL(buff.advanceRptr(8),::RTC::BufferStatus::PRECONDITION_NOT_MET);
        CPPUNIT_ASSERT_EQUAL(buff.advanceRptr(-5),::RTC::BufferStatus::BUFFER_OK);
        buff.reset();
        buff.advanceWptr(5);
        buff.advanceRptr(5);
        for(int ic(0);ic<10;++ic)
        {
            buff.put(idata[ic]);
            buff.advanceWptr();
        }
        CPPUNIT_ASSERT_EQUAL(buff.advanceRptr(-6),::RTC::BufferStatus::PRECONDITION_NOT_MET);
        CPPUNIT_ASSERT_EQUAL(buff.advanceRptr(8),::RTC::BufferStatus::BUFFER_OK);
        CPPUNIT_ASSERT_EQUAL(idata[8],*buff.rptr());
        CPPUNIT_ASSERT_EQUAL((size_t)8,buff.writable());
        CPPUNIT_ASSERT_EQUAL(buff.advanceRptr(-5),::RTC::BufferStatus::BUFFER_OK);
        CPPUNIT_ASSERT_EQUAL(idata[3],*buff.rptr());
        CPPUNIT_ASSERT_EQUAL((size_t)3,buff.writable());
    }

	/*
	 * デッドロックの検証1
	 *
	 * バッファサイズ: 1, 8
	 * write.full_policy: overwrite
	 * read.empty_policy: readback
	 *
	 */
	void test_owrite_rback()
	{
	  RTC::RingBuffer<int> buff(1);

	  // デフォルト設定以外のバッファ
	  coil::Properties prop;
	  prop["write.full_policy"] = "overwrite";
	  prop["read.empty_policy"] = "readback";
	  buff.init(prop);

	  g_ret = RTC::BufferStatus::BUFFER_OK;
	  do_test(buff);

	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;

	  RTC::RingBuffer<int> buff2(8);
	  buff2.init(prop);

	  do_test(buff2);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;

	  std::cout << std::flush;
	  return;
	}


	/*
	 * デッドロックの検証2
	 *
	 * バッファサイズ: 1, 8
	 * write.full_policy: overwrite
	 * read.empty_policy: do_nothing
	 *
	 */
	void test_owrite_dnothing()
	{
	  RTC::RingBuffer<int> buff(1);

	  // デフォルト設定以外のバッファ
	  coil::Properties prop;
	  prop["write.full_policy"] = "overwrite";
	  prop["read.empty_policy"] = "do_nothing";
	  buff.init(prop);

	  g_ret = RTC::BufferStatus::BUFFER_OK;
	  do_test(buff);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  RTC::RingBuffer<int> buff2(8);
	  buff2.init(prop);

	  do_test(buff2);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;

	  std::cout << std::flush;
	  return;
	}


	/*
	 * デッドロックの検証3
	 *
	 * バッファサイズ: 1, 8
	 * write.full_policy: overwrite
	 * read.empty_policy: block
	 * read.timeout: 3.0
	 *
	 */
	void test_owrite_block()
	{
	  RTC::RingBuffer<int> buff(1);

	  // デフォルト設定以外のバッファ
	  coil::Properties prop;
	  prop["write.full_policy"] = "overwrite";
	  prop["read.empty_policy"] = "block";
	  prop["read.timeout"]      = "3.0";
	  buff.init(prop);

	  g_ret = RTC::BufferStatus::BUFFER_OK;
	  do_test(buff);
	  // CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  RTC::RingBuffer<int> buff2(8);
	  buff2.init(prop);
	  do_test(buff2);
	  // CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  std::cout << std::flush;
	  return;
	}
		

	/*
	 * デッドロックの検証4
	 *
	 * バッファサイズ: 1, 8
	 * write.full_policy: do_nothing
	 * read.empty_policy: readback
	 *
	 */
	void test_dnothing_rback()
	{
	  RTC::RingBuffer<int> buff(1);

	  // デフォルト設定以外のバッファ
	  coil::Properties prop;
	  prop["write.full_policy"] = "do_nothing";
	  prop["read.empty_policy"] = "readback";
	  buff.init(prop);

	  g_ret = RTC::BufferStatus::BUFFER_OK;
	  do_test(buff);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  RTC::RingBuffer<int> buff2(8);
	  buff2.init(prop);
	  do_test(buff2);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  std::cout << std::flush;
	  return;
	}


	/*
	 * デッドロックの検証5
	 *
	 * バッファサイズ: 1, 8
	 * write.full_policy: do_nothing
	 * read.empty_policy: do_nothing
	 *
	 */
	void test_dnothing_dnothing()
	{
	  RTC::RingBuffer<int> buff(1);

	  // デフォルト設定以外のバッファ
	  coil::Properties prop;
	  prop["write.full_policy"] = "do_nothing";
	  prop["read.empty_policy"] = "do_nothing";
	  buff.init(prop);

	  g_ret = RTC::BufferStatus::BUFFER_OK;
	  do_test(buff);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  RTC::RingBuffer<int> buff2(8);
	  buff2.init(prop);
	  do_test(buff2);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  std::cout << std::flush;
	  return;
	}


	/*
	 * デッドロックの検証6
	 *
	 * バッファサイズ: 1, 8
	 * write.full_policy: do_nothing
	 * read.empty_policy: block
	 * read.timeout: 1.0
	 *
	 */
	void test_dnothing_block()
	{
	  RTC::RingBuffer<int> buff(1);

	  // デフォルト設定以外のバッファ
	  coil::Properties prop;
	  prop["write.full_policy"] = "do_nothing";
	  prop["read.empty_policy"] = "block";
	  prop["read.timeout"]      = "1.0";
	  buff.init(prop);

	  g_ret = RTC::BufferStatus::BUFFER_OK;
	  do_test(buff);
	  // CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  RTC::RingBuffer<int> buff2(8);
	  buff2.init(prop);
	  do_test(buff2);
	  // CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  std::cout << std::flush;
	  return;
	}


	/*
	 * デッドロックの検証7
	 *
	 * バッファサイズ: 1, 8
	 * write.full_policy: block
	 * read.empty_policy: readback
	 * write.timeout: 1.0
	 *
	 */
	void test_block_rback()
	{
	  RTC::RingBuffer<int> buff(1);

	  // デフォルト設定以外のバッファ
	  coil::Properties prop;
	  prop["write.full_policy"] = "block";
	  prop["read.empty_policy"] = "readback";
	  prop["write.timeout"]      = "1.0";
	  buff.init(prop);

	  g_ret = RTC::BufferStatus::BUFFER_OK;
	  do_test(buff,100);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  RTC::RingBuffer<int> buff2(8);
	  buff2.init(prop);
	  do_test(buff2,100);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  std::cout << std::flush;
	  return;
	}


	/*
	 * デッドロックの検証8
	 *
	 * バッファサイズ: 1, 8
	 * write.full_policy: block
	 * read.empty_policy: do_nothing
	 * write.timeout: 1.0
	 *
	 */
	void test_block_dnothing()
	{
	  RTC::RingBuffer<int> buff(1);

	  // デフォルト設定以外のバッファ
	  coil::Properties prop;
	  prop["write.full_policy"] = "block";
	  prop["read.empty_policy"] = "do_nothing";
	  prop["write.timeout"]      = "1.0";
	  buff.init(prop);

	  g_ret = RTC::BufferStatus::BUFFER_OK;
	  do_test(buff,100);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  RTC::RingBuffer<int> buff2(8);
	  buff2.init(prop);
	  do_test(buff2,100);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  std::cout << std::flush;
	  return;
	}


	/*
	 * デッドロックの検証9
	 *
	 * バッファサイズ: 1, 8
	 * write.full_policy: block
	 * read.empty_policy: block
	 * write.timeout: 3.0
	 * read.timeout: 3.0
	 *
	 */
	void test_block_block_wr()
	{
	  RTC::RingBuffer<int> buff(1);

	  // デフォルト設定以外のバッファ
	  coil::Properties prop;
	  prop["write.full_policy"] = "block";
	  prop["read.empty_policy"] = "block";
	  prop["write.timeout"]     = "3.0";
	  prop["read.timeout"]      = "3.0";
	  buff.init(prop);

	  isBlockTest = true;
	  g_ret = RTC::BufferStatus::BUFFER_OK;
	  do_test(buff);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  RTC::RingBuffer<int> buff2(8);
	  buff2.init(prop);
	  do_test(buff2);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  isBlockTest = false;
	  std::cout << std::flush;
	  return;
	}


	/*
	 * デッドロックの検証10 read -> write
	 *
	 * バッファサイズ: 1, 8
	 * write.full_policy: block
	 * read.empty_policy: block
	 * write.timeout: 3.0
	 * read.timeout: 3.0
	 *
	 */
	void test_block_block_rw()
	{
	  RTC::RingBuffer<int> buff(1);

	  // デフォルト設定以外のバッファ
	  coil::Properties prop;
	  prop["write.full_policy"] = "block";
	  prop["read.empty_policy"] = "block";
	  prop["write.timeout"]     = "3.0";
	  prop["read.timeout"]      = "3.0";
	  buff.init(prop);

	  isBlockTest = true;
	  g_ret = RTC::BufferStatus::BUFFER_OK;
	  do_test(buff,true);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  RTC::RingBuffer<int> buff2(8);
	  buff2.init(prop);
	  do_test(buff2,true);
	  CPPUNIT_ASSERT(g_ret == RTC::BufferStatus::BUFFER_OK);
	  g_ret = RTC::BufferStatus::BUFFER_OK;


	  isBlockTest = false;
	  std::cout << std::flush;
	  return;
	}



  };
}; // namespace RingBuffer

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(RingBuffer::RingBufferTests);

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
#endif // RingBuffer_cpp
