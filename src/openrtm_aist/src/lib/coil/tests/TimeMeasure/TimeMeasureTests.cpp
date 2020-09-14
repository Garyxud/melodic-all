// -*- C++ -*-
/*!
 * @file   TimeMeasureTests.cpp
 * @brief  TimeMeasure test class
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

#ifndef TimeMeasure_cpp
#define TimeMeasure_cpp

#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>
#include <coil/Time.h>
#include <coil/TimeMeasure.h>
#include <math.h>
/*!
 * @class TimeMeasureTests class
 * @brief TimeMeasure test
 */
namespace TimeMeasure
{
  class TimeMeasureTests
   : public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE(TimeMeasureTests);
    CPPUNIT_TEST(test_count);
    CPPUNIT_TEST(test_stat);
    CPPUNIT_TEST(test_buflen);
    CPPUNIT_TEST(test_30ms);
    CPPUNIT_TEST(test_1s);
    CPPUNIT_TEST_SUITE_END();
  
  private:
  
  public:
  
    /*!
     * @brief Constructor
     */
    TimeMeasureTests()
    {
    }
    
    /*!
     * @brief Destructor
     */
    ~TimeMeasureTests()
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
  
    void test_count()
    {
      coil::TimeMeasure tm;

      CPPUNIT_ASSERT(tm.count() == 0);

      const unsigned long int count0(10);
      for (unsigned int i(0); i < count0; ++i)
        {
          tm.tick();
          tm.tack();
        }
      CPPUNIT_ASSERT(tm.count() ==  count0);

      tm.reset();

      CPPUNIT_ASSERT(tm.count() == 0);

      const unsigned long int count1(53);
      for (unsigned int i(0); i < count1; ++i)
        {
          tm.tick();
          tm.tack();
        }
      CPPUNIT_ASSERT(tm.count() == count1);
    }

    void test_stat()
    {
      coil::TimeMeasure tm;
      double maxi, mini, mean, stdev;
      CPPUNIT_ASSERT(tm.getStatistics(maxi, mini, mean, stdev) == false);
      tm.tick();
      CPPUNIT_ASSERT(tm.getStatistics(maxi, mini, mean, stdev) == false);
      tm.tack();
      CPPUNIT_ASSERT(tm.getStatistics(maxi, mini, mean, stdev) == true);
      tm.reset();
      CPPUNIT_ASSERT(tm.getStatistics(maxi, mini, mean, stdev) == false);
    }

    void test_buflen()
    {
      {
        coil::TimeMeasure tm0(1);
        CPPUNIT_ASSERT(tm0.count() == 0);
        tm0.tick();
        tm0.tack();
        CPPUNIT_ASSERT(tm0.count() == 1);
      }

      {
        const unsigned int count(1024);
        coil::TimeMeasure tm1(count);

        for (unsigned int i(0); i < count; ++i)
          {
            CPPUNIT_ASSERT(tm1.count() == i);
            tm1.tick();
            tm1.tack();
          }
        for (unsigned int i(0); i < count; ++i)
          {
            tm1.tick();
            tm1.tack();
            CPPUNIT_ASSERT(tm1.count() == (count + 1));
          }

      }
    }

    /* test case */
    void test_30ms()
    {
      const double wait(0.03); // [s]
      coil::TimeMeasure tm;
      for (int i(0); i < 10; ++i)
        {
          tm.tick();
          coil::usleep((int)(wait * 1000000));
          tm.tack();
        }
      double maxi, mini, mean, stdev;
      tm.getStatistics(maxi, mini, mean, stdev);
      /*
      std::cout << "max interval : " << maxi << " [sec]" << std::endl;
      std::cout << "min interval : " << mini << " [sec]" << std::endl;
      std::cout << "mean interval: " << mean << " [sec]" << std::endl;
      std::cout << "stddev       : " << stdev<< " [sec]" << std::endl;
      */
      CPPUNIT_ASSERT(maxi < (wait + 0.030));
      CPPUNIT_ASSERT(mini > (wait - 0.010));
      CPPUNIT_ASSERT(fabs(mean - wait) < 0.03);
      CPPUNIT_ASSERT(stdev < (wait / 5.0));
    }
    void test_1s()
    {
      const double wait(1.0); // [s]
      coil::TimeMeasure tm;
      for (int i(0); i < 1; ++i)
        {
          tm.tick();
          coil::sleep(1);
          tm.tack();
        }
      double maxi, mini, mean, stdev;
      tm.getStatistics(maxi, mini, mean, stdev);
      /*
      std::cout << "max interval : " << maxi << " [sec]" << std::endl;
      std::cout << "min interval : " << mini << " [sec]" << std::endl;
      std::cout << "mean interval: " << mean << " [sec]" << std::endl;
      std::cout << "stddev       : " << stdev<< " [sec]" << std::endl;
      */
      CPPUNIT_ASSERT(maxi < (wait + 0.030));
      CPPUNIT_ASSERT(mini > (wait - 0.010));
      CPPUNIT_ASSERT(fabs(mean - wait) < 0.03);
      CPPUNIT_ASSERT(stdev < (wait / 5.0));
    }
  };
}; // namespace TimeMeasure

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(TimeMeasure::TimeMeasureTests);

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
#endif // TimeMeasure_cpp
