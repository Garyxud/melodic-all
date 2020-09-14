// -*- C++ -*-
/*!
 * @file   TaskTests.cpp
 * @brief  Task test class
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

#ifndef Task_cpp
#define Task_cpp

#include <iostream>
#include <iomanip>
#include <string>
#include <stdio.h>
#include <time.h>
#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/TextOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>
#include <cppunit/TestAssert.h>

//#include <../../include/coil/Task.h>
#include <coil/Task.h>
#include <coil/Time.h>

/*!
 * @class TaskTests class
 * @brief Task test
 */
namespace Task
{
  class TaskTests
   : public CppUnit::TestFixture , 
     public coil::Task
  {
    CPPUNIT_TEST_SUITE(TaskTests);
//    CPPUNIT_TEST(test_case0);
    CPPUNIT_TEST(test_open_close);
    CPPUNIT_TEST(test_activate);
    CPPUNIT_TEST(test_activate2);
    CPPUNIT_TEST(test_wait);
    CPPUNIT_TEST(test_suspend);
    CPPUNIT_TEST(test_resume);
    CPPUNIT_TEST_SUITE_END();
  
  private:
    bool m_statflag;
    short m_tasknumber;
    short m_threadcmd;
    short m_threadcnt[256];
  public:
    coil::Task * testtk;  
    /*!
     * @brief Constructor
     */
    TaskTests()
    {
        short ic;
        for (ic=0; ic<256; ic++)
        {
            m_threadcnt[ic] = 0;
        }
        m_statflag = false;
        m_tasknumber =0;
        m_threadcmd = 0;
    }
    
    /*!
     * @brief Destructor
     */
    ~TaskTests()
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
  
    /*
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    */
    int svc(void)
    {
        short ic;
        m_statflag = true;
        switch(m_threadcmd)
        {
            case 0:
                std::cout<<"/"<<std::endl;
                m_tasknumber ++;
                for(;;)
                {  
                  if(m_statflag != true)
		  {
                    break;
		  }
		  m_threadcnt[m_tasknumber-1]++;
                }
                break;
            case 1:
                std::cout<<"/"<<std::endl;
                for(ic=0;ic<10;ic++){
                  ;;
                }
                break;
            default:
                break;
        }
        return 0;
    }
    /* test case */
    void test_case0()
    {
    }
    /*
    ---------------------------------------------------------------------------
    This function tests the Task::open function and the Task::close function.
    Check that the open function and the close function return 0.
    ---------------------------------------------------------------------------
    */
    void test_open_close()
    {
        int iret;
        iret = 1;
        iret = open(0);
        CPPUNIT_ASSERT_MESSAGE("open", (iret == 0) );

        iret = 1;
        iret = close(0);
        CPPUNIT_ASSERT_MESSAGE("close", (iret == 0) );

    }
    /*
    ---------------------------------------------------------------------------
    This function tests the Task::activate function.
    Check that the thread makes only one even if the activate function is
    called two or more times.
    ---------------------------------------------------------------------------
    */
    void test_activate()
    {
        
        time_t tmstart, tmend;
		char cstr[256];
        short ic;
        if ( m_statflag == true )
        {
            m_statflag = false;
        }
        m_threadcmd = 0;
        m_tasknumber = 0;
        //Start 10 threads. & Check that only 1 thread start.
        for (ic=0; ic<10; ic++)
        {
            //Start a thread. 
            activate();
            time(&tmstart);
            for(;;)
            {
              time(&tmend);
              if(difftime(tmend,tmstart)>=1.0)
              {
                break;
              }
	    }
            sprintf(cstr, "counter:%d ", m_tasknumber);
            //Check that a thread start.
            CPPUNIT_ASSERT_MESSAGE(cstr , (m_tasknumber == 1) );
        }
        m_statflag = false;
        wait();

    }
    /*!
     * @brief activate()
     *
     * This function tests the Task::activate function.
     */
    void test_activate2()
    {
        
        time_t tmstart, tmend;
		char cstr[256];
        short ic;
        if ( m_statflag == true )
        {
            m_statflag = false;
        }
        m_threadcmd = 0;
        m_tasknumber = 0;
        //Start 10 threads. & Check that 10 thread start.
        for (ic=0; ic<10; ic++)
        {
            //Start a thread. 
            activate();
            time(&tmstart);
            for(;;)
            {
              time(&tmend);
              if(difftime(tmend,tmstart)>=1.0)
              {
                break;
              }
	    }
            sprintf(cstr, "m_tasknumber:%d (ic+1):%d", m_tasknumber,ic+1);
            //Check that a thread start.
            CPPUNIT_ASSERT_MESSAGE(cstr , (m_tasknumber == ic+1) );
            m_statflag = false;
            wait();
        }

    }
    /*
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    */
    void test_wait()
    {
        wait(); //If Segmentation fault is not caused, it is OK.
        m_threadcmd = 1;        
        activate();
        wait();
    }
    /*
    ---------------------------------------------------------------------------
    This function tests the Task::suspend function.
    Check that the suspend function returns 0.
    ---------------------------------------------------------------------------
    */
    void test_suspend()
    {
        int iret;
        iret = 1;
        iret = suspend();
        CPPUNIT_ASSERT_MESSAGE("suspend", (iret == 0) );
    }
    /*
    ---------------------------------------------------------------------------
    This function tests the Task::resume function.
    Check that the resume function returns 0.
    ---------------------------------------------------------------------------
    */
    void test_resume()
    {
        int iret;
        iret = 1;
        iret = resume();
        CPPUNIT_ASSERT_MESSAGE("resume", (iret == 0) );
    }
  };
}; // namespace Task

/*
 * Register test suite
 */
CPPUNIT_TEST_SUITE_REGISTRATION(Task::TaskTests);

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
#endif // Task_cpp
