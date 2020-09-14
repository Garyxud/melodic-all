// -*- C++ -*-

#include <iostream>
#include <string>

#include "libDll.h"


namespace coil
{
    /*!
     *
     */
    int libDllTest::FuncTest(void) 
      { 
        std::cout<<"ForExternTest"<<std::endl;
        return 0xdeadbeef; 
      }
};

/*!
 * for unit test.
 */
extern "C"
{
  int ForExternTest(void) 
    { 
       return coil::libDllTest::FuncTest(); 
    }
};

