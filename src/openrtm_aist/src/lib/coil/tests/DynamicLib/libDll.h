#ifndef COIL_LIBDLLTEST_H
#define COIL_LIBDLLTEST_H

#include <coil/DynamicLib.h>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define  DynamicLib_EXPORT __declspec(dllexport)
extern "C"
{
  DynamicLib_EXPORT int ForExternTest(void);
}
#else
extern "C"
{
  int ForExternTest(void);
}
#endif



namespace coil
{
  class libDllTest
  {
  public:
    libDllTest(){};
    ~libDllTest(){};
    static int FuncTest(void); 
  };
};

#endif
