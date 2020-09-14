/*
 * How to compile:
 * gcc -shared -fPIC -o DummyModule2.so DummyModule2.cpp
 */
#ifndef DUMMYMODULE2_H_
#define DUMMYMODULE2_H_

namespace RTC
{

class Manager;

extern "C" void InitProc(Manager* manager);
extern "C" int getInitProcCount();
extern "C" void resetInitProcCount();

};

#endif /*DUMMYMODULE2_H_*/
