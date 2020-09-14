/*
 * How to compile:
 * gcc -shared -fPIC -o DummyModule.so DummyModule.cpp
 */
#ifndef DUMMYMODULE_H_
#define DUMMYMODULE_H_

namespace RTC
{

class Manager;

extern "C" void InitProc(Manager* manager);
extern "C" int getInitProcCount();
extern "C" void resetInitProcCount();

};

#endif /*DUMMYMODULE_H_*/
