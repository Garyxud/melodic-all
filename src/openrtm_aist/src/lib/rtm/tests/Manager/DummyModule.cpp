#include <rtm/DataFlowComponentBase.h>
#include "DummyModule.h"

namespace RTC
{

class Manager;

class DummyModule
  : public RTC::DataFlowComponentBase
{
public:
        DummyModule(RTC::Manager* manager)
            : RTC::DataFlowComponentBase(manager)
        {
        };
        ~DummyModule(){};
 
	static void InitProc(Manager* manager) { m_counter++; }
	static int getInitProcCount() { return m_counter; }
	static void resetInitProcCount() { m_counter = 0; }
	
private:
	static int m_counter;
};
int DummyModule::m_counter = 0;

void InitProc(Manager* manager)
{
	DummyModule::InitProc(manager);
}

int getInitProcCount()
{
	return DummyModule::getInitProcCount();
}

void resetInitProcCount()
{
	DummyModule::resetInitProcCount();
}

};

static const char* dummy_spec[] =
  {
    "implementation_id", "Dummy",
    "type_name",         "Dummy",
    "description",       "",
    "version",           "",
    "vendor",            "",
    "category",          "",
    "activity_type",     "",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };

extern "C"
{
 
  void DummyModuleInit(RTC::Manager* manager)
  {
    RTC::Properties profile(dummy_spec);
    manager->registerFactory(profile,
                             RTC::Create<RTC::DummyModule>,
                             RTC::Delete<RTC::DummyModule>);
  }
  
};
