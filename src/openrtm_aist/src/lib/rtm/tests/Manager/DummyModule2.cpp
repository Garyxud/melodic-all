#include <rtm/DataFlowComponentBase.h>
#include "DummyModule2.h"

namespace RTC
{

class Manager;

class DummyModule2
  : public RTC::DataFlowComponentBase
{
public:
        DummyModule2(RTC::Manager* manager)
            : RTC::DataFlowComponentBase(manager)
        {
        };
	static void InitProc(Manager* manager) { m_counter++; }
	static int getInitProcCount() { return m_counter; }
	static void resetInitProcCount() { m_counter = 0; }
	
private:
	static int m_counter;
};
int DummyModule2::m_counter = 0;

void InitProc(Manager* manager)
{
	DummyModule2::InitProc(manager);
}

int getInitProcCount()
{
	return DummyModule2::getInitProcCount();
}

void resetInitProcCount()
{
	DummyModule2::resetInitProcCount();
}

};

static const char* dummy2_spec[] =
  {
    "implementation_id", "Dummy2",
    "type_name",         "Dummy2",
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
 
  void DummyModule2Init(RTC::Manager* manager)
  {
    RTC::Properties profile(dummy2_spec);
    manager->registerFactory(profile,
                             RTC::Create<RTC::DummyModule2>,
                             RTC::Delete<RTC::DummyModule2>);
  }
  
};
