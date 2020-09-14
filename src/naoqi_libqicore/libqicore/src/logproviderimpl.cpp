/*
** Author(s):
**  - Herve Cuche <hcuche@aldebaran-robotics.com>
**  - Matthieu Nottale <mnottale@aldebaran-robotics.com>
**
** Copyright (C) 2013 Aldebaran Robotics
*/

// VS2015 fix atomic alignment and require an acknowledgement from developer
#include <boost/predef.h>
#if BOOST_COMP_MSVC
# if (BOOST_COMP_MSVC >= BOOST_VERSION_NUMBER(14, 0, 0))
#  define _ENABLE_ATOMIC_ALIGNMENT_FIX
# endif
#endif

#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/lockfree/queue.hpp>
#include <boost/lambda/algorithm.hpp>

#include <qi/application.hpp>
#include <qi/anymodule.hpp>
#include <qi/anyobject.hpp>
#include <qi/type/objecttypebuilder.hpp>
#include <qi/os.hpp>
#include <qi/getenv.hpp>

#include "src/logproviderimpl.hpp"

QI_TYPE_INTERFACE(LogProvider);

qiLogCategory("log.provider");

namespace
{
const bool debug = (!qi::os::getenv("LOG_DEBUG").empty());
#define DEBUG(a)                   \
  do                               \
  {                                \
    if (debug)                     \
      std::cerr << a << std::endl; \
  } while (0)
}

namespace qi
{

boost::lockfree::queue<qi::LogMessage*> _pendingMessages(qi::os::getEnvDefault("QI_LOG_MAX_MSGS_BUFFERS", 500));

LogProviderPtr makeLogProvider()
{
  return boost::shared_ptr<qi::LogProviderImpl>(new LogProviderImpl());
}

LogProviderPtr makeLogProvider(LogManagerPtr logger)
{
  return boost::shared_ptr<qi::LogProviderImpl>(new LogProviderImpl(logger));
}

static void removeProviderAtStop(SessionPtr session, int id)
{
  DEBUG("LP removeProviderAtStop " << id);
  LogManagerPtr lm = session->service("LogManager");
  lm->removeProvider(id);
}

static bool initialized = false;
qi::FutureSync<qi::LogProviderPtr> initializeLogging(SessionPtr session, const std::string& categoryPrefix)
{
  DEBUG("registering new provider");
  if (initialized)
    throw std::runtime_error("Provider already registered for this process");

  LogManagerPtr lm = session->service("LogManager");
  LogProviderPtr instance = makeLogProvider(lm);
  if (!categoryPrefix.empty())
    instance->setCategoryPrefix(categoryPrefix);

  qi::Future<int> id = lm.async<int>("addProvider", instance);
  DEBUG("LP registerToLogger " << instance.ptrUid());

  initialized = true;

  qi::Application::atStop(boost::bind(removeProviderAtStop, session, id));
  return id.then(boost::lambda::constant(instance));
}


static void silenceQiCategories(qi::log::SubscriberId subscriber)
{
  // Safety: avoid infinite loop
  ::qi::log::addFilter("qitype.*", qi::LogLevel_Silent, subscriber);
  ::qi::log::addFilter("qimessaging.*", qi::LogLevel_Silent, subscriber);
  ::qi::log::addFilter("qi.*", qi::LogLevel_Silent, subscriber);
}

LogProviderImpl::LogProviderImpl()
  : _logger()
{
  DEBUG("LP subscribed this " << this);
  _subscriber =
      qi::log::addHandler("remoteLogger", boost::bind(&LogProviderImpl::log, this, _1, _2, _3, _4, _5, _6, _7, _8));

  DEBUG("LP subscribed " << _subscriber);
  silenceQiCategories(_subscriber);
  ++_ready;
  sendTask.setName("LogProvider");
  sendTask.setUsPeriod(100 * 1000); // 100ms
  sendTask.setCallback(&LogProviderImpl::sendLogs, this);
  sendTask.start();
}

LogProviderImpl::LogProviderImpl(LogManagerPtr logger)
  : _logger(std::move(logger))
{
  DEBUG("LP subscribed this " << this);
  _subscriber =
      qi::log::addHandler("remoteLogger", boost::bind(&LogProviderImpl::log, this, _1, _2, _3, _4, _5, _6, _7, _8));

  DEBUG("LP subscribed " << _subscriber);
  silenceQiCategories(_subscriber);
  ++_ready;
  sendTask.setName("LogProvider");
  sendTask.setUsPeriod(100 * 1000); // 100ms
  sendTask.setCallback(&LogProviderImpl::sendLogs, this);
  sendTask.start();
}

LogProviderImpl::~LogProviderImpl()
{
  DEBUG("LP ~LogProviderImpl");
  sendTask.stop();
  sendLogs();
  qi::log::removeHandler("remoteLogger");
}

void LogProviderImpl::setLogger(LogManagerPtr logger)
{
  _logger = logger;
}

void LogProviderImpl::sendLogs()
{
  if (!_pendingMessages.empty() && _logger)
  {
    DEBUG("LP sendLogs");
    std::vector<qi::LogMessage> msgs;
    qi::LogMessage* msg;
    while (_pendingMessages.pop(msg))
    {
      msgs.push_back(*msg);
      delete msg;
    }
    try
    {
      _logger->log(msgs);
    }
    catch (const std::exception& e)
    {
      DEBUG(e.what());
    }
  }
}

void LogProviderImpl::log(qi::LogLevel level,
                          const Clock::time_point date,
                          const SystemClock::time_point systemDate,
                          const char* category,
                          const char* message,
                          const char* file,
                          const char* function,
                          int line)
{
  DEBUG("LP log callback: " << message << " " << file << " " << function);
  if (!_ready.load())
    return;

  LogMessage* msg = new LogMessage();
  std::string source(file);
  source += ':';
  source += function;
  source += ':';
  source += boost::lexical_cast<std::string>(line);
  msg->source = source;
  msg->level = level;
  msg->date = date;
  msg->systemDate = systemDate;
  if (_categoryPrefix.empty())
    msg->category = category;
  else
    msg->category = _categoryPrefix + "." + category;
  msg->location = qi::os::getMachineId() + ":" + boost::lexical_cast<std::string>(qi::os::getpid());
  msg->message = message;
  msg->id = -1;

  _pendingMessages.push(msg);

  DEBUG("LP:log done");
}

void LogProviderImpl::setCategoryPrefix(const std::string& categoryPrefix)
{
  DEBUG("LP setCategoryPrefix " << categoryPrefix);
  _categoryPrefix = categoryPrefix;
}

void LogProviderImpl::setLevel(qi::LogLevel level)
{
  DEBUG("LP verb " << level);
  ::qi::log::setLogLevel(level, _subscriber);
}

void LogProviderImpl::addFilter(const std::string& filter, qi::LogLevel level)
{
  DEBUG("LP addFilter level: " << level << " cat: " << filter);
  {
    boost::mutex::scoped_lock sl(_setCategoriesMutex);
    _setCategories.insert(filter);
  }
  ::qi::log::addFilter(filter, level, _subscriber);
}

void LogProviderImpl::setFilters(const std::vector<std::pair<std::string, qi::LogLevel> >& filters)
{
  DEBUG("LP setFilters");
  {
    boost::mutex::scoped_lock sl(_setCategoriesMutex);
    for (std::set<std::string>::iterator it = _setCategories.begin(); it != _setCategories.end(); ++it)
    {
      if (*it != "*")
        ::qi::log::addFilter(*it, qi::LogLevel_Debug, _subscriber);
    }

    _setCategories.clear();
  }
  qi::LogLevel wildcardLevel = qi::LogLevel_Silent;
  bool wildcardIsSet = false;
  for (unsigned i = 0; i < filters.size(); ++i)
  {
    if (filters[i].first == "*")
    {
      wildcardLevel = filters[i].second;
      wildcardIsSet = true;
    }
    else
      addFilter(filters[i].first, filters[i].second);
  }

  silenceQiCategories(_subscriber);

  if (wildcardIsSet)
    ::qi::log::addFilter("*", wildcardLevel, _subscriber);
}

QI_REGISTER_MT_OBJECT(LogProvider, setLevel, addFilter, setFilters, setLogger, setCategoryPrefix);
QI_REGISTER_IMPLEMENTATION(LogProvider, LogProviderImpl);

void registerLogProvider(qi::ModuleBuilder* mb)
{
  mb->advertiseFactory<LogProviderImpl, LogManagerPtr>("LogProvider");
  mb->advertiseMethod("makeLogProvider", static_cast<LogProviderPtr (*)(LogManagerPtr)>(&makeLogProvider));
  mb->advertiseMethod("makeLogProvider", static_cast<LogProviderPtr (*)()>(&makeLogProvider));
  mb->advertiseMethod("initializeLogging", &initializeLogging);
  mb->advertiseMethod("initializeLogging", (boost::function<qi::FutureSync<qi::LogProviderPtr> (SessionPtr)>(boost::bind(&initializeLogging, _1, ""))));
}

} // !qi
