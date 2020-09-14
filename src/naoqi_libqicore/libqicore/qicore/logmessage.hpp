/*
** Author(s):
**  - Herve Cuche <hcuche@aldebaran-robotics.com>
**  - Matthieu Nottale <mnottale@aldebaran-robotics.com>
**
** Copyright (C) 2013 Aldebaran Robotics
*/

#ifndef QICORE_LOG_HPP_
#define QICORE_LOG_HPP_

#include <qi/log.hpp>
#include <qi/anyobject.hpp>
#include <qi/clock.hpp>
#include <tuple>

QI_TYPE_ENUM(qi::LogLevel)

namespace qi
{
namespace detail
{
  namespace name
  {
  // TODO will be used with VS2015
  // constexpr auto LogMessage_timestamp = "timestamp";
  // constexpr auto LogMessage_systemDate = "systemDate";
  // constexpr auto LogMessage_date = "date";

  inline const char* LogMessage_timestamp() { return "timestamp"; }
  inline const char* LogMessage_systemDate() { return "systemDate"; }
  inline const char* LogMessage_date() { return "date"; }
  }
}

struct LogMessage
{
  std::string source;                     // File:function:line
  qi::LogLevel level = qi::LogLevel_Info; // Level of verbosity of the message
  std::string category;                   // Category of the message
  std::string location;                   // machineID:PID
  std::string message;                    // The message itself
  unsigned int id = 0;                    // Unique message ID
  qi::Clock::time_point date;             // Steady clock timestamp
  qi::SystemClock::time_point systemDate; // Wall clock timestamp

  // timestamp when the message has been posted
  qi::os::timeval timestamp = qi::os::timeval(systemDate.time_since_epoch());
};
}

inline bool toOld(std::map<std::string, ::qi::AnyValue>& fields,
                  const std::vector<std::tuple<std::string, qi::TypeInterface*>>& missing,
                  const std::map<std::string, ::qi::AnyReference>& dropfields)
{
  try
  {
    if (missing.size() == 1 && std::get<0>(missing.front()) == qi::detail::name::LogMessage_timestamp())
    {
      if (dropfields.size() == 2)
      {
        auto systemDateIt = dropfields.find(qi::detail::name::LogMessage_systemDate());
        auto dateIt = dropfields.find(qi::detail::name::LogMessage_systemDate());
        if (systemDateIt != dropfields.end() && dateIt != dropfields.end())
        {
          const qi::SystemClock::time_point systemDate = systemDateIt->second.to<qi::SystemClock::time_point>();
          fields[qi::detail::name::LogMessage_timestamp()] =
              qi::AnyValue(qi::os::timeval(systemDate.time_since_epoch()));
          return true;
        }
      }
    }
  }
  catch (const std::exception& e)
  {
    qiLogVerbose("qi.core.LogMessage") << "Conversion error: " << e.what();
  }
  return false;
}

inline bool fromOld(std::map<std::string, ::qi::AnyValue>& fields,
                    const std::vector<std::tuple<std::string, qi::TypeInterface*>>& missing,
                    const std::map<std::string, ::qi::AnyReference>& dropfields)
{
  try
  {
    if (dropfields.size() == 1)
    {
      auto dropfieldsIt = dropfields.find(qi::detail::name::LogMessage_timestamp());
      if (dropfieldsIt != dropfields.end() && missing.size() == 2 &&
          ((std::get<0>(missing.at(0)) == qi::detail::name::LogMessage_date() &&
            std::get<0>(missing.at(1)) == qi::detail::name::LogMessage_systemDate()) ||
           (std::get<0>(missing.at(1)) == qi::detail::name::LogMessage_date() &&
            std::get<0>(missing.at(0)) == qi::detail::name::LogMessage_systemDate())))
      {
        const qi::os::timeval timestamp = dropfieldsIt->second.to<qi::os::timeval>();
        fields[qi::detail::name::LogMessage_date()] = qi::AnyValue(qi::Clock::time_point());
        fields[qi::detail::name::LogMessage_systemDate()] = qi::AnyValue(
            qi::SystemClock::time_point(qi::Seconds(timestamp.tv_sec) + qi::MicroSeconds(timestamp.tv_usec)));
        return true;
      }
    }
  }
  catch (const std::exception& e)
  {
    qiLogVerbose("qi.core.LogMessage") << "Conversion error: " << e.what();
  }
  return false;
}

QI_TYPE_STRUCT_EXTENSION_CONVERT_HANDLERS(::qi::LogMessage, fromOld, toOld);
QI_TYPE_STRUCT(::qi::LogMessage, source, level, category, location, message, id, date, systemDate);

#endif // !QICORE_LOG_HPP_
