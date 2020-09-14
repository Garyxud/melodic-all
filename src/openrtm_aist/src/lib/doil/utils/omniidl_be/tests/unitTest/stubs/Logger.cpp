
#include <Logger.h>

namespace UnitTest
{
namespace Servant
{
  int Logger::push(std::string& str)
  {
    m_q.push(str);
    return m_q.size();
  }

  int Logger::push(const char * aStr)
  {
    std::string str(aStr);
    return push(str);
  }

  std::string Logger::pop()
  {
    std::string str("");
    if (m_q.empty()) return str;

    str = m_q.front();
    m_q.pop();
    return str;
  }
}; // namespace Servant
}; // namespace UnitTest

