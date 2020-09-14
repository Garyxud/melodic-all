#ifndef LOGGER_H
#define LOGGER_H

#include <queue>
#include <string>

namespace UnitTest
{
namespace Servant
{
class Logger
{
public:
  int push(std::string& str);
  int push(const char* str);
  std::string pop();
  int size() { return m_q.size(); }
private:
  std::queue<std::string> m_q;
}; // class Logger
}; // namespace Servant
}; // namespace UnitTest


#endif // LOGGER_H
