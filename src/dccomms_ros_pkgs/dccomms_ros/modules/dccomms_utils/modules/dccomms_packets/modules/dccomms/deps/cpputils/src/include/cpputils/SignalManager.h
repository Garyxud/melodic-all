#include <csignal>
#include <functional>
#include <iostream>
#include <list>

namespace cpputils {
class SignalManager {
public:
  //http://en.cppreference.com/w/cpp/utility/program/signal
  //http://en.cppreference.com/w/cpp/language/lambda
  static void AddCallback(int signal, std::function<void(int)> handler);
  static void SetLastCallback(int signal, std::function<void(int)> handler);

private:
  static void _topHandler(int signal);
  static std::list<std::function<void(int)>> _sigint_callbacks, _sigterm_callbacks;
  static std::function<void(int)> _sigint_last_callback, _sigterm_last_callback;
};
}
