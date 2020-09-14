#ifndef CPPUTILS_OBJECT_H
#define CPPUTILS_OBJECT_H
#include <cstdarg>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace cpputils {

template <typename T> using Ptr = std::shared_ptr<T>;

template <typename T, typename... Targs>
static Ptr<T> CreateObject(Targs... Fargs) {
  Ptr<T> ptr(new T(Fargs...));
  return ptr;
}

} // namespace cpputils
#endif
