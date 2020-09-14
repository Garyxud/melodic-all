macro(ecl_enable_cxx11_compiler)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)  # aborts with an error if the requested standard is not available
  set(CMAKE_CXX_EXTENSIONS OFF)  # if ON, it will use gnu++14 instead of std++14
  set(CMAKE_CXX_STANDARD 11)
endmacro()

