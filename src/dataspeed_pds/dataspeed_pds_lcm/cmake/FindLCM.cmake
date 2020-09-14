INCLUDE(FindPackageHandleStandardArgs)

FIND_PATH(LCM_INCLUDE_PATH
  NAMES lcm.h
  HINTS
  /usr/local/include
  /usr/include
  PATH_SUFFIXES
  lcm
  )


FIND_LIBRARY(LCM_LIBRARIES
  NAMES
  lcm
  HINTS
  /usr/local/lib
  /usr/lib
  /usr/lib/x86_64-linux-gnu
  )

FIND_PACKAGE_HANDLE_STANDARD_ARGS(LCM DEFAULT_MSG
  LCM_INCLUDE_PATH
  LCM_LIBRARIES
  )
