## Copyright (c) 2012, 2014 Aldebaran Robotics. All rights reserved.

include_directories("." ${catkin_INCLUDE_DIRS})

set(PUBLIC_HEADERS
  qicore/api.hpp
  qicore/loglistener.hpp
  qicore/logmanager.hpp
  qicore/logmessage.hpp
  qicore/logprovider.hpp
  qicore/file.hpp
  qicore/detail/fileoperation.hxx
)
install(FILES ${PUBLIC_HEADERS} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/qicore)

add_library(qicore
  ${PUBLIC_HEADERS}
  src/loglistener_proxy.cpp
  src/logmanager_proxy.cpp
  src/logprovider_proxy.cpp
  src/registration.cpp
  src/logproviderimpl.cpp
  src/logproviderimpl.hpp
  src/file_proxy.cpp
  src/fileimpl.cpp
  src/fileoperation.cpp
  src/progressnotifier.cpp
  src/progressnotifier_proxy.cpp
)

target_link_libraries(qicore ${Boost_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS qicore DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#add_subdirectory("tests")