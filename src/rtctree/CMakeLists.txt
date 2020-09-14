project(rtctree)
cmake_minimum_required(VERSION 2.8)

if(EXISTS "/etc/debian_version")
  set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb")
else()
  set(SETUPTOOLS_ARG_EXTRA "")
endif()

add_custom_target(compile_rtctree ALL
  COMMAND python setup.py build
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

add_custom_target(install_rtctree
  COMMAND python setup.py install --prefix=$(DESTDIR)${CMAKE_INSTALL_PREFIX} ${SETUPTOOLS_ARG_EXTRA} --skip-build
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_BINARY_DIR}\" --target install_rtctree)")
install(FILES package.xml DESTINATION share/rtctree/)
