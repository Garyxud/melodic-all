project(rtshell)
cmake_minimum_required(VERSION 2.8)

if(EXISTS "/etc/debian_version")
  set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb")
else()
  set(SETUPTOOLS_ARG_EXTRA "")
endif()

add_custom_target(compile_rtshell ALL
  COMMAND python setup.py build
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

add_custom_target(install_rtshell
  COMMAND python setup.py install --prefix=${CMAKE_INSTALL_PREFIX} ${SETUPTOOLS_ARG_EXTRA} --root=/$(DESTDIR) --skip-build
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_BINARY_DIR}\" --target install_rtshell)")
install(FILES package.xml DESTINATION share/rtshell/)
