# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jskeus)

find_package(euslisp REQUIRED)
message(STATUS "ARCHDIR=${ARCHDIR}")
message(STATUS "EUSDIR=${EUSDIR}")

execute_process(COMMAND grep version ${PROJECT_SOURCE_DIR}/package.xml
                COMMAND sed -e s/[^0-9.]//g
                OUTPUT_VARIABLE jskeus_VERSION
                OUTPUT_STRIP_TRAILING_WHITESPACE)
# build irteus on install command
set(INSTALLDIR \${DESTDIR}${CMAKE_INSTALL_PREFIX}/share/euslisp/jskeus/eus/)
add_custom_command(OUTPUT ${INSTALLDIR}
  COMMAND cmake -E make_directory ${INSTALLDIR}/lib
  COMMAND cmake -E make_directory ${INSTALLDIR}/${ARCHDIR}/bin
  COMMAND cmake -E make_directory ${INSTALLDIR}/${ARCHDIR}/obj
  COMMAND cmake -E make_directory ${INSTALLDIR}/${ARCHDIR}/lib)
add_custom_target(compile_jskeus
  DEPENDS ${INSTALLDIR}
  COMMAND export EUSDIR=${EUSDIR} DISPLAY="" lt_cv_sys_lib_dlsearch_path_spec=${lt_cv_sys_lib_dlsearch_path_spec} && \$\(MAKE\) all -j1 -l1 -d -C ${PROJECT_SOURCE_DIR}/irteus -f Makefile ARCHDIR=${ARCHDIR} EUSDIR=${EUSDIR} IRTEUSDIR=${INSTALLDIR} 'SVNVERSION=\\"${jskeus_VERSION}\\"' VERBOSE=1)

install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_BINARY_DIR}\" --target compile_jskeus RESULT_VARIABLE result_variable OUTPUT_VARIABLE output_variable)
if(NOT \${result_variable} EQUAL 0)
  message(FATAL_ERROR \"\${output_variable}\")
endif()")

install(DIRECTORY doc
   DESTINATION share/jskeus/
   USE_SOURCE_PERMISSIONS
   PATTERN "doc/fig" EXCLUDE
   PATTERN "doc/*.tex" EXCLUDE)
install(DIRECTORY irteus
   DESTINATION share/euslisp/jskeus/
   USE_SOURCE_PERMISSIONS)
install(FILES package.xml DESTINATION share/jskeus/)

add_custom_target(test) # dummy target for catkin test

find_package(catkin)
catkin_package()
string(REGEX MATCH "catkin" need_catkin "$ENV{_}")
get_filename_component(install_dir_name ${CMAKE_INSTALL_PREFIX} NAME)
if(need_catkin AND NOT install_dir_name MATCHES "install") # check if  install_prefix end with /install, we can  remove this section of we remove all rosrun euslisp ... usage
  install(CODE "
## this is tricky force write catkin marker file
set(_catkin_marker_file \"\${CMAKE_INSTALL_PREFIX}/.catkin\")
# check if the develspace marker file exists yet
if(EXISTS \${_catkin_marker_file})
  file(READ \${_catkin_marker_file} _existing_sourcespaces)
  if(_existing_sourcespaces STREQUAL \"\")
    # write this sourcespace to the marker file
    set(_catkin_marker_file_content \" ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}\") # head space is necessary because it is converted to semicolon.
    separate_arguments(_catkin_marker_file_content)
    file(APPEND \${_catkin_marker_file} \"${_catkin_marker_file_content}\")
  else()
    # append to existing list of sourcespaces if it's not in the list
    list(FIND _existing_sourcespaces \"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}\" _existing_sourcespace_index)
    if(_existing_sourcespace_index EQUAL -1)
      set(_catkin_marker_file_content \" ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}\") # head space is necessary because it is converted to semicolon.
      separate_arguments(_catkin_marker_file_content)
      file(APPEND \${_catkin_marker_file} \"${_catkin_marker_file_content}\")
    endif()
  endif()
else()
  # create a new develspace marker file
  # NOTE: extra care must be taken when running multiple catkin jobs in parallel 
  #       so that this does not overwrite the result of a similar call in another package
  set(_catkin_marker_file_content \" ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}\") # head space is necessary because it is converted to semicolon.
  separate_arguments(_catkin_marker_file_content)
  file(APPEND \${_catkin_marker_file} \"${_catkin_marker_file_content}\")
endif()
")
else()
  install(CODE "
file(GLOB catkin_install_files \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/*.py \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/*setup.* \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/env.* \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/.ros* \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/.catkin*)
foreach(file \${catkin_install_files})
  message(\"Removing: \${file}\")
  file(REMOVE \${file})
endforeach()
")
endif()
