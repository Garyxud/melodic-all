# need genmsg for _prepend_path()
find_package(catkin REQUIRED COMPONENTS genmsg)

include(CMakeParseArguments)

@[if DEVELSPACE]@
# program in develspace
set(GENSERVICEPAIR_BIN "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/genpair.py")
@[else]@
# program in installspace
set(GENSERVICEPAIR_BIN "${rocon_service_pair_msgs_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/genpair.py")
@[end if]@

macro(add_service_pair_files)
  cmake_parse_arguments(ARG "NOINSTALL" "DIRECTORY" "FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "add_service_pair_files() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_DIRECTORY)
    set(ARG_DIRECTORY "pairs")
  endif()

  if(NOT IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${ARG_DIRECTORY})
    message(FATAL_ERROR "add_service_pair_files() directory not found: ${CMAKE_CURRENT_SOURCE_DIR}/${ARG_DIRECTORY}")
  endif()

  # if FILES are not passed search service pair files in the given directory
  # note: ARGV is not variable, so it can not be passed to list(FIND) directly
  set(_argv ${ARGV})
  list(FIND _argv "FILES" _index)
  if(_index EQUAL -1)
    file(GLOB ARG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/${ARG_DIRECTORY}" "${CMAKE_CURRENT_SOURCE_DIR}/${ARG_DIRECTORY}/*.pair")
    list(SORT ARG_FILES)
  endif()
  _prepend_path(${CMAKE_CURRENT_SOURCE_DIR}/${ARG_DIRECTORY} "${ARG_FILES}" FILES_W_PATH)

  list(APPEND ${PROJECT_NAME}_ACTION_FILES ${FILES_W_PATH})
  foreach(file ${FILES_W_PATH})
    debug_message(2 "add_service_pair_files() action file: ${file}")
    assert_file_exists(${file} "service pair file not found")
  endforeach()

  if(NOT ARG_NOINSTALL)
    install(FILES ${FILES_W_PATH} DESTINATION share/${PROJECT_NAME}/${ARG_DIRECTORY})
  endif()

  foreach(service_pair_file ${FILES_W_PATH})
    get_filename_component(SERVICE_PAIR_SHORT_NAME ${service_pair_file} NAME_WE)
    set(MESSAGE_DIR ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/msg)
    set(OUTPUT_FILES
      ${SERVICE_PAIR_SHORT_NAME}Pair.msg
      ${SERVICE_PAIR_SHORT_NAME}Request.msg
      ${SERVICE_PAIR_SHORT_NAME}PairRequest.msg
      ${SERVICE_PAIR_SHORT_NAME}Response.msg
      ${SERVICE_PAIR_SHORT_NAME}PairResponse.msg
      )

    _prepend_path(${MESSAGE_DIR}/ "${OUTPUT_FILES}" OUTPUT_FILES_W_PATH)

    message(STATUS "Generating .msg files for service pair ${PROJECT_NAME}/${SERVICE_PAIR_SHORT_NAME}")
    stamp(${service_pair_file})

    if(${service_pair_file} IS_NEWER_THAN ${MESSAGE_DIR}/${SERVICE_PAIR_SHORT_NAME}Request.msg)
      safe_execute_process(COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENSERVICEPAIR_BIN} ${service_pair_file} -o ${MESSAGE_DIR})
    endif()

    add_message_files(
      BASE_DIR ${MESSAGE_DIR}
      FILES ${OUTPUT_FILES})
  endforeach()
  message(STATUS "     ... this is hard work, toss me a beer!")
endmacro()