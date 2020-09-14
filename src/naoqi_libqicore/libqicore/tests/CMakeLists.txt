function(qi_create_session_test name)
  if (DEFINED BUILD_TESTS AND NOT BUILD_TESTS)
    return()
  endif()
  cmake_parse_arguments(ARG "" "TIMEOUT" "ARGUMENTS;SRC;DEPENDS" ${ARGN})

  MATH( EXPR ARG_TIMEOUT_SSL "${ARG_TIMEOUT} * 2" )

  qi_create_gtest(${name} SRC ${ARG_SRC} DEPENDS ${ARG_DEPENDS} NO_ADD_TEST)
  qi_add_test(${name}_direct ${name} ARGUMENTS --mode=direct ${ARG_ARGUMENTS} TIMEOUT ${ARG_TIMEOUT})
  qi_add_test(${name}_sd ${name} ARGUMENTS --mode=sd ${ARG_ARGUMENTS} TIMEOUT ${ARG_TIMEOUT})
  if (WITH_SSL)
  qi_add_test(${name}_ssl ${name} ARGUMENTS --mode=ssl ${ARG_ARGUMENTS} TIMEOUT ${ARG_TIMEOUT_SSL})
  endif()
endfunction()

if(QI_WITH_TESTS)
  qi_create_gtest(test_file SRC test_file.cpp DEPENDS QICORE GTEST TESTSESSION)
endif()

qi_create_bin(send_robot_icon SRC send_robot_icon.cpp DEPENDS QICORE)
