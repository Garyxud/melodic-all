cmake_minimum_required(VERSION 3.0.2)
project(lex_common)

SET(LEX_COMMON_VERSION 1.0.0)

find_package(aws_common REQUIRED)

add_compile_options(-std=c++11 -fPIC)

if (AWSSDK_FOUND)
    set(SERVICE lex)
    AWSSDK_DETERMINE_LIBS_TO_LINK(SERVICE AWSSDK_LIBS)
    link_directories("${AWSSDK_LIB_DIR}")
endif ()

include_directories(
        include
        ${aws_common_LIBRARIES})

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/lex_common.cpp
        src/lex_param_helper.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        ${aws_common_LIBRARIES})

target_include_directories(${PROJECT_NAME} PRIVATE ${AWSSDK_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        EXPORT "${PROJECT_NAME}-targets"
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

## Mark cpp header files for installation
install(
        DIRECTORY include/
        DESTINATION include/
)

export(TARGETS ${PROJECT_NAME}
        FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")
export(PACKAGE ${PROJECT_NAME})

CONFIGURE_FILE(${PROJECT_NAME}Config.cmake.in
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY)
CONFIGURE_FILE(${PROJECT_NAME}ConfigVersion.cmake.in
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake @ONLY)

INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(EXPORT ${PROJECT_NAME}-targets DESTINATION share/${PROJECT_NAME}/cmake)


#############
## Testing ##
#############

enable_testing()
find_common_test_packages()
if(GMOCK_LIBRARY)

    macro(link_test_target target_name)
        if(TARGET ${target_name})
            target_include_directories("${target_name}" PRIVATE ${HEADERS_FOR_TESTS})
            target_link_libraries("${target_name}" ${LIBS_FOR_TESTS})
        endif()
    endmacro()

    set(TEST_TARGET test_lex_common)

    set(TEST_FILES
        test/lex_param_test.cpp
        test/lex_common_test.cpp
        test/lex_interactor_test.cpp
        test/parameter_reader_mock.cpp
    )

    add_common_gtest(${TEST_TARGET}
        ${TEST_FILES}
    )

    set(LIBS_FOR_TESTS
        ${GTEST_LIBRARIES}
        pthread
        ${GMOCK_LIBRARY}
        ${aws_common_LIBRARIES}
        ${PROJECT_NAME}
    )

    set(HEADERS_FOR_TESTS
        ${aws_common_INCLUDE_DIRS}
        ${AWSSDK_INCLUDE_DIR}
        ${GTEST_INCLUDE_DIR}
        test/include
    )

    link_test_target(${TEST_TARGET})

endif()
