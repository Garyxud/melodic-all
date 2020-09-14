if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW) # Allow project(xxx VERSION a.b.c)
endif()


project(fyaml LANGUAGES C VERSION 0.5.8)

cmake_minimum_required(VERSION 3.0)

# Must use GNUInstallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

file(GLOB_RECURSE
    LIBHDRS
    "src/*.h"
)
file(GLOB
    LIBSRCS
    "src/lib/*.c"
    "src/xxhash/*.c"
)
file(GLOB_RECURSE
    LIBHDRSPUB
    "include/*.h"
)

add_library(${PROJECT_NAME}
    ${LIBHDRS}
    ${LIBSRCS}
    ${LIBHDRSPUB}
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Libraries")

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${${PROJECT_NAME}_SOURCE_DIR}/src/lib
        ${${PROJECT_NAME}_SOURCE_DIR}/src/lib/internal
        ${${PROJECT_NAME}_SOURCE_DIR}/src/xxhash
)

#target_link_libraries(${PROJECT_NAME} PUBLIC xx)
#target_compile_definitions(${PROJECT_NAME} PUBLIC xx)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    VERSION="0.5.7.34"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  $<$<NOT:$<CONFIG:DEBUG>>:NDEBUG>
)

# Shared options between GCC and CLANG:
if (NOT MSVC)
	target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall
        -fPIC
        -D_GNU_SOURCE
    )

	if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
		target_compile_options(${PROJECT_NAME} PRIVATE
            -O2
        )
	endif()
endif()

# ==== Install & export target ========

install(TARGETS	${PROJECT_NAME}
		EXPORT 	${PROJECT_NAME}-targets
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ${PROJECT_NAME}-targets
    FILE ${PROJECT_NAME}-targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
export(
	TARGETS ${PROJECT_NAME}
	FILE "${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME}-targets.cmake"
	NAMESPACE mbse::
)


include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
    ${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME}-config-version.cmake
    VERSION ${YART_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(
    FILES
        ${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME}-config.cmake
        ${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME}-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
