cmake_minimum_required(VERSION 3.5)
project(abseil_cpp)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set( ABSL_PUBLIC_LIBRARIES
    absl_spinlock_wait
    absl_dynamic_annotations
    absl_malloc_internal
    absl_base
    absl_throw_delegate
    absl_scoped_set_env
    absl_hashtablez_sampler
    absl_raw_hash_set
    absl_examine_stack
    absl_failure_signal_handler
    absl_debugging_internal
    absl_demangle_internal
    absl_leak_check
    absl_leak_check_disable
    absl_hash
    absl_city
    absl_int128
    absl_graphcycles_internal
    absl_bad_any_cast_impl
    absl_bad_optional_access
    absl_bad_variant_access
    absl_civil_time
    absl_flags_internal
    absl_flags_config
    absl_flags_marshalling
    absl_flags_handle
    absl_flags_registry
    absl_flags
    absl_flags_usage
    absl_flags_parse
    absl_str_format_internal
    absl_strings
    absl_strings_internal
    absl_stacktrace
    absl_symbolize
    absl_synchronization
    absl_time
    absl_time_zone
)

catkin_package(
   INCLUDE_DIRS ./abseil_cpp
   LIBRARIES ${ABSL_PUBLIC_LIBRARIES}
)

add_subdirectory(abseil_cpp)


