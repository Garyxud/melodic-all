cmake_minimum_required(VERSION 2.8.3)
project(knowledge_representation)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roslint
        )
find_package(Boost REQUIRED COMPONENTS python)

find_package(PythonLibs 2.7 REQUIRED)

find_package(PostgreSQL)

find_library(libpqxx NAMES pqxx)

find_library(mysqlcppconn NAMES libmysqlcppconn8.so)

# Check to see if we have the right version of MySQL Cpp Connector installed
if(POSTGRESQL_FOUND OR libpqxx)
    set(POSTGRES_AVAILABLE true)
    set(DB_INCLUDES ${PostgreSQL_INCLUDE_DIRS})
    set(DB_LIBS ${PostgreSQL_LIBRARIES} ${libpqxx})
    set(EXPORTED_DEPEND "")
    add_definitions(-DUSE_POSTGRESQL)
elseif(mysqlcppconn)
    message(WARNING "PostgreSQL not found. Using incomplete MySQL implementation.")
    set(MYSQL_AVAILABLE true)
    find_path(DB_INCLUDES mysqlx/xdevapi.h HINT /usr/local/mysql/connector-c++-8.0/include)
    set(mysqlcppconn_INCLUDE_DIRS ${DB_INCLUDES})
    set(mysqlcppconn_LIBRARIES ${mysqlcppconn})
    set(DB_LIBS ${mysqlcppconn})
    set(EXPORTED_DEPEND mysqlcppconn)
    add_definitions(-DUSE_MYSQL)
else()
    message(FATAL_ERROR "Neither MySQL nor PostgreSQL found. Please install one.")
endif()

catkin_python_setup()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES knowledge_rep ${DB_LIBS}
        CATKIN_DEPENDS
        DEPENDS ${EXPORTED_DEPEND}
)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${DB_INCLUDES}
        include
        ${PYTHON_INCLUDE_DIRS}
)


if ("${MYSQL_AVAILABLE}" OR "${POSTGRES_AVAILABLE}")
    set(INTERFACE_HEADER_PATH ${PROJECT_SOURCE_DIR}/include/knowledge_representation/LongTermMemoryConduit.h)
if (MYSQL_AVAILABLE)
    set(DB_SOURCES src/libknowledge_rep/LongTermMemoryConduitMySQL.cpp)
    set(DB_BACKEND MySQL)

elseif (POSTGRES_AVAILABLE)
    set(DB_SOURCES src/libknowledge_rep/LongTermMemoryConduitPostgreSQL.cpp)
    set(DB_BACKEND PostgreSQL)

endif()

# This header file imports the correct backend. Lets library users avoid
# depending on a particular backend
configure_file(${INTERFACE_HEADER_PATH}.in ${INTERFACE_HEADER_PATH})
add_library(knowledge_rep
        ${DB_SOURCES}
        src/libknowledge_rep/convenience.cpp
        )

target_link_libraries(knowledge_rep ${DB_LIBS} ${catkin_LIBRARIES})

add_library(_libknowledge_rep_wrapper_cpp src/libknowledge_rep/python_wrapper.cpp)
target_link_libraries(_libknowledge_rep_wrapper_cpp
        knowledge_rep
        ${DB_LIBS}
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES})

set_target_properties(_libknowledge_rep_wrapper_cpp PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )

catkin_add_env_hooks(${PROJECT_NAME} SHELLS bash zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

### INSTALL

install(TARGETS knowledge_rep _libknowledge_rep_wrapper_cpp
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

catkin_install_python(PROGRAMS scripts/ikr scripts/populate_with_knowledge scripts/populate_with_map scripts/show_me
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/configure_postgresql.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY sql/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sql)

### LINTING

file(GLOB_RECURSE ${PROJECT_NAME}_CPP_SRC
        RELATIVE ${PROJECT_SOURCE_DIR} src/lib${PROJECT_NAME}/*.cpp src/libknowledge_rep/*.h include/${PROJECT_NAME}/*.h test/*.cpp)
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-build/header_guard,-runtime/references,-build/c++11,-whitespace/braces")
roslint_cpp(${${PROJECT_NAME}_CPP_SRC})

# Lint Python modules for PEP8 compatibility
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SCRIPTS
        RELATIVE ${PROJECT_SOURCE_DIR} scripts/populate_* scripts/test_ltmc scripts/dump_to_owl)

# List FILTER isn't supported in 16.04's CMake
# list(FILTER ${PROJECT_NAME}_PY_SCRIPTS EXCLUDE REGEX ".*.sh")
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRC
        RELATIVE ${PROJECT_SOURCE_DIR} src/${PROJECT_NAME}/*.py)
set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
roslint_python(${${PROJECT_NAME}_PY_SCRIPTS})
roslint_python(${${PROJECT_NAME}_PY_SRC})

### TEST TARGETS
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_ltmc test/ltmc.cpp test/entity.cpp test/concept_instance.cpp test/map_types.cpp)
    target_link_libraries(test_ltmc knowledge_rep ${catkin_LIBRARIES})
    add_dependencies(test_ltmc _libknowledge_rep_wrapper_cpp)

    catkin_add_nosetests(test/ltmc.py)
    catkin_add_nosetests(test/loaders.py)
endif()

endif ()

### DOCS
find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "API Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
            VERBATIM)
endif()