ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

ADD_UNIT_TEST(pendulum pendulum.cpp)
TARGET_LINK_LIBRARIES(pendulum ${PROJECT_NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} dynamic-graph::dynamic-graph)
TARGET_INCLUDE_DIRECTORIES(pendulum PUBLIC dynamic-graph::dynamic-graph)

ADD_UNIT_TEST(controller controller.cpp)
TARGET_LINK_LIBRARIES(controller ${PROJECT_NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} dynamic-graph::dynamic-graph)
TARGET_INCLUDE_DIRECTORIES(controller PUBLIC dynamic-graph::dynamic-graph)

IF(BUILD_PYTHON_INTERFACE)
  ADD_PYTHON_UNIT_TEST(simu "tests/simu.py" src)
ENDIF(BUILD_PYTHON_INTERFACE)
