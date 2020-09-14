#
#	EXAMPLES	#
#
#
INCLUDE_DIRECTORIES(BEFORE ${CMAKE_SOURCE_DIR}/src)

#	TEST EPOS2
ADD_EXECUTABLE(test_epos2 test_epos2.cpp)
#		dependences
TARGET_LINK_LIBRARIES(test_epos2 epos2)
#
#	CONFIGTEST
ADD_EXECUTABLE(configtest configtest.cpp)
#		dependences
TARGET_LINK_LIBRARIES(configtest epos2)
#
# TEST like h3d
#ADD_EXECUTABLE(like_h3d test_like_h3d.cpp)
#   dependences
#TARGET_LINK_LIBRARIES(like_h3d epos2)
#
# TEST test positions
#ADD_EXECUTABLE(positions test_positions.cpp)
#   dependences
#TARGET_LINK_LIBRARIES(positions epos2)
#
# TEST test homing
ADD_EXECUTABLE(homing test_homing.cpp)
#   dependences
TARGET_LINK_LIBRARIES(homing epos2)
#
# TEST test marker
ADD_EXECUTABLE(marker test_marker.cpp)
#   dependences
TARGET_LINK_LIBRARIES(marker epos2)
