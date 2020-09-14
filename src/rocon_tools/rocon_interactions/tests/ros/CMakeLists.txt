##############################################################################
# RosTests - only run when CATKIN_ENABLE_TESTING is true
##############################################################################

find_package(catkin REQUIRED COMPONENTS rostest)

add_rostest(loader/loader.test)
