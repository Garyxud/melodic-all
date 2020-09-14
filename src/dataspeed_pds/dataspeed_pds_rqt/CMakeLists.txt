cmake_minimum_required(VERSION 3.0.2)
project(dataspeed_pds_rqt)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
  python_qt_binding
)

catkin_python_setup()

catkin_package()

install(DIRECTORY images resource
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/dataspeed_pds_rqt
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

