cmake_minimum_required(VERSION 2.8.3)
project(yocs_msgs)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure actionlib_msgs std_msgs geometry_msgs std_srvs message_generation)

add_message_files(FILES ARPair.msg
                        ARPairList.msg
                        Wall.msg
                        WallList.msg
                        Column.msg
                        ColumnList.msg
                        MagicButton.msg
                        NavigationControl.msg
                        NavigationControlStatus.msg
                        Table.msg
                        TableList.msg
                        Trajectory.msg
                        TrajectoryList.msg
                        Waypoint.msg
                        WaypointList.msg)

add_service_files(FILES WaypointListService.srv)

add_action_files(
  DIRECTORY action
  FILES
  NavigateTo.action
  DockingInteractor.action
  Localize.action
)

generate_dynamic_reconfigure_options( dynamic_reconfigure/Joystick.cfg )

generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs std_srvs)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure actionlib_msgs std_msgs geometry_msgs std_srvs message_runtime)

# install(DIRECTORY dynamic_reconfigure DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})