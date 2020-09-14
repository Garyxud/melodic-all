#!/bin/bash
catkin build kvh_geo_fog_3d_driver -DCMAKE_BUILD_TYPE=Debug
gdb -tui ~/catkin_ws/devel/lib/kvh_geo_fog_3d_driver/kvh_geo_fog_3d_driver_node