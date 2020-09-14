#!/bin/bash

rosservice call /kvh_geo_fog_3d_driver_node/set_logger_level "{logger: 'ros.kvh_geo_fog_3d_driver', level: 'info'}"
rosservice call /kvh_geo_fog_3d_driver_node/set_logger_level "{logger: 'rosout', level: 'info'}"
