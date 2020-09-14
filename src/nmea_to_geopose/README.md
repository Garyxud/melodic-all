# nmea_to_geopose
ROS package for converting 

![Developed By OUXT Polaris](img/logo.png "Logo")

| *master* | *develop* |
|----------|-----------|
|[![Build Status](https://travis-ci.org/OUXT-Polaris/nmea_to_geopose.svg?branch=master)](https://travis-ci.org/OUXT-Polaris/nmea_to_geopose)|[![Build Status](https://travis-ci.org/OUXT-Polaris/nmea_to_geopose.svg?branch=develop)](https://travis-ci.org/OUXT-Polaris/nmea_to_geopose)|

# nodes
nmea_to_geopose_node

## rosparam
### input_topic (string)
default : "nmea_sentence"

## rostopic
### Subscribe
/nmea_sentence [nmea_msgs/Sentence]  
defined by input_topic rospram.

### Publish
/nmea_to_geopose_node/geopose [geographic_msgs/GeoPoseStamped]  
robot pose in geopose format.  
frame_id is filled by nmea_sentence frame_id.  

# how to launch
roslaunch nmea_to_geopose nmea_to_geopose.launch input_topic:="nmea_sentence"