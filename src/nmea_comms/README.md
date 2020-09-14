nmea_comms
==========

Generalized ROS interface to NMEA-speaking devices. 

Provides a serial node and socket server node, which relay CRLF-terminated strings on `rx` and `tx`
ROS topics using the [nmea_msgs/Sentence](http://docs.ros.org/latest-available/api/nmea_msgs/html/msg/Sentence.html) message type.
