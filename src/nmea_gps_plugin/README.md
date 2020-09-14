# nmea_gps_plugin

![Developed By OUXT Polaris](img/logo.png "Logo")

| *master* | *develop* |
|----------|-----------|
|[![Build Status](https://travis-ci.org/OUXT-Polaris/nmea_gps_plugin.svg?branch=master)](https://travis-ci.org/OUXT-Polaris/nmea_gps_plugin)|[![Build Status](https://travis-ci.org/OUXT-Polaris/nmea_gps_plugin.svg?branch=develop)](https://travis-ci.org/OUXT-Polaris/nmea_gps_plugin)|

# Gazebo gps sensor plugin for publishing nmea_msgs/Sentence

## Available Sentence

### GPRMC
Timestamp,Status,Latitude,Longitude,Velocity,Heading of Velocity  

https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm  

### GPGGA
Timestamp,Latitude,Longitude,Number of satelite,Altitude

https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm?Highlight=GPGGA  

### GPVTG
Heading of Velocity,Velocity  

https://docs.novatel.com/OEM7/Content/Logs/GPVTG.htm?Highlight=GPVTG  

### GPHDT
True Heading

https://docs.novatel.com/OEM7/Content/Logs/GPHDT.htm?Highlight=GPHDT

### example urdf setup
```
    <gazebo>
      <plugin name="gps_plugin_${name}" filename="libnmea_gps_plugin.so">
        <updateRate>15.0</updateRate>
        <alwaysOn>true</alwaysOn>
        <bodyName>${name}_link</bodyName>
        <frameId>${name}_link</frameId>
        <topicName>nmea_sentence</topicName>
        <!--<velocityTopicName>gps/fix_velocity</velocityTopicName>-->
        <!-- Location of origin of Gazebo Sand Island map -->
        <referenceLatitude>21.30996</referenceLatitude>
        <referenceLongitude>-157.8901</referenceLongitude>
        <referenceAltitude>0</referenceAltitude>
        <referenceHeading>90</referenceHeading>
        <velocityGaussiaNoise>0.05</velocityGaussiaNoise>
        <orientationGaussiaNoise>0.05</orientationGaussiaNoise>
        <positionGaussiaNoise>0.05</positionGaussiaNoise>        
      </plugin>
    </gazebo>
```

## Demo

[![No video](http://img.youtube.com/vi/DXBwnfiT0oM/0.jpg)](https://www.youtube.com/watch?v=DXBwnfiT0oM)
