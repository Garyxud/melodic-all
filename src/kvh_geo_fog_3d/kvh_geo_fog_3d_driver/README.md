# kvh-geo-fog-3d-driver

Driver for the KVH GEO FOG 3D inertial navigation systems. Connects over serial
to the KVH GEO FOG 3D device and publishes out both KVH-specific and generic
ROS-ified messages, as well as performs basic transforms to convert data to
the ROS-conformant conventions.

Initial testing at The MITRE Corporation was done on the "Dual" model, which
provides dual-antenna heading solutions. However, the API should match between
the dual and non-dual models.

For detailed information on the KVH GEO FOG 3D functionality, consult the 
technical reference manual.

Product pages:

[KVH GEO FOG 3D](https://www.kvh.com/admin/products/gyros-imus-inss/ins/geo-fog-3d/commercial-geo-fog-3d)

[KVH GEO FOG 3D Dual](https://www.kvh.com/admin/products/gyros-imus-inss/ins/geo-fog-3d-dual/commercial-geo-fog-3d-dual)

Copyright 2019 The MITRE Corporation. All Rights Reserved.

# ROS API

## kvh_geo_fog_3d_driver_node

The main driver node.

### Published Topics

#### KVH-specific messages

All of these messages are defined as part of the kvh_geo_fog_3d_msgs package. They
are direct pass-throughs of KVH messages, so almost universally do not conform
with ROS REPs (103, 105, or 145).

KVH reports latitude and longitude in radians.

- `~<node_name>/kvh_system_state` (kvh_geo_fog_3d_msgs/KvhGeoFog3DSystemState)
- `~<node_name>/kvh_satellites` (kvh_geo_fog_3d_msgs/KvhGeoFog3DSatellites)
- `~<node_name>/kvh_detailed_satellites` (kvh_geo_fog_3d_msgs/KvhGeoFog3DDetailSatellites)
- `~<node_name>/kvh_local_magnetics` (kvh_geo_fog_3d_msgs/KvhGeoFog3DLocalMagneticField)
- `~<node_name>/kvh_utm_position` (kvh_geo_fog_3d_msgs/KvhGeoFog3DUTMPosition)
- `~<node_name>/kvh_ecef_pos` (kvh_geo_fog_3d_msgs/KvhGeoFog3DECEFPos)
- `~<node_name>/kvh_north_seeking_status` (kvh_geo_fog_3d_msgs/KvhGeoFog3DNorthSeekingInitStatus)
- `~<node_name>/kvh_odometer_state` (kvh_geo_fog_3d_msgs/KvhGeoFog3DSatellites)
- `~<node_name>/kvh_raw_sensors` (kvh_geo_fog_3d_msgs/KvhGeoFog3DRawSensors)
- `~<node_name>/kvh_raw_gnss` (kvh_geo_fog_3d_msgs/KvhGeoFog3DRawGNSS)

#### Conventional ROS messages

Not all of these messages conform to ROS REP-105 or REP-145. See the
"ROS Conformance and Conventions" section below for more details.

Generally speaking, anything with a "_flu" or a "_enu" suffix conforms to ROS
REPs.

All angles are measured in radians unless suffixed with "_deg".

NavSatFix latitude/longitude measured in degrees (see message definition). This
is different than the KVH messages, which report latitude/longitude in radians.

- `~<node_name>/imu/data_raw_frd` (sensor_msgs/Imu)
- `~<node_name>/imu/data_raw_flu` (sensor_msgs/Imu) **REP-105, REP-145 compliant**
- `~<node_name>/imu/data_ned` (sensor_msgs/Imu)
- `~<node_name>/imu/data_enu` (sensor_msgs/Imu) **REP-105, REP-145 compliant**
- `~<node_name>/imu/rpy_ned` (geometry_msgs/Vector3Stamped)
- `~<node_name>/imu/rpy_ned_deg` (geometry_msgs/Vector3Stamped)
- `~<node_name>/imu/rpy_enu` (geometry_msgs/Vector3Stamped)
- `~<node_name>/imu/rpy_enu_deg` (geometry_msgs/Vector3Stamped)
- `~<node_name>/gps/fix` (sensor_msgs/NavSatFix) **Filtered GNSS location, in degrees**
- `~<node_name>/gps/raw_fix` (sensor_msgs/NavSatFix) **Raw GNSS location, in degrees**
- `~<node_name>/gps/mag` (sensor_msgs/MagneticField)
- `~<node_name>/gps/utm_ned` (nav_msgs/Odometry)
- `~<node_name>/gps/utm_enu` (nav_msgs/Odometry) **REP-105 compliant**
- `~<node_name>/odom/wheel_encoder` (nav_msgs/Odometry)
- `~<node_name>imu/raw_sensor_frd` (nav_msgs/Odometry)
- `~<node_name>/odom/raw_sensor_flu` (nav_msgs/Odometry) **REP-105 compliant**

### Parameters

The driver asks for the wheel odometer pulses per meter to be passed as a parameter.
This is because the KVH API provides the odometer pulses directionally (i.e. if
you back up, the pulse count decreases) but the odometer distance as an absolute
distance traveled (i.e. backing up increases distance traveled, and it never goes
negative). This is the only source of non-differentiated data, which is useful
to fuse as a position in filters since it doesn't drift over time (unlike velocity,
which will drift).
The KVH API does not provide a method for obtaining its current
wheel odometer pulse distance value. Thus, it must be hand-coded in as a parameter.
In the future, hopefully KVH exposes this value over the API.

- `~<node_name>/port` (string, default: /dev/ttyUSB0)
- `~<node_name>/baud` (int, default: 115200)
- `~<node_name>/debug` (bool, default: false)
- `~<node_name>/filterVehicleType` (string, default: 2 (car, see spatial_packets.h))
- `~<node_name>/atmosphericAltitudeEnabled` (bool, default: true)
- `~<node_name>/velocityHeadingEnabled` (bool, default: false)
- `~<node_name>/reversingDetectionEnabled` (bool, default: true)
- `~<node_name>/motionAnalysisEnabled` (bool, default: true)
- `~<node_name>/odomPulseToMeters` (double, default: 0.000604)

## determine_baud_node

A node to determine and, optionally, set the baud rate of the sensor. The supported
baud rates are listed out in the technical reference manual.

### Parameters

- `~<node_name>/port` (string, default: /dev/ttyUSB0)

# Setup

## Installation

Below are instructions on getting connected to the KVH.

### Serial Port Configuration

To connect to the KVH you can use the FTDI cable KVH provides or a serial cable to connect. In our setup, we use the FTDI cable to connect to our laptops for development and debug, and then a serial cable to connect to our mission computer. 

By default, the serial port is owned by the root.dialout group. To run this driver without root privileges, you must add your user to the dialout group. If you are still having permission errors you can use chmod to change the permissions manually.

```bash
$ sudo chmod a+rw <port>  # E.g., port may be /dev/ttyUSB0
```

Beware that some COM ports are limited to 115200, which is not enough bandwidth to handle large amounts of KVH data. In our case, we had to enable a multiplexer via the bios on our mission computer to allow larger baud rates. The FTDI cable KVH provides should support all baud rates.

### Running the driver

First you will need to clone and build the repository

```bash
$ git clone <url>
$ cd kvh_geo_fog_3d
$ catkin build kvh_geo_fog_3d
```

It is important that you have the KVH messages installed properly. After building initially you can check by running:
```bash
$ source .../devel/setup.bash # May be required after building
$ rosmsg list | grep kvh
```

You should see the messages listed in the KVH-specific messages section above.

Now you should be able to launch the driver by running

```bash
$ roslaunch kvh_geo_fog_3d_driver kvh_geo_fog_3d_node.launch port:=<port> baud:=<baud> # E.g., port="/dev/ttyUSB0" baud="921600"
```
If everything is set up properly, you should be able to see data being output on each of the topics listed above. Currently the driver is hardcoded to output the data from the following kvh packets:
1. System State
2. Satellites
3. Detailed Satellites
4. Local Magnetics
5. UTM position
6. ECEF position
7. North Seeking Status
8. Odometer State
9. Raw Sensors
10. Raw GNSS

If you need other packets, please see the development guide below.

Though the KVH defaults to 115200 baud, we recommend using the baud utility in the section below to set this baud rate to 921600 to prevent data overflow.

#### Baud Debug
Note that the above assumes you know the baudrate, which is 115200 by default. If you do not know the baud rate you have a couple options:

1. Relaunch the node manually trying each of the possible baud rates.
2. Contact KVH and get access to the GEO-FOG manager, which is a GUI tool for interacting with the KVH.
3. We have provided a ros node that attempts to automatically detect the baud rate by trying each of the buad rates and checking for returned data. You can launch this node by running the command below. By default it will start at 1200, but the starting baud rate setting is available if you believe that the baud rate it is finding is incorrect or recieving random data (i.e. allows you to skip baud rates).
```bash
$ roslaunch kvh_geo_fog_3d_driver determine_baud.launch starting_baud:=<baud_rate>
```
Due to the rs232 connecting library we use, the possible baud rates are:
1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 576000, 921600, 1000000

This will give you the option to set the buad rate as well. It is currently recommended to set the baud rate to 921600 as that is the smallest baud rate that will never overflow. 

**Note: this number can be smaller and still work, but due to the possibility of the detailed satellites packet being the maximum size we recommend this.**

# ROS conformance and Conventions

This driver attempts to conform to ROS Enhancement Proposals (REPs) when appropriate.

## REP 105 - Coordinate Frames for Mobile Platforms

Link: https://www.ros.org/reps/rep-0105.html

Summary:
* Data will be published by default in sensor conventions (FRD/NED) with the suffixes _frd and _ned
* Data will also be published with FLU/ENU topics, with the suffixes _flu and _enu

This sensor, like many navigation sensors, uses the somewhat standard Front-Right-Down
and North-East-Down (FRD/NED) coordinate frame conventions. ROS opts to use the standards
Front-Left-Up and East-North-Up (FLU/ENU) frames.

All data will be published in the sensor-standard formats (FRD/NED) and suffixed with "_frd"
and "_ned". The driver will also publish FLU/ENU topics to conform with ROS REP 105.
These topics will have the suffix "_flu" and "_enu".

## REP 145 - Conventions for IMU Sensor Drivers

Link: http://www.ros.org/reps/rep-0145.html

Summary:
* The driver will not perform additional off-sensor processing, other than coordinate frame transforms.
* The raw data will be published in two frames: imu_link_frd and imu_link_flu.
* imu/data_raw_frd and imu/data_raw_flu will be data without orientation data, i.e. pure measurement data
* imu/data_ned and imu/data_enu will be data with orientation data.
* Orientation data is north-oriented.
* In the orientation messages, the twist component (i.e. accelerations and velocities) are given with respect
to the child_frame_id and the pose component (i.e. position and orientation) are given with respect to the
frame_id.

This sensor fuses orientation data into its inertial solution, using GPS-baseline heading calculations,
gyro-compassing, and course-over-ground calculations, among other methods. To conform with ROS REP 145,
we will publish the raw inertial data, stripped of orientation, in both FRD and FLU frames.

We will deviate from REP 145 by explicitly terminating topics with _frd and _flu. There will be no
standard imu/data_raw and imu/data labelled topics. The end user should select the topic with the
frame convention they'd like to consume.

For frame_id strings, we will deviate from REP 103 and use the following:
* imu_link_frd and imu_link_flu - These frames will be used for raw data.
* utm_ned and utm_enu - These frames are necessary because the orientation data is north-oriented.

## Bearing Convention

This sensor produces bearing measurements with respect to true north. The other two forms of north are
magnetic north and grid north. Grid north differs very slightly from true north, and is dependent upon
where in the UTM grid your sensor is located. Magnetic north differs significantly from true north, and
is dependent upon the fluctuation of the magnetic field.

## Orientation Convention

The orientations presented by the sensor are either in ENU or NED conventions. Practically, this means
a sign change in pitch and yaw between the two conventions. Yaw is aligned with true north.

Because of a limitation in how ROS sensor_msgs/Imu reports data, it can be unclear when viewing the
message specifications the frame respective of which orientation is reported. This ambiguity is caused
by not having a separate frame_id for orientation (which is measured w.r.t. a relatively fixed point) versus
rates and accelerations (which are measured w.r.t. the body frame), and is not present in nav_msgs/Odometry
which separates these two frames. Orientation data within sensor_msgs/Imu conforms to the same conventions
as nav_msgs/Odometry, and is globally fixed in the NED and ENU frames.

## Height Convention

The KVH reports height as the height above the WGS84 ellipsoid. This is the same height that
sensor_msgs/NavSatFix expects. Other types of height include height above mean sea level (MSL) or
height above ground (AGL) which is often used by aerial platforms.

# Contributing

Below are instructions on how to complete several common tasks that may be required depending on your uses.

## Quick Start Development

The KVH driver was written as a standalone driver, and the ROS node simply uses and recieves data from that driver. Below is the general structure of the ROS side. 

```cpp
    typedef std::pair<packet_id_e, int> freqPair;

    kvh::KvhPacketRequest packetRequest{
        freqPair(packet_id_utm_position, 50),
        freqPair(packet_id_system_state, 50),
        ... // Any additional packets
    };

    // Set connected port
    std::string kvhPort("/dev/ttyUSB0");

    kvhDriver.Init(kvhPort, packetRequest);

    system_state_packet_t sysPacket;

    while(1)
    {
        kvhDriver.Once(); // Check for new published packets

        if (kvhDriver.PacketIsUpdated(packet_id_system_state))
        {
            kvhDriver.GetPacket(packet_id_system_state, sysPacket);

            ... // Use System State Packet
        }
        kvhDriver.SetPacketUpdated(packet_id_system_state, false);
        usleep(10000); // Usually will want to sleep for some amount of time
    }

```
*Developer Note: Notice that at the end we must run `kvhDriver.SetPacketUpdated(packet_id, bool)`. The driver has been implemented so that you must explicitly state when you have read its data. If you wish to keep track of when new data appears, the driver must be used this way. In any case, the driver will always store the most recent packet of a specific type that it recieves and will set the update status to true.*


## Adding a new packet type
The KVH offers a wide variety of packets. As such we have only implemented the ones that we found most useful. If you have others that you need, please see the instructions below for a guide on how to add them to the driver. 

1. Add packet information to each set/map currently in `src/kvh_driver/kvh_global_vars.cpp`. 

For the most part, you should be able to follow the predefined pattern, but you will be adding the packet_id to the supportedPackets_, the packet size to packetSize_, and the string literal of the packet type to packetTypeStr_. Note, this is essentially registering your desired packet. 

*Developer Note: The packetSize_ and packetTypeStr_ could be inferred from the packet_id, except for the fact that some packets exist that were not properly implemented in kvh's api, and which we have had to manually extend. We have chosen to make these fixes in our code instead of modifying their api.*

2. Add packet-specific decoding function to **DecodePacket** function in `src/kvh_driver/decode_packets.cpp`

You should be able to just follow the pattern shown by the previously implemented packets, but will be included here for comprehensiveness.

```cpp
    case packet_id:
        packet_type_t packet;
        if (decode_packet_type(&packet, _anPacket) == 0)
        {
            packetStorage_.UpdatePacket(packet_type_id, packet);
            packetStorage_.SetPacketUpdated(packet_type_id, true);
        }
```

3. Add packet to initialisation mapping in the **Init** function of `src/kvh_driver/packet_storage.cpp`

The packet storage is initialised with the requested packets at the beginning. You must add a line similar to the following to make sure the initialization succeeds.

```cpp
    case packet_type_id:
      packetMap_[packet_type_id] = std::make_pair(false, std::make_shared<packet_type_struct_t>());
      break;
```

4. Add to the `KvhPacketRequest` that you are sending upon creation of the driver.

# Packaging

For information on packaging and releasing this package at MITRE, see PACKAGING.md

# Limitations
Here is just a list of some currently know limitations of the current architecture

1. Hard coded packets. It is currently not in the design to allow users to customize which packets they do and do not want to receive. 

2. Added complexity due to make changes to struct outside of API classes. Many of the steps shown in the *Adding a new packet type* section could be done automatically if we did not need to account for the possibility of extended structs. Currently the only example we have is the utm_fix struct type which extends the utm_position packet the kvh api has. We did this since we did not want to change their API in any way, but there may be a time the added complexity is not worth it. 


