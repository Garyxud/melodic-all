Volksbot Driver
===============

ROS driver for the [volksbot robots](https://www.volksbot.de/). 

## Installation
  `sudo apt install ros-melodic-volksbot-driver`
  
## Volksbot URDF
See the unified robot description file in `urdf/volksbot.urdf.xacro` and the `urdf/example.urdf.xacro` file to use the 
volksbot base for your needs.

## Parameters
| parameter            | data type  | default value |
|----------------------|------------|---------------|  
| `wheel_radius`       | `double`   | `0.0985`      |
| `axis_length`        | `double`   | `0.41`        |
| `gear_ratio`         | `int`      | `74`          | 
| `turning_adaptation` | `double`   | `0.95`        |
| `x_stddev`           | `double`   | `0.002`       |
| `rotation_stddev`    | `double`   | `0.017`       |
| `cov_xy`             | `double`   | `value`       |
| `cov_xrotation`      | `double`   | `0.0`         |
| `cov_yrotation`      | `double`   | `0.0`         |
| `publish_tf`         | `bool`     | `False`       |
| `num_wheels`         | `int`      | `4`           |
| `joint_names`        | `str list` |               |

## Build Status

|        | Master | Melodic |
|--------|--------|---------|
| Travis | [![Build Status](https://api.travis-ci.org/uos/volksbot_driver.svg?branch=master)](https://travis-ci.org/uos/volksbot_driver) | [![Build Status](https://api.travis-ci.org/uos/volksbot_driver.svg?branch=melodic)](https://travis-ci.org/uos/volksbot_driver) |
| Binary Deb | | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__volksbot_driver__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__volksbot_driver__ubuntu_bionic_amd64__binary) |
| Source Deb | | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__volksbot_driver__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__volksbot_driver__ubuntu_bionic__source/) |
| Develop | | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__volksbot_driver__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__volksbot_driver__ubuntu_bionic_amd64) |
| Documentation | | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdoc__volksbot_driver__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdoc__volksbot_driver__ubuntu_bionic_amd64) |
