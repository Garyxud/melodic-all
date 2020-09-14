# Continental's HFL110 ROS Driver
This package was designed to be a [Robotic Operating System (ROS)](https://index.ros.org/about/) driver for Continental's 3D Flash Lidar products.

**Supported platforms/releases**:
| Platform                                                   | ROS Release                                                    |
| ---------------------------------------------------------- | -------------------------------------------------------------- |
| [Ubuntu 16.04 Bionic](https://releases.ubuntu.com/16.04.4/) | [ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu) |
| [Ubuntu 18.04 Bionic](https://releases.ubuntu.com/18.04/) | [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu) |
| [Ubuntu 20.04 Bionic](https://releases.ubuntu.com/20.04/) | [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) |

**License**: BSD Two Clause License

Please [review the source code documentation](https://continental.github.io/hfl_driver/html/index.html) for more details on how the project is structured.

### Quickstart

[WIP] Install like any other ROS package:
```
sudo apt install ros-<ros-distro>-hfl-driver
```

### Getting Started
1. First, make sure your system is supported and already has ROS installed (see table above)
2. Go ahead and clone this repository into your `catkin_ws`. Read up on `catkin_ws` by [following this tutorial(http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
3. Within your `catkin_ws` directory, go ahead and compile the code:
    1. `catkin_make`
    2. **you may have to run [dos2unix](https://www.poftut.com/how-to-install-and-use-dos2unix-command-in-linux/) on the `hfl_driver/cfg/HFL.cfg` file in order to get it to compile**
4. After a successful compile, add the new HFL ROS packages to your environment:
```
echo "source <path/to/your/catkin_ws>/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
5. In two separate terminals, run the following commands:
      1. `roscore`
      2. `roslaunch hfl_driver hfl110dcu.launch`

[Parameters](http://wiki.ros.org/roslaunch/XML/arg) for hfl_driver launch file

| Parameter           | Description           | Default Values        |
| ------------------- | --------------------- |:---------------------:|
| camera_model        | HFL Model to launch   | hfl110dcu             |
| camera_version      | HFL Firmware version  | v1                    |
| camera_ip_address   | HFL IP address (IPv4) | 192.168.10.21         |
| frame_data_port     | HFL PCA Port          | 57410                 |
| computer_ip_address | Computer IPv4 Address | 192.168.10.5          |

**TIP**: check a launch files arguments before calling roslaunch to confirm you are passing the correct parameters. Use [wireshark](https://www.wireshark.org/) or another network tool to see if you are receiving packets.

Be sure to check the documentation website for more information.

## CPP static code analysis

ROS also comes with static code analysis support, therefore in order to run it for the hfl_driver package, type:
```bash
catkin_make run_tests roslint_<name_of_ros_package>
```
Where in this case you would put hfl_driver or udp_com in for <name_of_ros_package>.
This will output the errors and warnings on console. If more info is required see [this](http://wiki.ros.org/roslint).

### Authors
Many have contributed to this project beyond just the people listed here.
Thank you to those who have answered any questions, emails or supported the project in other ways.
Without you none of this would have been possible.
- Gerardo Bravo
- Evan Flynn
