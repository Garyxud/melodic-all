# radar_omnipresense, a package for ROS
A Robotic Operating System (ROS) publisher and service for the OmniPreSense short range radar.
 
### Package Dependencies
 You will need to download and install:
* [rapidJson]( https://github.com/Tencent/rapidjson)

in order to successfully build the package with the ROS development tools.

In order to run (and build) this package, a system has to have the at least the ROS-base package.
* [ROS]( http://www.ros.org/install/ )

You can test if you have ROS installed correctly by attempting to run roscore, a required program for all subsequent examples.  If roscore is already running, it is safe to run roscore again and ignore any warnings about it already running. 
```
roscore
```

### Running the package with roslaunch
To run the package you will need to ensure that you have at least one OmniPreSense short range radar device plugged into your USB port(s).  Once this is done, use the roslaunch command in a shell terminal.  To do so, enter the following command. Note:  (This was developed on the Lunar release of ROS on a Linux machine, and tested with the ROS Kinetic release on Raspberry Pis running Ubuntu.  We recommend installing the ROS Kinetic release or later on a Linux machine.) 
```
roslaunch radar_omnipresense single_radar.launch
```	
or, if you have multiple radar modules, edit launch/multi_radar.launch as required and enter
```
roslaunch radar_omnipresense multi_radar.launch
```

This will run the package on the topics as declared in the .launch file you specified.  (For example, "radar_1")

Note: roslaunch stays running.  You will need to have another shell open to enter other commands while the radar_omnipresense node is running.  The unix utility "screen" is a very valuable utility if you are accessing the ROS linux instance over ssh, or more generally do not have the ability to have multiple terminals.

To view these topics please type the following command into a terminal.
```
rostopic echo /radar_report
```
The above comman will for if you are specifically using the single_radar.launch file. If you are using the multi_radar. launch file, use the following commands. 
```	
rostopic echo /radar_1/radar_report
```	
Then, if running a second radar module, in another terminal please type the following command to view the topic that the second radar is publishing to.
```
rostopic echo /radar_2/radar_report
```

Note: If you only have one radar device but use the multi_radar.launch file, the roslaunch command will still work, but you will see "error 2 opening /dev/ttyACM#", and # will be the port the device is connected to. The one radar will still publish data to the topic and the associated service can still be used.

### Running the ROS service to activate and deactivate FFT output
By default, the OmniPreSense radar sensor reports the speed of the most interesting target (which refers to the strongest bounced signal).
If you would like for the radar devices to output the FFT data, utilize a "ROS service call" to the module, after you have executed the previous commands listed above.  We recommend the following commands be typed into a new terminal. 
```
rosservice call /radar_1/send_api_command "command: 'OF'"
```
This will enable FFT data to be sent from the radar device.  You can see the data published to the topic by the command
```
rostopic echo /radar_1/radar_report
```

If running more than one sensor, substitute _1 with the appropriate number for each additional sensor when issuing more commands.







