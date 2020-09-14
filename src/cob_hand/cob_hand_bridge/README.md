# Raspberry (1 or 2) setup

1. `sudo apt-get install git cmake g++ libboost-thread-dev`
2. `git clone --recursive https://github.com/ipa320/cob_hand.git`
3. `mkdir cob_hand/cob_hand_bridge/client/build`
4. `cd cob_hand/cob_hand_bridge/client/build`
5. `cmake ..`
6. `make`
7. `make install` *(OPTIONAL)*

# Raspberry usage

`cob_hand_bridge */dev/ttyToROS[@baudrate]* [looprate (default: 20Hz)]`

# Maintenance
## update ros_lib (on a ROS PC)
1. `cd cob_hand/cob_hand_bridge/client/`
2. `python make_library.py`

# ROS usage
Overlay for rosserial: `git clone https://github.com/ipa-mdl/rosserial.git -b indigo-devel`

Run `roslaunch cob_hand_bridge serial_bridge.launch port:=*/dev/ttyToRasp* [baud:=...]`

Publications:                                                                                                                                                                                                                                                                  
 * /cob_hand_bridge/status [cob_hand_bridge/Status]

Subscriptions: 
 * /cob_hand_bridge/set_pin [std_msgs/UInt8]: set single pin
 * /cob_hand_bridge/clear_pin [std_msgs/UInt8]: clear single pin
 * /cob_hand_bridge/command [cob_hand_bridge/JointValues]: motor command 

Services: 
 * /cob_hand_bridge/set_pwm [cob_hand_bridge/SetPWM]: set pwm output for multiple pins
 * /cob_hand_bridge/init_finger [cob_hand_bridge/InitFinger]: init SDHx
 * /cob_hand_bridge/halt [std_srvs/Trigger]: stop motor motion
 * /cob_hand_bridge/init_pins [cob_hand_bridge/InitPins]: set direction of pins, especially output/pwm pins
 * /cob_hand_bridge/update_pins [cob_hand_bridge/UpdatePins]: set and clear multiple ourput pins


