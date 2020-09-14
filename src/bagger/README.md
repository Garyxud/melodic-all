# bagger

This package allows easy programmatic control of rosbag record processes.

## Nodes

The Bagger node was created to help compartmentalize the bagging of different publications at different / the same times, and to help keep bag sizes in check, which speeds up post-mission analysis.  It uses fork, exec, and kill to manage the lifecycle of various specified rosbag record processes

### Configuration

- profile_name_to_record_options_map
A map containing any number (over 0) of profile names and rosbag record options pairings.  Record options are arguments to be passed to the rosbag record process for that profile name.  Check out config/example_profiles.yaml for an example profile_name_to_record_options_map.

### Publications

- bagger/bag_states
A message (bagger/BaggingState) containing the names of all record profiles and their current bagging state (true / false)

### Subscriptions

- N/A

### Services

- bagger/set_bag_state
Service (bagger/SetBagState) for setting the bagging state for any record profile.  The requested profile name should be passed as the bag_profile_name variable of the service call.  If the state successfully changes to the requested one, returns true, else false.

### Utilities

In the scripts/ directory, there is a simple convenience script for reindexing / decompressing any rosbags that require either in a passed directory of bags.  pass the -h argument for more details


