# mikrotik_swos_tools

Integration between ROS (Robot Operating System) and Mikrotik SwOS.

## Nodes

### swos_api

Gives access to the web management interface via ROS.

If connection to the switch fails, a `requests.exceptions.RequestException` python exception is thrown and the node exits - unless `wait_for_switch` is set to True.

#### Parameters

- `rate` (`double`): Rate at which to query the switch for updates (in seconds, default `0.1`).
- `username` (`string`): Username of the management user (default: `admin`).
- `password` (`string`): Password of the management user (default: empty string)
- `address` (`string`): URL of the management interface (default: `http://192.168.88.1/`)
- `wait_for_switch` (`bool`): If true, the script will wait for the switch management interface to become available before advertising its topics and services. If false, the script dies if the management interface is not accessible. Default is `False`.

#### Published topics

- `~statistics` (`mikrotik_swos_tools/Statistics`): Data from the Statistics tab as a ROS message. Published regularly on the given rate.