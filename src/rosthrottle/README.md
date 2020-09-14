# rosthrottle
ROS Python package for throttling ROS nodes programatically in Python. Sits on top of [ros_comm](http://wiki.ros.org/ros_comm)
[topic_tools](http://wiki.ros.org/topic_tools) [throttle](http://wiki.ros.org/topic_tools/throttle) utility.

## Use
The rosthrottle package provides two types of throttlers: `MessageThrottle` and `Bandwidth Throttle`.

### `MessageThrottle`
Used to throttle a topic on a message frequency basis. Example:
```python
from rosthrottle import MessageThrottle

intopic = 'in'
outtopic = 'out_throttled'
rate = 1.0
bt = MessageThrottle(intopic, outtopic, rate)
# start throttler process
pid = bt.start()
# update throttler configuration
bt.update(rate=5.0)
# kill throttler
bt.stop()
```

### `BandwidthThrottle`
Used to throttle a topic on a bandwidth basis. Example:
```python
from rosthrottle import BandwidthThrottle

intopic = 'in'
outtopic = 'out_throttled'
bandwidth = 1024
window = 1.0
bt = BandwidthThrottle(intopic, outtopic, bandwidth, window)
# start throttler process
pid = bt.start()
# update throttler configuration
bt.update(bandwidth=100)
# kill throttler
bt.stop()
```

See `src/rosthrottle/tests/` for additional examples and documentation.
