# Testing

To run these tests, first set up the ROS environment by running three commands in separate terminals:
```bash
roscore                         # starts ros core (do this in a separate terminal)
./setup.sh                      # starts basic talker/listener nodes communicating over a topic to be throttled
python throttledListener.py     # starts listener node which is listening for throttled topics
```

## Throttle Testing
There are two main tests, one for each type of throttle. Test MessageThrottles with:

```bash
python message_test.py
```

Test BandwidthThrottes with:

```bash
python bandwidth_test.py
```

Both of these tests follow the same procedure:
1. Create throttle with initial parameters.
2. Start throttle.
3. Sleep 10 seconds.
4. Update throttle parameters.
5. Sleep 10 seconds.
6. Stop throttle and exit.

You can monitor the change in throttle parameters and their affects by watching the output of the throttledListener.py process.

## Cleanup Testing
To ensure rosthrottle is safe to use even if the program is killed or crashes it registers its own cleanup 
function to kill running throttles on exit. To test this functionality, run:

```bash
python cleanup_test.py
```

The cleanup test creates and starts a MessageThrottle, then spins, simply printing 'testing' once per second. You can
check running nodes with `rosnode list` to see the created throttling node. Then, kill the cleanup test using 
Ctrl + C (or any other kill command) and check that the throttling node is killed too.
