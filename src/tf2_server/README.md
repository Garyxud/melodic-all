# tf2_server

An upgraded [tf2_ros/buffer_server](https://github.com/ros/geometry2/blob/melodic-devel/tf2_ros/src/buffer_server.cpp).
It supports the same API as the tf2_ros/buffer_server node. Moreover, it allows clients
to request only subparts of the TF tree, which results in lowering the data transfer rates.
This mode is useful as a middle way between standard TransformListener and the 
action interface of the buffer_server - it transmits only the requested transform(s),
but isn't burdened with the overhead the action interface brings in.

This package also provides helper libraries for use in C++ and Python clients
that make usage of this server easier.

## RequestTransformStream

The subtree subscription API is built around [ReuqestTranformStream](srv/RequestTransformStream.srv)
service type. The subtree listeners are configured by RequestTransformStreamRequest
objects. The meaning of the individual fields is as follows:

 - `string` `parent_frame`: The top-most frame in TF tree you're interested in.
 - `string[]` `child_frames`: In case a nonempty list is given, this specifies the
  child transforms you are interested in. They don't need to be direct children
  of `parent_frame`, but they should be in the same TF tree. If an empty list is
  given, stream the whole subtree of `parent_frame`. This mode requires 
  `intermediate_frames` set to `True`.
 - `bool` `intermediate_frames`: If `False`, stream only direct
 `parent_frame`->`child_frame` transforms (no matter if they were originally
 direct neighbors). If `True`, stream all the transforms between `parent_frame`
 and all `child_frames`.
 - `duration` `publication_period`: How often to publish the transforms.
 - `bool` `allow_transforms_update`: If true, periodically checks for updates in
   subtree topology. Can even handle cases when `parent_frame` isn't reachable
   at the time of the request. Default is false.
 - `int32` `publisher_queue_size`: Queue size of the transform publisher.
 - `string` `requested_topic_name`: If nonempty, the stream will be published on
   the given topic. Leads to an error if another stream is only registered on
   this topic with incompatible settings.
 - `string` `requested_static_topic_name`: If nonempty, specifies the name of
   the stream with static transforms. Defaults to `$(requested_topic_name)/static`.
 - `double` `initial_streams_wait_time`: Specifies how long to wait before 
   registering streams from parameter `streams` (in seconds).
 - `dict` `streams`: If nonempty, specifies some streams that should be registered
   right after starting the server. Each stream has a name (its key in this dict).
   The stream will be published at `tf2_server/stream_key` and 
   `tf2_server/stream_key/static`. The structure is the following:


    streams:
      body:
        parent_frame: 'base_link'
        child_frames: []
        intermediate_frames: True
        publication_period: 0.1
        publisher_queue_size: 11
 
## Example usage

### tf2_server.launch

```XML
<launch>
    <param name="buffer_size" value="30.0"/>
    <param name="publish_frame_service" value="true"/>
    <param name="use_node_namespace" value="true"/>
    <node name="tf_server" pkg="tf2_server" type="tf2_server_node">
        <param name="transforms_update_period" value="10.0" />
    </node>
</launch>
```
    
### C++ client

This C++ client code shows how to subscribe the whole TF subtree under the
`base_link` frame. The transforms in the buffer get updated once every 0.1 s.

```C++
#include <tf2_server/tf2_subtree_listener.h>

RequestTransformStreamRequest req;
req.parent_frame = "base_link";
req.child_frames = { };
req.intermediate_frames = true;
req.publisher_queue_size = 10;
req.publication_period = ros::Duration(0.1);

tf2_ros::Buffer buffer;
TransformSubtreeListener listener(req, buffer, false, ros::Duration(10));

...

buffer.canTransform("base_link", "left_track", ros::Time(0));
```
    
### Python client

This Python client example shows how to subscribe exactly two transforms:
`base_link`->`left_track` and `base_link`->`front_left_flipper_endpoint`.
No other transforms are transmitted to the client.

```Python
req = RequestTransformStreamRequest()
req.parent_frame = "base_link"
req.child_frames = ["left_track", "front_left_flipper_endpoint"]
req.intermediate_frames = False
req.publisher_queue_size = 10
req.publication_period = rospy.Duration(0.1)

buffer = Buffer()
listener = TransformSubtreeListener(req, buffer, max_server_wait=rospy.Duration(10))

...

buffer.can_transform("base_link", "left_track", rospy.Time(0))
```
    
## Principle of working of the subtree listener/publisher

The TransformSubtreeListener classes hide some implementation details from the
user. Here we describe what exactly happens when a subtree is requested.

First, service `tf2_server/request_transform_stream` is called with the given
subtree configuration. The server responds with a message that specifies
`topic_name` and `static_topic_name`, which are autogenerated topic names from
namespace `tf2_server/streams/*`. The client then remaps `/tf` and `/tf_static`
to these topics and subscribes to them.

On the server side, the autogenerated topics are cached and when a new client
asks for an already published subtree configuration, the already existing topic
names are returned. Each unique subtree configuration starts a timer in the
server node, which publishes the requested transforms on the requested rate.
When last client with a given configuration disconnects from its topics, the
timer is stopped so that it doesn't eat resources. But the topic remains and
anyone can connect later, which will start the timer again.  
