# movie_publisher

This package contains several tools for using movie files in ROS (playback, conversion to bag files etc.).

It handles any file formats the system installation of ffmpeg can decode.

**Important: This package is meant to work with moviepy. However, due to packaging issues, moviepy cannot be installed
automatically as a dependency. There's a fallback using OpenCV, which is however worse. Please, install moviepy manually
calling:**

    sudo pip install moviepy

or

    rosdep install python-moviepy-pip

## Main tools

- `movie_publisher_node`: A ROS node that serves a video file as video topic source (`sensor_msgs/Image` and friends).
- `movie_publisher.launch`: A launch file for convenient usage of the node. It also allows starting an 
`image_transport/republish` node that converts the video stream from raw to compressed/theora.
In the `immediate` mode, it can also be used to convert video files to bagfiles in a batch.
- `movie_to_bag`: A batch script that takes a video as input and transforms it into a bag file.
- `add_movie_to_bag`: A batch script that adds a video as a topic into an existing bag file.

### Helper tools

- `fix_bag_timestamps`: A batch script that rewrites bag files so that the message publication time is taken from the 
message header. This allows you to generate bagfile data at high speed and then reprocess them to have more suitable
publication timestamps.
- `merge.py`: A copy of [srv_tools/merge.py](https://github.com/srv/srv_tools/blob/kinetic/bag_tools/scripts/merge.py) 
which is not available in indigo.

## movie_publisher_node

The node can run with either of two backends - `moviepy` and `opencv`. `moviepy` is strongly recommended, as it uses
`ffmpeg`, which is quite versatile and efficient. Is you do not set the `backend` param, autodetection is run.

### Published topics

- `movie` (`sensor_msgs/Image`): The published movie, in raw format.

### Node-private parameters:

- `movie_file` (string, required): Path to the movie to play. Any format that ffmpeg can decode.
- `fps` (float, optional): If set, the playback will be at the specified FPS (speeding up/slowing down the movie).
- `start` (float|tuple|string, optional): If set, playback will start from the specified time.
      Can be expressed in seconds `(15.35)`, in `(min, sec)`, in `(hour, min, sec)`,
      or as a string: `'01:03:05.35'`.
      Cannot be set together with `end` and `duration`.
- `end` (float|tuple|string, optional): If set, playback will stop at the specified time (not affected by start).
      Can be expressed in seconds `(15.35)`, in `(min, sec)`, in `(hour, min, sec)`,
      or as a string: `'01:03:05.35'`.
      Cannot be set together with `start` and `duration`.
- `duration` (float|tuple|string, optional): If set, playback will have this duration. If end is also set, the
      duration is counted from the end of the clip, otherwise, it is the duration from the start of the clip.
      Can be expressed in seconds `(15.35)`, in `(min, sec)`, in `(hour, min, sec)`,
      or as a string: `'01:03:05.35'`.
      Cannot be set together with `start` and `end`.
- `loop` (bool, default False): Whether to loop the movie until the node is shut down. Exludes `immediate`.
- `immediate` (bool, default False): If True, the movie will be processed and published as quickly as possible not
      waiting for the real time. The timestamps in the resulting messages act "real-world-like" (i.e. 15 FPS means
      the frames' timestamps will be 1/15 sec apart). You can set `fake_time_start` if you want these timestamps to
      begin from a non-zero time. Excludes `loop`.
- `playback_rate` (float, optional): If set to a number, immediate mode will not play as fast as possible, but at this
      rate (set the rate to a number where you do not lose any messages, e.g. in image_transport/republish).
- `fake_time_start` (float, default 0.0): Used with `immediate` to specify the timestamp of the first message.
- `frame_id` (string, default ""): The frame_id used in the messages' headers.
- `spin_after_end` (bool, default False): If True, a rospy.spin() is called after the movie has been published.
- `verbose` (bool, default False): If True, logs info about every frame played.
- `wait_after_publisher_created` (float, default 1.0): A workaround for the case where you need to give your
      subscribers some time after the publisher was created. Tweak this number until you get no missing start
      messages.
- `publisher_queue_size` (int, default 1000 in immediate mode, 10 otherwise): `queue_size` of the movie publisher.
- `backend` (string, default "moviepy"): The backend to use for reading video. Either `moviepy` or `opencv`. If
      `moviepy` is selected and not found, `opencv` will be used (which might support less codecs).
- `ffmpeg` (string, default ""): If nonempty, specifies the (absolute) path to the ffmpeg binary to use.

## movie_publisher.launch

This launch file takes arguments with the same name as the node's parameters.

Additionally, it takes these arguments:

- `transport` (string, default `'raw'`): Type of the image transport. The
      launch file will start up an `image_transport/republish` node that publishes the video
      encoded to this transport type.
- `republished_topic_basename` (string, default `movie_$(arg transport)`): Base name of the
      topic that will serve the republished messages. Full name of the topic will be
      `$(arg republished_topic_basename)/$(arg transport)` (or `$(arg republished_topic_basename)` in case of `raw` 
      transport). The base name cannot be the same as the topic the `movie_publisher_node` subscribes to (`movie` by 
      default).
      
## movie_to_bag

Convert a movie file to a bag file with video topic.

It is a Bash script with ROS node-like API - you pass it parameters via `_param:=value` on commandline or via ROS param
server. 

### Node-private parameters:

- `movie` (string): Path to the source movie file.
- `bag` (string): Path where the result should be stored.
- `topic` (string): name of the movie's topic in the bag file
- `overwrite_bag` (bool, default false): If true, overwrites existing `bag` file, otherwise exits with error if `bag` 
exists.
- `tmp_bag` (string, default `/tmp/movie.bag`): Path where a temporary bag file might be stored.
- `transport` (string, default `compressed`): Name of the image transport to use for encoding the image. Either `raw`,
`compressed`, `theora` or any other installed image transport.

### Usage

Call this script from commandline setting the node-private parameters, and pass any other `arg:=value` arguments - 
these will be relayed to `movie_publisher.launch` as is. Do not pass arguments that would collide with the node-private
parameters of this script (e.g. `movie_file`).

Example:

    rosrun movie_publisher movie_to_bag _movie:=movie.mp4 _bag:=movie.bag _topic:="/movie" start:=5 fake_time_start:=1548323340.24
    
## add_movie_to_bag

Add a movie file to an existing bagfile as a topic.

It is a Bash script with ROS node-like API - you pass it parameters via `_param:=value` on commandline or via ROS param
server. 
    
### Node-private parameters:

- `movie` (string): Path to the source movie file.
- `bag_in` (string): Path to the source bag file.
- `bag_out` (string, default: `output.bag`): Path where the result should be stored.
- `topic` (string): Name of the movie's topic in the bag file.
- `movie_delay` (int, default `0`): Delay (in seconds) of the movie file from the bag file start. May be negative.
- `overwrite_out_bag` (bool, default false): If true, overwrites existing `bag_out` file, otherwise exits with error if 
`bag_out` exists.
- `bag_tmp` (string, default `/tmp/movie_add_to.bag`): Path where a temporary bag file might be stored.
- `transport` (string, default `compressed`): Name of the image transport to use for encoding the image. Either `raw`,
`compressed`, `theora` or any other installed image transport.

### Usage

Call this script from commandline setting the node-private parameters, and pass any other `arg:=value` arguments - 
these will be relayed to `movie_publisher.launch` as is. Do not pass arguments that would collide with the node-private
parameters of this script (e.g. `movie_file`).

Example:

    rosrun movie_publisher add_movie_to_bag _movie:=movie.mp4 _bag_in:=movie_in.bag _bag_out:=movie_out.bag _topic:="/movie" start:=5 movie_delay:=-1
    
    
## fix_bag_timestamps

Goes through `in_bag` and for all messages with a header field sets their publication time to the time stored in their 
`header.stamp` plus `delay`. If `topics` are set, only works on messages on the listed topics. Reading timestamps from 
`/tf` is also supported.

It is a Python script with ROS node-like API - you pass it parameters via `_param:=value` on commandline. 
      
### Node-private parameters:

- `in_bag` (string): Path to the source bag file.
- `out_bag` (string): Path where the result should be stored.
- `topics` (string, default: `''`): If nonempty, rewrite only timestamps of the listed topics. Pass a comma-separated 
list of topics. 
- `delay` (int, default `0`): Delay (in seconds) which is added to the timestamp read from header. May be negative.
- `overwrite_existing` (bool, default false): If true, overwrites existing `out_bag` file, otherwise exits with error if 
`out_bag` exists.
