# rosbag_pandas

[![Build Status](https://travis-ci.org/eurogroep/rosbag_pandas.svg?branch=master)](https://travis-ci.org/eurogroep/rosbag_pandas)

Python library (and some tools) for converting [ROS bagfiles](http://wiki.ros.org/rosbag) to [Pandas dataframes](https://pandas.pydata.org/).

## Python library

```python
import rosbag_pandas

# Convert a ROSBag to a dataframe
df = rosbag_pandas.bag_to_dataframe('data/rosout.bag')
df_exclude = rosbag_pandas.bag_to_dataframe('data/example.bag', exclude=['/scan']))
df_include = rosbag_pandas.bag_to_dataframe('data/rosout.bag', include=['/rosout']))

# Select a dataframe key based on topic and (conform msgevalgen pattern http://docs.ros.org/api/rostopic/html/)
print(df['/rosout/header/stamp/secs'].to_string())

# Obtain possible ROS topics from a selection pattern (conform msgevalgen pattern http://docs.ros.org/api/rostopic/html/)
# This will return the possible topics: /pose, /pose/pose, /pose/pose/position
rosbag_pandas.topics_from_keys(["/pose/pose/position/x"])
```

## Key definition

Key definition conform the msgevalgen pattern http://docs.ros.org/api/rostopic/html/). Example:

```
/pose/pose/position/x
```

This will select the `/pose/position/x` property of topic `/pose` in the message of type http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html.

## Scripts

### bag_csv

Convert a ROS bag file to a CSV file:
```
usage: bag_csv [-h] [-b BAG] [-i [INCLUDE [INCLUDE ...]]]
               [-e [EXCLUDE [EXCLUDE ...]]] [-o OUTPUT] [-v]

Script to parse bagfile to csv file

optional arguments:
  -h, --help            show this help message and exit
  -b BAG, --bag BAG     Bag file to read
  -i [INCLUDE [INCLUDE ...]], --include [INCLUDE [INCLUDE ...]]
                        List for topics to include
  -e [EXCLUDE [EXCLUDE ...]], --exclude [EXCLUDE [EXCLUDE ...]]
                        List for topics to exclude
  -o OUTPUT, --output OUTPUT
                        name of the output file
  -v, --verbose         Log verbose
```

### bag_plot

Plot a key (or multiple keys) in a ROS bag file:
```
usage: bag_plot [-h] -b BAG -k [KEY [KEY ...]] [-y  YLIM YLIM] [-c] [-v]

Bagfile key to graph

optional arguments:
  -h, --help            show this help message and exit
  -b BAG, --bag BAG     Bag file to read
  -k [KEY [KEY ...]], --key [KEY [KEY ...]]
                        Key you would like to plot
  -y  YLIM YLIM, --ylim YLIM YLIM
                        Set min and max y lim
  -c, --combined        Graph them all on one
  -v, --verbose         Log verbose
```

### bag_print

Print a key (or multiple keys) in a ROS bag file:
```
usage: bag_print [-h] -b BAG -k [KEY [KEY ...]] [-v]

Print one or multiple bag keys

optional arguments:
  -h, --help            show this help message and exit
  -b BAG, --bag BAG     Bag file to read
  -k [KEY [KEY ...]], --key [KEY [KEY ...]]
                        Key you would like to print
  -v, --verbose         Log verbose
```
