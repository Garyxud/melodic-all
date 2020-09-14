#!/usr/bin/env python

import logging

import flatdict
import numpy as np
import pandas as pd
import rosbag
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary


class RosbagPandaException(Exception):
    pass


def topics_from_keys(keys):
    """
    Extracts the desired topics from specified keys
    :param Keys: List of desired keys
    :return: List of topics
    """
    topics = set()
    for key in keys:
        if not key.startswith("/"):
            key = "/" + key
        chunks = key.split("/")
        for i in range(2, len(chunks)):
            topics.add("/".join(chunks[0:i]))
    return list(topics)


def bag_to_dataframe(bag_name, include=None, exclude=None):
    """
    Read in a rosbag file and create a pandas data frame that
    is indexed by the time the message was recorded in the bag.

    :param bag_name: String name for the bag file
    :param include: None, or List of Topics to include in the dataframe
    :param exclude: None, or List of Topics to exclude in the dataframe (only applies if include is None)

    :return: a pandas dataframe object
    """
    logging.debug("Reading bag file %s", bag_name)

    bag = rosbag.Bag(bag_name)
    type_topic_info = bag.get_type_and_topic_info()
    topics = type_topic_info.topics.keys()

    # get list of topics to parse
    logging.debug("Bag topics: %s", topics)

    if not topics:
        raise RosbagPandaException("No topics in bag")

    topics = _get_filtered_topics(topics, include, exclude)
    logging.debug("Filtered bag topics: %s", topics)

    if not topics:
        raise RosbagPandaException("No topics in bag after filtering")

    df_length = sum([type_topic_info.topics[t].message_count for t in topics])

    index = np.empty(df_length)
    index.fill(np.NAN)
    data_dict = {}
    for idx, (topic, msg, t) in enumerate(bag.read_messages(topics=topics)):
        flattened_dict = _get_flattened_dictionary_from_ros_msg(msg)
        for key, item in flattened_dict.iteritems():
            data_key = topic + "/" + key
            if data_key not in data_dict:
                if isinstance(item, float) or isinstance(item, int):
                    data_dict[data_key] = np.empty(df_length)
                    data_dict[data_key].fill(np.NAN)
                else:
                    data_dict[data_key] = np.empty(df_length, dtype=np.object)
            data_dict[data_key][idx] = item
        index[idx] = t.to_sec()

    bag.close()

    # now we have read all of the messages its time to assemble the dataframe
    return pd.DataFrame(data=data_dict, index=index)


def _get_flattened_dictionary_from_ros_msg(msg):
    """
    Return a flattened python dict from a ROS message
    :param msg: ROS msg instance
    :return: Flattened dict
    """
    return flatdict.FlatterDict(convert_ros_message_to_dictionary(msg), delimiter="/")


def _get_filtered_topics(topics, include, exclude):
    """
    Filter the topics.
    :param topics: Topics to filter
    :param include: Topics to include if != None
    :param exclude: Topics to exclude if != and include == None
    :return: filtered topics
    """
    logging.debug("Filtering topics (include=%s, exclude=%s) ...", include, exclude)
    return [t for t in include if t in topics] if include is not None else \
        [t for t in topics if t not in exclude] if exclude is not None else topics
