import rosbag_pandas


def test_topics_from_keys():
    topics = rosbag_pandas.topics_from_keys(["/pose/pose/position/x"])
    assert set(topics) == {'/pose', '/pose/pose', '/pose/pose/position'}
