import rosbag_pandas


def test_numbers_3_4():
    df = rosbag_pandas.bag_to_dataframe('data/rosout.bag')
    assert set(df) == {'/rosout/file', '/rosout/function', '/rosout/header/frame_id', '/rosout/header/seq',
                       '/rosout/header/stamp/nsecs', '/rosout/header/stamp/secs', '/rosout/level', '/rosout/line',
                       '/rosout/msg', '/rosout/name', '/rosout/topics/0'}
