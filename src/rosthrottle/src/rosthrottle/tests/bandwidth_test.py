import time
from rosthrottle import BandwidthThrottle

if __name__=='__main__':
    intopic = 'chatter'
    outtopic = 'chatter_bandwidth_throttled'
    bandwidth = 1024
    window = 1.0
    t = BandwidthThrottle(intopic, outtopic, bandwidth, window)
    pid = t.start()
    time.sleep(10)
    t.update(bandwidth=100)
    time.sleep(10)
    t.stop()
