import time
from rosthrottle import MessageThrottle

if __name__=='__main__':
    intopic = 'chatter'
    outtopic = 'chatter_message_throttled'
    rate = 1.0
    t = MessageThrottle(intopic, outtopic, rate)
    pid = t.start()
    time.sleep(10)
    t.update(rate=5.0)
    time.sleep(10)
    t.stop()
