import time
from rosthrottle import MessageThrottle

if __name__=='__main__':
    intopic = 'chatter'
    outtopic = 'chatter_message_throttled'
    rate = 5.0
    t = MessageThrottle(intopic, outtopic, rate)
    pid = t.start()
    print('Throttle PID: ' + str(pid))
    while True:
        print('testing')
        time.sleep(1)
