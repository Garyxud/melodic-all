import subprocess 
import signal
import os
import atexit
from abc import ABCMeta, abstractmethod

COMMAND_BASE = ['rosrun', 'topic_tools', 'throttle']
    
class AbstractThrottle(object):
    """Represents an abstract throttler. Should not be used externally.

    Attributes:
        intopic (str): topic to be throttled
        outtopic (str): topic to publish throttled output to
        process (Popen): topic_tools throttling process if running, otherwise None
    """
    __metaclass__ = ABCMeta

    def __init__(self, intopic, outtopic):
        self._intopic = intopic
        self._outtopic = outtopic
        self._process = None
        atexit.register(self._cleanup)

    def stop(self):
        """Stops the running throttler.
        
        Note that this function does nothing if the throttle is not running.

        Returns:
            int: process return code, or None if there is no runnning throttler
        """
        if self.process is not None:
            self.process.send_signal(signal.SIGINT)
            return_code = self.process.wait()
            self.process = None
            return return_code
        return None
    
    def update(self, **kwargs):
        """Provides a simple way of updating a running throttler. Can be 
        used instead of calling stop(), updating fields, then calling start() again. 

        Note that this function does nothing if the throttle is not running.
        
        Returns:
            int: process ID, or None if there is no process to update
        """
        if self.process is not None:
            self.stop()
            for field in kwargs.keys():
                setattr(self, field, kwargs[field])
            return self.start()
        return None
    
    @abstractmethod
    def start(self):
        """Starts the throttler.

        Note that this function should do nothing if the throttle is 
        already running.
        
        Returns:
            int: process ID of the throttling node. 
        """
        pass
        
    def _cleanup(self):
        """Cleans up the throttling process on exit.
        
        This function is registered with atexit on throttle creation.
        """
        self.stop()
    
    @property
    def intopic(self):
        return self._intopic
    
    @intopic.setter
    def intopic(self, i):
        self._intopic = i
    
    @property
    def outtopic(self):
        return self._outtopic
    
    @outtopic.setter
    def outtopic(self, o):
        self._outtopic = o
    
    @property
    def process(self):
        return self._process
    
    @process.setter
    def process(self, p):
        self._process = p
        
class MessageThrottle(AbstractThrottle):
    """Represents a message throttler.
    
    Args:
        intopic: string name of topic to throttle
        outtopic: string name of topic to republish on
        rate: desired rate of messages (Hz) on outtopic

    Attributes:
        rate: desired rate of messages (Hz) on outtopic
    """

    def __init__(self, intopic, outtopic, rate):
        super(MessageThrottle, self).__init__(intopic, outtopic)
        self._rate = rate
        
    def start(self):
        """Starts the throttler.

        Returns:
            int: process PID, or None if throttler is already running
        """
        if self.process is None:
            command = COMMAND_BASE + ['messages', self.intopic, str(self.rate), self.outtopic]
            self.process = subprocess.Popen(command, stdout=open(os.devnull, 'wb'))
            return self.process.pid
        return None
        
    @property
    def rate(self):
        return self._rate
    
    @rate.setter
    def rate(self, r):
        self._rate = r
    
class BandwidthThrottle(AbstractThrottle):
    """Represents a bandwidth throttler.
    
    Args:
        intopic: string name of topic to throttle
        outtopic: string name of topic to republish on
        bandwidth: desired throughput in bytes per second on outtopic
        window: throughput averaging window

    Attributes:
        bandwidth: desired throughput in bytes per second on outtopic
        window: throughput averaging window
    """

    def __init__(self, intopic, outtopic, bandwidth, window):
        super(BandwidthThrottle, self).__init__(intopic, outtopic)
        self._bandwidth = bandwidth
        self._window = window
    
    def start(self):
        """Starts the throttler.
        
        Returns:
            int: process PID, or None if throttler is already running
        """
        if self.process is None:
            command = COMMAND_BASE + ['bytes', self.intopic, str(self.bandwidth), str(self.window), self.outtopic]
            self.process = subprocess.Popen(command, stdout=open(os.devnull, 'wb'))
            return self.process.pid
        return None
        
    @property
    def bandwidth(self):
        return self._bandwidth
    
    @bandwidth.setter
    def bandwidth(self, b):
        self._bandwidth = b
    
    @property
    def window(self):
        return self._window
    
    @window.setter
    def window(self, w):
        self._window = w
