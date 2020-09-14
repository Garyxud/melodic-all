#! /usr/bin/env python

import system
from netlink_monitor import netlink_monitor, IFSTATE
from state_publisher import CompositeStatePublisher
from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks, DeferredLock

# TODO Add tests.

class InterfaceUpper:
    def __init__(self, iface):
        self.iface = iface
        self._lock = DeferredLock()
        CompositeStatePublisher(lambda x: x, [
            netlink_monitor.get_state_publisher(iface, IFSTATE.PLUGGED),
            netlink_monitor.get_state_publisher(iface, IFSTATE.UP),
        ]).subscribe(self._cb)
        self._is_shutdown = False
        self.state = None
        reactor.addSystemEventTrigger('before', 'shutdown', self._shutdown)
   
    @inlineCallbacks
    def restart(self):
        yield self._lock.acquire()
        try:
            yield system.system('ifconfig', self.iface, '0.0.0.0')
            yield system.system('ifconfig', self.iface, 'down')
        finally:
            self._lock.release()

    @inlineCallbacks
    def _cb(self, old_state, new_state):
        plugged, up = new_state
        self.state = new_state
        if plugged and not up and not self._is_shutdown:
            yield self._lock.acquire()
            try:
                yield system.system('ifconfig', self.iface, 'up')
            finally:
                self._lock.release()

    @inlineCallbacks
    def _shutdown(self):
        self._is_shutdown = True
        if self.state:
            yield self.restart()
