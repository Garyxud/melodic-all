#! /usr/bin/env python

import roslib; roslib.load_manifest('multi_interface_roam')
from state_publisher import StatePublisher, CompositeStatePublisher
from event import Event
import network_monitor_udp.udpmoncli as udpmoncli
from netlink_monitor import IFSTATE, netlink_monitor
from twisted.internet import reactor
import config
import time

class PingTester:
    def __init__(self, iface, rate, pingtarget, state_pub):
        self.update_event = Event()
        reactor.addSystemEventTrigger('before', 'shutdown', self._shutdown)
        self.udp_monitor = udpmoncli.MonitorClient([config.get_parameter('ping_max_latency', 0.2)], pingtarget, rate, 32, True)
        CompositeStatePublisher(lambda (addr, ready): None if not ready else addr, [
            netlink_monitor.get_state_publisher(iface, IFSTATE.ADDR),
            state_pub,
        ]).subscribe(self._addr_cb)

    def update(self, update_rate):
        result = self.udp_monitor.get_smart_bins(update_rate)
        self.update_event.trigger(result)
        self.state = result
        return result

    def _addr_cb(self, old_state, new_state):
        if new_state:
            # FIXME This should only happen after routes are set up.
            #print "New address", new_state
            self.udp_monitor.start_monitor((new_state[0], 0))
            print "Starting monitor on %s"%new_state[0]
        else:
            self.udp_monitor.stop_monitor()
            self.state = None
            self.update_event.trigger(None)

    def _shutdown(self):
        try:
            self.udp_monitor.shutdown()
        except:
            import sys
            sys.print_exc()

class PingMonitor:
    def __init__(self, iface):
        self.iface = iface
        self.ping_timeout = config.get_parameter('ping_timeout', 4)
        self.is_verified = StatePublisher(False)
        self.has_address = False
        iface.ping_tester.update_event.subscribe_repeating(self._update)
        netlink_monitor.get_state_publisher(iface.iface, IFSTATE.ADDR).subscribe(self._has_address_cb, iface)

    def _update(self, status):
        #print "PingMonitor._update"
        if status:
            (bins, latency1, latency2) = status
            #print "PingMonitor", bins, self.restart_time - time.time()
        if status and bins[0]:
            self.restart_time = time.time() + self.ping_timeout
            self.is_verified.set(True)
        elif self.has_address and self.restart_time < time.time():
            if not config.get_parameter('disable_ping_timeout'):
                print "PingMonitor restarting", self.iface.iface
                self.has_address = False
                self.iface.interface_upper.restart()
            else:
                print "PingMonitor restart was disabled by disable_ping_timeout", self.iface.iface
                self._set_timeout()

    def _has_address_cb(self, iface, old_state, new_state):
        self.is_verified.set(False)
        self.has_address = bool(new_state)
        if new_state:
            self._set_timeout()
        else:
            self.restart_time = 1e1000

    def _set_timeout(self):
        self.restart_time = time.time() + self.ping_timeout
