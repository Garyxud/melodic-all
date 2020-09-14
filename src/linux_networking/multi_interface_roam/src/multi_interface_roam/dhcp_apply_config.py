#! /usr/bin/env python

import system
from twisted.internet.defer import inlineCallbacks, DeferredLock
from twisted.internet import reactor
from state_publisher import CompositeStatePublisher
from netlink_monitor import netlink_monitor, IFSTATE
import ip_rule

# FIXME Allow anybody to call shutdown.

class _DhcpSetterCommon:
    # Provides support common code for shutting down on shutdown, and
    # handles locking.
    def __init__(self):
        self._lock = DeferredLock()
        self.is_shutdown = False
        reactor.addSystemEventTrigger('before', 'shutdown', self._shutdown)
    
    @inlineCallbacks
    def _cb(self, old_state, new_state):
        if self.is_shutdown:
            return
        yield self._lock.acquire()
        try:
            yield self._locked_cb(old_state, new_state)
        finally:
            self._lock.release()
    
    @inlineCallbacks
    def _shutdown(self):
        #print "Shutdown", self
        yield self._cb(None, None)
        self.is_shutdown = True

class DhcpAddressSetter(_DhcpSetterCommon):
    def __init__(self, iface, state_pub):
        _DhcpSetterCommon.__init__(self)
        self.iface = iface
        self._iface_status_pub = netlink_monitor.get_status_publisher(iface)
        state_pub.subscribe(self._cb)

    @inlineCallbacks
    def _locked_cb(self, old_state, new_state):
        #print "Address", self.iface, new_state
        if new_state is None or old_state is not None:
            if self._iface_status_pub.get() >= IFSTATE.PLUGGED:
                yield system.system('ifconfig', self.iface, '0.0.0.0')

        if new_state is not None:
            ip = new_state['ip']
            ip_slashed = new_state['ip_slashed']
            yield system.system('ip', 'addr', 'add', ip_slashed, 'dev', self.iface)
            # Send both request and response gratuitous arp.
            yield system.system('arping', '-q', '-c', '1', '-A', '-I', self.iface, ip)
            yield system.system('arping', '-q', '-c', '1', '-U', '-I', self.iface, ip)

class DhcpRouteSetter(_DhcpSetterCommon):
    def __init__(self, iface, table, state_pub):
        _DhcpSetterCommon.__init__(self)
        self.iface = iface
        self.table = str(table)
        self._iface_status_pub = netlink_monitor.get_status_publisher(iface)
        CompositeStatePublisher(lambda (addr, dhcp): None if not addr else dhcp, [
            netlink_monitor.get_state_publisher(iface, IFSTATE.ADDR),
            state_pub
        ]).subscribe(self._cb)

    @inlineCallbacks
    def _locked_cb(self, old_state, new_state):
        if self._iface_status_pub.get() >= IFSTATE.PLUGGED:
            yield system.system('ip', 'route', 'flush', 'table', self.table, 'dev', self.iface)

        if new_state:
            gateway = new_state['gateway']
            ip = new_state['ip']
            network_slashed = new_state['network_slashed']
            yield system.system('ip', 'route', 'add', 'table', self.table, 'default', 'dev', self.iface, 'via', gateway, 'src', ip, 'onlink')
            yield system.system('ip', 'route', 'add', 'table', self.table, network_slashed, 'dev', self.iface, 'src', ip)

class DhcpSourceRuleSetter:
    def __init__(self, iface, table, priority):
        self._ip_rule = ip_rule.IpRule(priority)
        self.table = str(table)
        self.state_pub = self._ip_rule.state_pub
        netlink_monitor.get_state_publisher(iface, IFSTATE.ADDR).subscribe(self._cb)
        
    def _cb(self, old_state, new_state):
        if new_state:
            ip = new_state[0]
            self._ip_rule.set('table', self.table, 'from', ip+"/32")
        else:
            self._ip_rule.set()
