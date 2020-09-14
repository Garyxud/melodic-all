#! /usr/bin/env python

import command_with_output
import state_publisher
import threading
import time
import traceback
import pythonwifi.iwlibs
import async_helpers
from twisted.internet.defer import inlineCallbacks

# TODO 
# Make this autoshutdown when there are no references.

# FIXME Move this elsewhere.
import subprocess
class RunCommand:
    def __init__(self, *args):
        proc = subprocess.Popen(list(args), stdout = subprocess.PIPE, stderr = subprocess.PIPE, close_fds = True)
        (self.stdout, self.stderr) = proc.communicate()

class IFSTATE:
    PLUGGED    = 0
    UP         = 1
    LINK       = 2
    LINK_ADDR  = 3
    ADDR       = 4
    NUM_STATES = 5

class NetlinkMonitor(command_with_output.CommandWithOutput):
    def __init__(self):
        self.lock = threading.RLock()
        self.raw_state_publishers = [ {} for i in range(0, IFSTATE.NUM_STATES)]
        self.state_publishers = [ {} for i in range(0, IFSTATE.NUM_STATES)]
        self.status_publishers = {}
        self.cur_iface = None
        self.deleted = None
        command_with_output.CommandWithOutput.__init__(self, ['ip', 'monitor', 'link', 'addr'], 'ip_monitor')

    @inlineCallbacks
    def child_restart(self):
        yield async_helpers.async_sleep(0.2) # Limit race conditions on getting the startup state.
        current_state = RunCommand('ip', 'addr')
        with self.lock:
            old_cur_iface = self.cur_iface
            old_deleted = self.deleted
            for line in current_state.stdout.split('\n'):
                self.got_line(line)
            self.deleted = old_deleted
            self.cur_iface = old_cur_iface

    def get_state_publisher(self, interface, level):
        if level == 0:
            return self.get_raw_state_publisher(interface, level)
        pubs =  self.state_publishers[level]
        if not interface in pubs:
            pubs[interface] = state_publisher.CompositeStatePublisher(lambda l: l[0] and l[1], [
                    self.get_state_publisher(interface, level - 1),
                    self.get_raw_state_publisher(interface, level), 
                    ])
        return pubs[interface]
    
    def get_raw_state_publisher(self, interface, level):
        pubs =  self.raw_state_publishers[level]
        if not interface in pubs:
            pubs[interface] = state_publisher.StatePublisher(False)
            #pubs[interface].subscribe(self.debug_print, interface, level)
        return pubs[interface]

    def debug_print(self, interface, level, old_state, new_state):
        print "Netlink transition", interface, level, "from", old_state, "to", new_state

    def get_status_publisher(self, interface):
        if not interface in self.status_publishers:
            def lowest_nonzero(args):
                best = -1
                for arg in args:
                    if not arg:
                        break
                    else:
                        best += 1
                return best
            raw_pubs = [ self.get_raw_state_publisher(interface, level) for level in range(IFSTATE.NUM_STATES) ]
            self.status_publishers[interface] = state_publisher.CompositeStatePublisher(lowest_nonzero, raw_pubs)
        return self.status_publishers[interface]

    def got_line(self, line):
        with self.lock:
            try:
                # Figure out the interface, and whether this is a delete
                # event.
                
                if len(line) == 0 or (line[0] == ' ' and self.cur_iface == None):
                    return
                tokens = line.rstrip().split()
                link_info = False
                if line[0] != ' ':
                    if tokens[0] == 'Deleted':
                        self.deleted = True
                        tokens.pop(0)
                    else:
                        self.deleted = False
                    self.cur_iface = tokens[1].rstrip(':')
                    if tokens[1][-1] == ':':
                        link_info = True
                    tokens.pop(0)
                    tokens.pop(0)

                if link_info:
                    # Plugged or not?
                    self.get_raw_state_publisher(self.cur_iface, IFSTATE.PLUGGED).set(not self.deleted)
                    
                    # Up or not?
                    flags = tokens[0].strip('<>').split(',')
                    self.get_raw_state_publisher(self.cur_iface, IFSTATE.UP).set('UP' in flags)

                    # Have a link?
                    try:
                        state_idx = tokens.index('state')
                        state = tokens[state_idx + 1]
                        if state != 'DOWN':
                            try:
                                link_state = pythonwifi.iwlibs.Wireless(self.cur_iface).getAPaddr()
                            except IOError, e:
                                if e.errno == 95 or e.errno == 22:
                                    link_state = 'Wired'
                                else:
                                    raise
                        else:
                            link_state = False
                        self.get_raw_state_publisher(self.cur_iface, IFSTATE.LINK).set(link_state)
                    except ValueError:
                        pass # Sometimes state is not listed.
                
                else:
                    # Find the address.
                    if tokens[0] == 'inet':
                        if self.deleted:
                            addr_state = False
                        else:
                            addr_state = tokens[1].split('/')
                        self.get_raw_state_publisher(self.cur_iface, IFSTATE.ADDR).set(addr_state)

                    if tokens[0].startswith('link/') and len(tokens) > 1:
                        if self.deleted:
                            addr_state = False
                        else:
                            addr_state = tokens[1]
                        self.get_raw_state_publisher(self.cur_iface, IFSTATE.LINK_ADDR).set(addr_state)

            except Exception, e:
                print "Caught exception in NetlinkMonitor.run:", e
                traceback.print_exc(10)
                print

monitor = netlink_monitor = NetlinkMonitor()

def print_status(iface):
    from twisted.internet import reactor
    print monitor.get_status_publisher(iface).get(), ' : ',
    for i in range(0,IFSTATE.NUM_STATES):
        print monitor.get_raw_state_publisher(iface, i).get(),
        print monitor.get_state_publisher(iface, i).get(), '  /  ',
    print
    reactor.callLater(1, print_status, iface)

def main():
    from twisted.internet import reactor
    iface = 'wlan0'
    try:
        print_status(iface)
        reactor.run()
    except KeyboardInterrupt:
        print "Shutting down on CTRL+C"
        #monitor.shutdown()

if __name__ == "__main__":
    main()
