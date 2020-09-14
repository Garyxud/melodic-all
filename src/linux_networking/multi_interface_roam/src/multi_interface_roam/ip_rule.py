#! /usr/bin/env python

from twisted.internet.defer import inlineCallbacks, DeferredLock
from twisted.internet import reactor
import state_publisher
import system

class IpRule:
    """Manage the ip rules for a given priority level.
    
    There are two intended uses:
    
    1. There is a single dynamic rule at that priority level. Use set to
    change it.
    
    2. There are multiple static rules at that level. Use add to add each
    one of them, and flush to remove them all.
    """
    def __init__(self, priority):
        self._lock = DeferredLock()
        self.priority = str(priority)
        self.is_shutdown = False
        self.state_pub = state_publisher.StatePublisher(())
        self.flush()
        reactor.addSystemEventTrigger('before', 'shutdown', self._shutdown)

    @inlineCallbacks
    def flush(self):
        self.prev = () # Won't hurt to do it here, just means we might not
                       # delete during ongoing set. Ensures it is set at
                       # end of construction.
        yield self._lock.acquire()
        try:
            retcode = None
            while not retcode:
                retcode = yield system.system(system.Quiet, 'ip', 'rule', 'del', 'priority', self.priority)
        finally:
            self._lock.release()
    
    @inlineCallbacks
    def _set_or_add(self, remove_old, args):
        """Changes/adds the rule. Call with no arguments to clear the rule."""
        if self.is_shutdown:
            return
        yield self._lock.acquire()
        try:
            # Are we replacing a rule with itself?
            if remove_old and self.prev and args == self.prev:
                return
            # Add new rule
            if args:
                yield system.system('ip', 'rule', 'add', "priority", self.priority, *args)
            # Add remove old rule
            if self.prev and remove_old:
                yield system.system('ip', 'rule', 'del', "priority", self.priority, *self.prev)
            self.prev = args
            self.state_pub.set(args)

        finally:
            self._lock.release()

    def set(self, *args):
        """Changes/adds the rule. Call with no arguments to clear the rule."""
        self._set_or_add(True, args)

    def add(self, *args):
        """Adds a rule without removing the existing one."""
        self._set_or_add(False, args)

    @inlineCallbacks
    def _shutdown(self):
        yield self.flush()
        self.is_shutdown = True
