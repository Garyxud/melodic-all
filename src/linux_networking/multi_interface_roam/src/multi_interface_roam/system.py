#! /usr/bin/env python

import logging
import logging.handlers
import os
from twisted.internet import protocol, reactor
from twisted.internet.defer import Deferred, inlineCallbacks
import sys
import command_with_output # There is a SIGCHLD hack in there that we want to run

class Quiet:
    pass

class AutoShutdown:
    pass

class System(protocol.ProcessProtocol):
    def __init__(self, *args):
        self.deferred = Deferred()
        self.quiet = False
        autoshutdown = False
        while True:
            if args[0] == Quiet:
                self.quiet = True
            elif args[0] == AutoShutdown:
                autoshutdown = True
            else:
                break
            args = args[1:]
        self.proc = None
        self.args = args
        self.shutdown_deferreds = []
        if autoshutdown:
            self.shutdown_trigger = reactor.addSystemEventTrigger('before', 'shutdown', self._shutdown)
        else:
            self.shutdown_trigger = None
        self.proc = reactor.spawnProcess(self, args[0], args, None)
    
    def errReceived(self, data):
        if not self.quiet:
            print >> sys.stderr, data

    def outReceived(self, data):
        if not self.quiet:
            print >> sys.stdout, data

    def processEnded(self, status_object):
        if self.shutdown_trigger:
            reactor.removeSystemEventTrigger(self.shutdown_trigger)
            self.shutdown_trigger = None
        self.deferred.callback(status_object.value.exitCode)
        for d in self.shutdown_deferreds:
            d.callback(status_object.value.exitCode)

    def _shutdown(self):
        """Called at system shutdown."""
        self.shutdown_trigger = None
        return self.shutdown()

    def shutdown(self):
        """Call this to interrupt the program. Returns a deferred that will
        be called when shutdown is complete."""
        d = Deferred()
        self.shutdown_deferreds.append(d)
        if self.proc:
            print "Killing command:", self.args
            self.proc.signalProcess("INT")
        else:
            d.callback()
        return d

def system(*args):
    debug = False
    if debug:
        import time
        print time.time(), "system: ", args
    #print "system: ", args
    s = System(*args)
    if debug:
        def printout(value):
            print time.time(), "system done: ", args
            return value
        s.deferred.addCallback(printout)
    return s.deferred

if __name__ == "__main__":
    import unittest
    from async_helpers import unittest_with_reactor, async_test
    import time

    class SystemTest(unittest.TestCase):
        @async_test
        def test_basic(self):
            """Runs a simple command."""
            retval = yield system('echo', 'Hello')
            self.assertEqual(retval, 0)

        @async_test
        def test_retval(self):
            """Checks that return values work."""
            retval = yield system('sh', '-c', 'exit 42')
            self.assertEqual(retval, 42)

        @async_test
        def test_no_return_early(self):
            """Checks that System waits until termination."""
            start = time.time()
            retval = yield system('sleep', '1')
            duration = time.time() - start
            self.assertEqual(retval, 0)
            self.assertTrue(duration >= 1, "%s > 1 is false"%duration)

    def run_ros_tests():
        import rostest
        rostest.unitrun('multi_interface_roam', 'system', SystemTest)
    
    unittest_with_reactor(run_ros_tests)
