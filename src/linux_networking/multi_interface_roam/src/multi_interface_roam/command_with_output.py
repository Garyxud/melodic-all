#! /usr/bin/env python

import logging
import logging.handlers
import os
from twisted.internet import protocol, reactor
import signal

logging_enabled = False

# This is a workaround for older versions of twisted that use SIGCHLD, but
# do not set SA_RESTART.
def sa_restart_hack():
    import twisted.internet.base
    old_handleSignals = twisted.internet.base._SignalReactorMixin._handleSignals
    def _handleSignals(self):
        old_handleSignals(self)
        signal.siginterrupt(signal.SIGCHLD, False)
    twisted.internet.base._SignalReactorMixin._handleSignals = _handleSignals
sa_restart_hack()

class CommandWithOutput(protocol.ProcessProtocol):
    def __init__(self, args, name):
        self.restart_delay = 0.2
        self.logger = logging.getLogger(name)
        self.console_logger = logging.getLogger('console.%s'%name)
        if logging_enabled:
            logger_handler = logging.handlers.TimedRotatingFileHandler(os.path.join(logdir,'%s.log'%name), when='midnight', backupCount=logfilecount)
            logger_handler.setFormatter(file_formatter)
            self.logger.addHandler(logger_handler)
            self.console_logger.addHandler(logger_handler)
            logger_handler.setLevel(logging.DEBUG)
        self.logger.setLevel(logging.DEBUG)
        self.console_logger.setLevel(logging.DEBUG)
        self.proc_args = args
        self.outline = ""
        self.errline = ""
        self.shutting_down = False
        self.proc = None
        self.shutdown_trigger = reactor.addSystemEventTrigger('during', 'shutdown', self.shutdown)
        reactor.callWhenRunning(self.start_proc)
                            
    def errReceived(self, data):
        self.errline = self.data_received(self.errline, data)

    def outReceived(self, data):
        self.outline = self.data_received(self.outline, data)

    def data_received(self, curline, data):
        curline += data
        while True:
            splitpos = curline.find('\n')
            if splitpos == -1:
                break
            self.got_line(curline[:splitpos])
            curline = curline[splitpos+1:]
        return curline

    def processEnded(self, status_object):
        if self.outline: 
            self.got_line(self.outline)
        if self.errline:
            self.got_line(self.errline)
        if self.shutting_down:
            return
        self.console_logger.info("Process died, restarting: %s"%(" ".join(self.proc_args)))
        self.start_proc()
    
    def start_proc(self):
        try:
            self.proc = reactor.spawnProcess(self, self.proc_args[0], self.proc_args, None)
            self.child_restart()
        except OSError:
            self.console_logger.fatal("Error trying to run: %s"%(" ".join(self.proc_args)))

    def child_restart(self):
        pass # Can be overridden by derived classes.

    def _got_line(self, line):
        self.logger.info(line)
        try:
            self.got_line(line)
        except Exception, e:
            self.console_logger.fatal("Caught exception in CommandWithOutput.run: %s"%str(e))
            raise # FIXME Remove this?

    def shutdown(self):
        self.shutting_down = True
        try:
            reactor.removeSystemEventTrigger(self.shutdown_trigger)
        except ValueError:
            pass # We may have been called automatically at shutdown.
        if self.proc:
            self.proc.signalProcess("INT")

if __name__ == "__main__":
    import unittest
    from async_helpers import unittest_with_reactor, async_test
    from twisted.internet.defer import Deferred

    class CommandWithOutputTest(unittest.TestCase):
        @async_test
        def test_basic(self):
            """Runs a hello command, and checks that Hello gets read, and
            that child_restart gets called."""
            class Tst(CommandWithOutput):
                def __init__(self):
                    self.deferred = Deferred()
                    self.lines = []
                    self.starts = 0
                    CommandWithOutput.__init__(self, ['echo', 'Hello'], "test")

                def got_line(self, line):
                    self.lines.append(line)
           
                def child_restart(self):
                    if self.starts < 2:
                        self.starts += 1
                        return
                    self.shutdown()
                    self.deferred.callback(self.lines)

            lines = yield Tst().deferred
            self.assertEqual(lines, ['Hello', 'Hello'])

        @async_test
        def test_kill(self):
            class Tst(CommandWithOutput):
                def __init__(self):
                    self.deferred = Deferred()
                    self.count = 0
                    self.second_start = False
                    CommandWithOutput.__init__(self, ['yes', 'yes'], "test")

                def got_line(self, line):
                    self.count += 1
                    if self.count == 100:
                        self.deferred.callback(self.count)
                        self.shutdown()

            count = yield Tst().deferred
            self.assertEqual(count, 100)

        @async_test
        def test_restart(self):
            class Tst(CommandWithOutput):
                def __init__(self):
                    self.deferred = Deferred()
                    self.count = 0
                    self.second_start = False
                    self.restarts = 0
                    CommandWithOutput.__init__(self, ['yes', 'yes'], "test")

                def got_line(self, line):
                    self.count += 1
                    if self.count == 100:
                        self.proc.signalProcess("INT")
                    if self.restarts > 1 and not self.deferred.called:
                        self.deferred.callback(self.restarts)
                        self.shutdown()
                
                def child_restart(self):
                    self.restarts += 1

            count = yield Tst().deferred
            self.assertEqual(count, 2)
        

    def run_ros_tests():
        import rostest
        rostest.unitrun('multi_interface_roam', 'command_with_output', CommandWithOutputTest)
    
    unittest_with_reactor(run_ros_tests)
