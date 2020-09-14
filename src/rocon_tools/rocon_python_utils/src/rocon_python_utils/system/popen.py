#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: system.popen
   :platform: Unix
   :synopsis: Workaround a few of the constrations in the official popen impl.

This module helps workaround the official popen with another implementation
that solves a few problems (post-exec callbacks, etc)

----

"""

##############################################################################
# Imports
##############################################################################

# system
import os
import threading
import subprocess
import signal

##############################################################################
# Subprocess
##############################################################################


class Popen(object):
    '''
      Use this if you want to attach a postexec function to popen (which
      is not supported by popen at all). It also defaults setting and
      terminating whole process groups (something we do quite often in ros).

      **Usage:**

      This is what you'd do starting a rosrunnable with a listener for
      termination.

      .. code-block:: python

          def process_listener():
              print("subprocess terminating")

          command_args = ['rosrun', package_name, rosrunnable_filename, '__name:=my_node']
          process = rocon_python_utils.system.Popen(command_args, postexec_fn=process_listener)

    '''
    __slots__ = [
            'pid',
            '_proc',
            '_thread',
            '_external_preexec_fn',
            '_shell',
            '_env',
            'terminate'
        ]

    def __init__(self, popen_args, shell=False, preexec_fn=None, postexec_fn=None, env=None):
        '''
          :param popen_args: list/tuple of usual popen args
          :type popen_args: str[]
          :param bool shell: same as the shell argument passed to subprocess.Popen

          :param preexec_fn: usual popen pre-exec function
          :type preexec_fn: method with no args

          :param postexec_fn: the callback which we support for postexec.
          :type postexec_fn: method with no args

          :param dict env: a customised environment to run the process in.
        '''
        self.pid = None
        self._proc = None
        self._shell = shell
        self._env = env
        self._external_preexec_fn = preexec_fn
        self._thread = threading.Thread(target=self._run_in_thread, args=(popen_args, self._preexec_fn, postexec_fn))
        self._thread.start()

    def send_signal(self, sig):
        """
        Send the process a posix signal. See `man 7 signal` for a list and pass
        them by keyword (e.g. signal.SIGINT) or directly by integer value.

        :param int sig: one of the posix signals.
        """
        os.killpg(self._proc.pid, sig)
        # This would be the normal way if not defaulting settings for process groups
        #self._proc.send_signal(sig)

    def _preexec_fn(self):
        """
        A default preexec function that is usually applicable in terminal
        launching situations since we need to take care of process groups.

        See http://stackoverflow.com/questions/3791398/how-to-stop-python-from-propagating-signals-to-subprocesses
        for some interesting information around this topic, specifically with
        resolving signal forwarding and use of preexec - there are some differences
        between 2.x and 3.2+ handling in subprocess.
        """
        os.setpgrp()
        if self._external_preexec_fn is not None:
            self._external_preexec_fn()

    def terminate(self):
        '''
          :raises: :exc:`.OSError` if the process has already shut down.
        '''
        return os.killpg(self._proc.pid, signal.SIGTERM)
        # if we were not setting process groups
        #return self._proc.terminate() if self._proc is not None else None

    def _run_in_thread(self, popen_args, preexec_fn, postexec_fn):
        '''
          Worker function for the thread, creates the subprocess itself.
        '''
        if preexec_fn is not None:
            if self._shell == True:
                #print("rocon_python_utils.os.Popen: %s" % " ".join(popen_args))
                self._proc = subprocess.Popen(" ".join(popen_args), shell=True, preexec_fn=preexec_fn, env=self._env)
            else:
                #print("rocon_python_utils.os..Popen: %s" % popen_args)
                self._proc = subprocess.Popen(popen_args, shell=self._shell, preexec_fn=preexec_fn, env=self._env)
        else:
            self._proc = subprocess.Popen(popen_args, shell=self._shell, env=self._env)
        self.pid = self._proc.pid
        self._proc.wait()
        if postexec_fn is not None:
            postexec_fn()
        return
