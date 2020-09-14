#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: system.pid
   :platform: Unix
   :synopsis: Helpers for working with system pids.

This module includes a few helpers to enable working with system pids in python.

----

"""

##############################################################################
# Imports
##############################################################################

# system
import os
import time
import errno

# local
from rocon_python_utils.exceptions import TimeoutExpiredError

##############################################################################
# PID
##############################################################################


def pid_exists(pid):
    """
    Check whether pid exists in the current process table.

    :param int pid:
    :returns: true or false depending on its existence.
    :rtype: bool
    """
    if pid < 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError, e:
        return e.errno == errno.EPERM
    else:
        return True


def wait_pid(pid, timeout=None):
    """Wait for process with pid 'pid' to terminate and return its
    exit status code as an integer.

    If pid is not a children of os.getpid() (current process) just
    waits until the process disappears and return None.

    If pid does not exist at all return None immediately.

    :param int pid:
    :param float timeout: timeout in seconds

    :raises: :exc:`.TimeoutExpiredError` on timeout expired (if specified).
    """
    def check_timeout(delay):
        if timeout is not None:
            if time.time() >= stop_at:
                raise TimeoutExpiredError
        time.sleep(delay)
        return min(delay * 2, 0.04)

    if timeout is not None:
        waitcall = lambda: os.waitpid(pid, os.WNOHANG)
        stop_at = time.time() + timeout
    else:
        waitcall = lambda: os.waitpid(pid, 0)

    delay = 0.0001
    while 1:
        try:
            retpid, status = waitcall()
        except OSError, err:
            if err.errno == errno.EINTR:
                delay = check_timeout(delay)
                continue
            elif err.errno == errno.ECHILD:
                # This has two meanings:
                # - pid is not a child of os.getpid() in which case
                #   we keep polling until it's gone
                # - pid never existed in the first place
                # In both cases we'll eventually return None as we
                # can't determine its exit status code.
                while 1:
                    if pid_exists(pid):
                        delay = check_timeout(delay)
                    else:
                        return
            else:
                raise
        else:
            if retpid == 0:
                # WNOHANG was used, pid is still running
                delay = check_timeout(delay)
                continue
            # process exited due to a signal; return the integer of
            # that signal
            if os.WIFSIGNALED(status):
                return os.WTERMSIG(status)
            # process exited using exit(2) system call; return the
            # integer exit(2) system call has been called with
            elif os.WIFEXITED(status):
                return os.WEXITSTATUS(status)
            else:
                # should never happen
                raise RuntimeError("unknown process exit status")
