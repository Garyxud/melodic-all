#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Description
##############################################################################

"""
.. module:: console
   :platform: Unix
   :synopsis: Tools for colourising the console.

Colour definitions and logging functions for colourising the console.

----

**Colour Definitions**

The current list of colour definitions include:

 * ``Regular``: black, red, green, yellow, blue, magenta, cyan, white,
 * ``Bold``: bold, bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white

These colour definitions can be used in the following way:

.. code-block:: python

   import rocon_console.console as console
   print(console.cyan + "    Name" + console.reset + ": " + console.yellow + "Dude" + console.reset)

----

"""

##############################################################################
# Imports
##############################################################################

import sys

##############################################################################
# Methods
##############################################################################


def console_has_colours(stream):
    """
    Detects if the specified stream has colourising capability.

    :param stream: stream to check (typically sys.stdout)
    """
    if not hasattr(stream, "isatty"):
        return False
    if not stream.isatty():
        return False  # auto color only on TTYs
    try:
        import curses
        curses.setupterm()
        return curses.tigetnum("colors") > 2
    except:
        # guess false in case of error
        return False

has_colours = console_has_colours(sys.stdout)
if has_colours:
    #reset = "\x1b[0;0m"
    reset = "\x1b[0m"
    bold = "\x1b[%sm" % '1'
    black, red, green, yellow, blue, magenta, cyan, white = ["\x1b[%sm" % str(i) for i in range(30, 38)]
    bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white = ["\x1b[%sm" % ('1;' + str(i)) for i in range(30, 38)]
else:
    reset = ""
    bold = ""
    black, red, green, yellow, blue, magenta, cyan, white = ["" for i in range(30, 38)]
    bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white = ["" for i in range(30, 38)]

colours = [
           bold,
           black, red, green, yellow, blue, magenta, cyan, white,
           bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white
          ]


def pretty_print(msg, colour=white):
    if has_colours:
        seq = colour + msg + reset
        sys.stdout.write(seq)
    else:
        sys.stdout.write(msg)


def pretty_println(msg, colour=white):
    if has_colours:
        seq = colour + msg + reset
        sys.stdout.write(seq)
        sys.stdout.write("\n")
    else:
        sys.stdout.write(msg)


##############################################################################
# Console
##############################################################################


def debug(msg):
    print(green + msg + reset)


def warning(msg):
    print(yellow + msg + reset)


def info(msg):
    print(msg)


def error(msg):
    print(red + msg + reset)


def logdebug(message):
    '''
    Prefixes '[debug]' and colours the message green.

    :param message str: message to log.
    '''
    print(green + "[debug] " + message + reset)


def loginfo(message):
    '''
    Prefixes '[info]' to the message.

    :param message str: message to log.
    '''
    print("[info ] " + message)


def logwarn(message):
    '''
    Prefixes '[warn ]' and colours the message yellow.

    :param message str: message to log.
    '''
    print(yellow + "[warn ] " + message + reset)


def logerror(message):
    '''
    Prefixes '[error]' and colours the message red.

    :param message str: message to log.
    '''
    print(red + "[error] " + message + reset)


def logfatal(message):
    '''
    Prefixes '[fatal]' and colours the message bold red.

    :param message str: message to log.
    '''
    print(bold_red + "[error] " + message + reset)


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    for colour in colours:
        pretty_print("dude\n", colour)
    logdebug("loginfo message")
    logwarn("logwarn message")
    logerror("logerror message")
    logfatal("logfatal message")
    pretty_print("red\n", red)
    print("some normal text")
    print(cyan + "    Name" + reset + ": " + yellow + "Dude" + reset)
