.. _creating-section-label:

Colouring the Console
=====================

The :mod:`rocon_console.console` modules provides definitions and methods that enable simple
colouring for your output on the console (without having to remember all the
specific keycodes that shells use).

It will also automatically try and detect if your console has colour support and if there
is none, the colour definitions will cause the :mod:`rocon_console.console` methods to gracefully fallback
to a non-coloured syntax for printing. 

Colour Definitions
------------------

There are definitions for many of the basic console colour codes
(send a pull request if you want support for anything more). The current list includes:

 * ``Regular``: black, red, green, yellow, blue, magenta, cyan, white,
 * ``Bold``: bold, bold_black, bold_red, bold_green, bold_yellow, bold_blue, bold_magenta, bold_cyan, bold_white

Usage
-----

**Importing**

.. code-block:: python

   import rocon_console.console as console

**Freeform Style**

Simply intersperse colour definitions throughout your printing statements, e.g.

.. code-block:: python

   import rocon_console.console as console
   print(console.cyan + "    Name" + console.reset + ": " + console.yellow + "Dude" + console.reset)
  

**Logging Style**

For standard style logging modes, there are a few functions that attach a descriptive prefix and colourise
the message according to the logging mode. Prefixes:

- ``logdebug`` : [debug], green
- ``loginfo``  : [info], white
- ``logwarn``  : [warn], yellow
- ``logerror`` : [error], red
- ``logfatal`` : [fatal], bold_red

.. code-block:: python

   import rocon_console.console as console
   logdebug("the ingredients of beer are interesting, but not important to the consumer")
   loginfo("the name of a beer is useful information")
   logwarn("this is a lite beer")
   logerror("this is a budweiser")
   logfatal("this is mereley a cider")
