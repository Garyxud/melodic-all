Terminals
=========

Terminal Detection
------------------

If a particular terminal is not flagged on the command line (e.g. ``--gnome`` or ``--konsole``), then
it will do a lookup to see what your default is via the command::

    > update-alternatives --query x-terminal-emulator


Supported Terminals
-------------------

Currently we have support for:

* gnome-terminal
* konsole

This is fairly easily extended by adding the appropriate functionality in :func:`.choose_terminal` function.

 
