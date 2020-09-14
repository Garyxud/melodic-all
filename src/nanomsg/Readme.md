## Overview

URL: http://nanomsg.org/

## Catkin Package

This uses a cmake build whereas nanomsg makes use of autotools. Consequently I'm just plugging in macro definitions for many
options that are tested by autotools, and only creating tests as I run into problems (e.g. test for accept4 support in the
toolchain).

One variable that I haven't got an automatic test for yet is for our limited toolchain on arm board.

* platform: arm1176jzf-s
* kernel 2.6.32
* toolchain: gcc 4.6

If you get a `libnanomsg.so: undefined reference to `__sync_fetch_and_add_4'` error when linking this library to
your programs (happens on some arm cross compiles), then you'll need to disable gcc's builtin atomic plugins. 
