Overview:
=========

This is the README file of SDHLibrary-CPP, the C++ library to access
an SDH (SCHUNK Dexterous Hand).


Directory structure:
====================

This directory contains the overall Makefile and sub directories:
- ./bin/  : location of the compiled library and demo programs
- ./demo/ : C++ source code of some demonstration programs that use the library
- ./doc/  : pregenerated documentation (pdf and html)
- ./sdh/  : C++ source and header files for the library itself
- ./vcc/  : project files for Visual Studio C++

Usage instructions:
===================

The following instructions below are for Cygwin and Linux environments.
If you have to use Visual Studio C++, please read ./vcc/README.txt.

Build:
------
To build the library and demo-programs run 
  > make build

in this directory. Then you can call the demonstration programs in
./bin/ directly. If you generated a shared library then you have to
add the generated files to your pathsfirst. Use the sdhsetenv script in the
parent-directory of this directory to set the paths. That script must
be sourced-in like in:
  > cd .. ; source sdhsetenv

To ease the usage of the library in your own programs you should install it:


Install:
--------
To install the library and demo-programs run 
  > make install
in this direcory. Adjust INSTALL_PREFIX in ./Makefile to install to another
directory than /usr/local. See there for additional settings.

- The static or shared library is installed in $INSTALL_PREFIX/lib/.
- The headers are installed in $INSTALL_PREFIX/include/sdh/.
- The demo programs are installed in $INSTALL_PREFIX/bin/
- The documentation is installed in /usr/share/doc/SDHLibrary-CPP

Uninstall:
----------
To uninstall the library, demo-programs and documentation run
  > make uninstall
in this directory.


Further info:
=============
See the demo programs in ./demo/ and the documentation in ./doc/ on how to
use the offered functionality.


Contact information:
====================
Dirk Osswald: mailto:dirk.osswald@de.schunk.com
              http://www.schunk.com
