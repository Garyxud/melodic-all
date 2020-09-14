^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libmodbus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.8 (2018-12-12)
------------------
* reintroduiced not so redundant include file
* Contributors: Gilbert Groten

0.8.7 (2018-12-12)
------------------
* removed previously generated .la files
* Contributors: Gilbert Groten

0.8.6 (2018-12-12)
------------------
* removed redundant include file
* Contributors: Gilbert Groten

0.8.5 (2018-12-12)
------------------
* removed redundant source file
* Contributors: Gilbert Groten

0.8.4 (2018-12-12)
------------------
* removed redundant makefile
* Contributors: Gilbert Groten

0.8.3 (2018-12-12)
------------------
* removed redundant makefile
* Contributors: Gilbert Groten

0.8.2 (2018-12-12)
------------------
* removed redundant makefile
* Contributors: Gilbert Groten

0.8.1 (2018-12-11)
------------------
* Cleaned libmodbus with correct includes (#2)
* Update CMakeLists.txt
* Update CMakeLists.txt
* Ati rs485 interface (#1)
  * Integrated the libmodbus package (https://libmodbus.org/).
  The package was build on the official v3.1.4. release of libmodbus and was slightly modified.
  The modifications enable us to use custom baudrates not supported in the termios.h by using 'baudrate aliasing'.
  This will redefine an existing baudrate to the baudrate requested by the user and then it for modbus communication.
  * Renamed repository and added travis build
* Contributors: Denis Štogl
