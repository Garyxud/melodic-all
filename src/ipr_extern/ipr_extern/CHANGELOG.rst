^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ipr_extern
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.8 (2018-12-12)
------------------

0.8.7 (2018-12-12)
------------------

0.8.6 (2018-12-12)
------------------

0.8.5 (2018-12-12)
------------------

0.8.4 (2018-12-12)
------------------

0.8.3 (2018-12-12)
------------------

0.8.2 (2018-12-12)
------------------

0.8.1 (2018-12-11)
------------------
* Ati rs485 interface (#1)
  * Integrated the libmodbus package (https://libmodbus.org/).
  The package was build on the official v3.1.4. release of libmodbus and was slightly modified.
  The modifications enable us to use custom baudrates not supported in the termios.h by using 'baudrate aliasing'.
  This will redefine an existing baudrate to the baudrate requested by the user and then it for modbus communication.
  * Renamed repository and added travis build
* Contributors: Denis Štogl
