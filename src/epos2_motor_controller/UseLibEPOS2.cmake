#                                               -*- cmake -*-
#
#  UseLibEPOS2.cmake
#
#  Copyright (C) 2013 Jochen Sprickerhof <jochen@sprickerhof.de>
#
#  This file is part of LibEPOS2.
#
#  LibEPOS2 is free software; you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License
#  version 2.1 as published by the Free Software Foundation;

FIND_PACKAGE(LibFTDI1 NO_MODULE REQUIRED)
INCLUDE(${LIBFTDI_USE_FILE})

add_definitions     ( ${LIBEPOS2_DEFINITIONS} )
include_directories ( ${LIBEPOS2_INCLUDE_DIRS} )
link_directories    ( ${LIBEPOS2_LIBRARY_DIRS} )
