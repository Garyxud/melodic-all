#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Asik (asik.okan@gmail.com)

  '''
import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon
from visualstates.gui.visualstates import VisualStates
from visualstates.configs.rospackage import getPackagePath

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon(getPackagePath()+'/resources/jderobot_logo_notext.png'))
    mainWindow = VisualStates()
    if len(sys.argv) > 1:
        mainWindow.openFile(sys.argv[1])
    sys.exit(app.exec_())