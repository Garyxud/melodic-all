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
from PyQt5.QtWidgets import QDialog, QVBoxLayout, \
    QApplication, QTabWidget
from visualstates.configs.rosconfig import RosConfig
from visualstates.gui.dialogs.packagetab import PackageTab
from visualstates.gui.dialogs.topicstab import TopicsTab
from visualstates.gui.dialogs.workspacetab import WorkspaceTab


class RosConfigDialog(QDialog):

    def __init__(self, name, config):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.tabWidget = None

        self.config = config

        self.tabWidget = QTabWidget()
        mainLayout = QVBoxLayout()
        mainLayout.addWidget(self.tabWidget)
        self.packageTab = PackageTab(self.config)
        self.tabWidget.addTab(self.packageTab, 'Package')
        self.topicsTab = TopicsTab(self.config)
        self.tabWidget.addTab(self.topicsTab, 'Topics')
        self.workspaceTab = WorkspaceTab()
        self.tabWidget.addTab(self.workspaceTab, 'Workspaces')
        self.setLayout(mainLayout)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    config = RosConfig()

    dialog = RosConfigDialog('Config')
    dialog.setConfig(config)

    dialog.exec_()
