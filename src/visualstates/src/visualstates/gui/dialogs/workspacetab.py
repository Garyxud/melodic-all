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
import sys, os
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QHBoxLayout, QPushButton, \
    QApplication, QLabel, QScrollArea, QBoxLayout, QFileDialog, QMessageBox
from PyQt5.QtCore import Qt
from visualstates.configs.rospackage import readWorkspaces
from visualstates.configs.rospackage import writeWorkspaces


class WorkspaceTab(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        layout = QVBoxLayout()
        self.setLayout(layout)

        scrollArea = QScrollArea()
        scrollArea.setMinimumHeight(200)
        scrollArea.setWidgetResizable(True)
        scrollArea.setStyleSheet('QScrollArea {border: 0;}')
        layout.addWidget(scrollArea)

        workspaceBox = QGroupBox('Catkin Workspaces')
        scrollArea.setWidget(workspaceBox)
        self.workspaceLayout = QVBoxLayout()
        self.workspaceLayout.setDirection(QBoxLayout.TopToBottom)
        self.workspaceLayout.setAlignment(Qt.AlignTop)
        workspaceBox.setLayout(self.workspaceLayout)

        buildRowLayout = QHBoxLayout()
        self.workspaceLayout.addLayout(buildRowLayout)
        chooseDirButton = QPushButton('Choose File')
        chooseDirButton.setMaximumWidth(100)
        chooseDirButton.setObjectName('choose')
        chooseDirButton.clicked.connect(self.chooseDir)
        buildRowLayout.addWidget(chooseDirButton)
        self.dirLabel = QLabel()
        self.dirLabel.setMinimumWidth(400)
        buildRowLayout.addWidget(self.dirLabel)

        addButton = QPushButton('Add')
        addButton.setMaximumWidth(100)
        addButton.setObjectName('build')
        addButton.clicked.connect(self.addWorkspace)
        buildRowLayout.addWidget(addButton)

        # load current workspaces
        self.workspaceUIs = []
        self.drawWorkspaces(readWorkspaces())

    def chooseDir(self):
        selectedDir = QFileDialog.getExistingDirectory(self, 'Choose Catkin Workspace Directory')
        self.dirLabel.setText(selectedDir)

    def addWorkspace(self):
        newDir = self.dirLabel.text()
        if newDir == '':
            QMessageBox.warning(self, 'Invalid Workspace', 'This is not a catkin workspace or specified path was not found')
        elif self.isValidWorkspace(newDir):
            workspaces = readWorkspaces()
            if newDir in workspaces:
                QMessageBox.information(self, 'Workspace present', 'The workspace entered is already present in the list')
                return
            workspaces.append(newDir)
            writeWorkspaces(workspaces)
            self.drawWorkspaces(workspaces)
        else:
            QMessageBox.warning(self, 'Invalid Workspace', 'The catkin workspace dir:' + newDir +
                                ' is not a valid. Please make sure that the directory has src, devel and build directories.')

    def isValidWorkspace(self, dir):
        files = os.listdir(dir)
        return '.catkin_workspace' in files

    def addWorkspaceUI(self, layout, dir, workspaceUIs, removeCallback):
        uiData = {}
        rowLayout = QHBoxLayout()
        uiData['layout'] = rowLayout
        layout.addLayout(rowLayout)
        label = QLabel(dir)
        label.setStyleSheet('QLabel { border: 1px solid black; }')
        uiData['label'] = label
        removeBtn = QPushButton('Remove')
        removeBtn.setMaximumWidth(80)
        removeBtn.clicked.connect(removeCallback)
        removeBtn.setObjectName(dir)
        uiData['btn'] = removeBtn
        rowLayout.addWidget(label)
        rowLayout.addWidget(removeBtn)
        workspaceUIs.append(uiData)

    def removeWorkspace(self):
        dir = self.sender().objectName()
        workspaces = readWorkspaces()
        workspaces.remove(dir)
        writeWorkspaces(workspaces)
        self.drawWorkspaces(workspaces)

    def drawWorkspaces(self, workspaces):
        # remove previous UIs
        for uiData in self.workspaceUIs:
            uiData['label'].deleteLater()
            uiData['btn'].deleteLater()
            uiData['layout'].deleteLater()
        self.workspaceUIs = []

        for dir in workspaces:
            self.addWorkspaceUI(self.workspaceLayout, dir, self.workspaceUIs, self.removeWorkspace)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    widget = WorkspaceTab()
    widget.show()
    app.exec_()

