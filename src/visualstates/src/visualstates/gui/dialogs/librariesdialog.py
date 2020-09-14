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
from PyQt5.QtWidgets import QDialog, \
    QLineEdit, QPushButton, QApplication, \
    QScrollArea, QVBoxLayout, QGroupBox, QHBoxLayout, \
    QBoxLayout, QMessageBox
from PyQt5.QtCore import pyqtSignal, Qt
from visualstates.gui.util.editablestringwidget import EditableStringWidget


class LibrariesDialog(QDialog):

    librariesChanged = pyqtSignal(list)

    def __init__(self, name, libraries):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.setMinimumHeight(500)
        self.setMinimumWidth(500)
        self.libraries = libraries
        self.libraryNameEdit = QLineEdit()
        self.addButton = QPushButton("Add")
        self.addButton.setMinimumWidth(100)

        # create gui of
        verticalLayout = QVBoxLayout()
        self.setLayout(verticalLayout)
        scrollArea = QScrollArea()
        scrollArea.setMinimumHeight(400)
        scrollArea.setWidgetResizable(True)
        scrollArea.setStyleSheet('QScrollArea {border: 0;}')
        verticalLayout.addWidget(scrollArea)
        groupBox = QGroupBox("Libraries")
        scrollArea.setWidget(groupBox)
        self.groupLayout = QVBoxLayout()
        self.groupLayout.setDirection(QBoxLayout.TopToBottom)
        self.groupLayout.setAlignment(Qt.AlignTop)
        groupBox.setLayout(self.groupLayout)

        # add row to group layout
        rowLayout = QHBoxLayout()
        self.groupLayout.addLayout(rowLayout)
        rowLayout.addWidget(self.libraryNameEdit)
        rowLayout.addWidget(self.addButton)
        self.addButton.clicked.connect(self.addClicked)

        self.libraryUIs = {}

        # add libraries to the ui
        for libraryName in self.libraries:
            self.addLibraryItem(libraryName)

    def removed(self, libraryName):
        if libraryName in self.libraryUIs:
            libRow = self.libraryUIs[libraryName]
            libRow.disconnect()
            self.groupLayout.removeWidget(libRow)
            del self.libraryUIs[libraryName]
            libRow.deleteLater()
            self.libraries.remove(libraryName)
            self.librariesChanged.emit(self.libraries)

    def updated(self, oldLibraryName, newLibraryName):
        if oldLibraryName in self.libraryUIs:
            libRow = self.libraryUIs[oldLibraryName]
            del self.libraryUIs[oldLibraryName]
            self.libraryUIs[newLibraryName] = libRow
            # update library list
            for i in range(len(self.libraries)):
                if self.libraries[i] == oldLibraryName:
                    self.libraries[i] = newLibraryName
                    break
            self.librariesChanged.emit(self.libraries)

    def addClicked(self):
        libraryInp = self.libraryNameEdit.text().strip()
        if libraryInp in self.libraries:
            QMessageBox.information(self, "Library Present", "Library already present in the list")
            return
        if libraryInp:
            self.libraries.append(self.libraryNameEdit.text())
            self.librariesChanged.emit(self.libraries)
            self.addLibraryItem(self.libraryNameEdit.text())
            self.libraryNameEdit.setText('')

    def addLibraryItem(self, libraryName):
        libRow = EditableStringWidget(libraryName)
        self.libraryUIs[libraryName] = libRow
        libRow.removed.connect(self.removed)
        libRow.updated.connect(self.updated)
        self.groupLayout.addWidget(libRow)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = LibrariesDialog('Libraries', ['okan'])
    dialog.exec_()





