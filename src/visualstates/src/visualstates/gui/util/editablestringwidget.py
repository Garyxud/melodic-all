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
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLineEdit, QPushButton, QLabel, \
    QApplication
from PyQt5.QtCore import pyqtSignal


class EditableStringWidget(QWidget):

    removed = pyqtSignal(str)
    updated = pyqtSignal(str, str)

    def __init__(self, text):
        super(QWidget, self).__init__()
        self.layout = QHBoxLayout()
        self.setLayout(self.layout)
        # by default show the text in label
        self.label = QLabel(text)
        self.label.setMinimumWidth(200)
        self.layout.addWidget(self.label)

        self.lineEdit = QLineEdit(text)
        self.lineEdit.setMinimumWidth(200)
        self.layout.addWidget(self.lineEdit)
        self.lineEdit.setVisible(False)

        self.editButton = QPushButton("Edit")
        self.editButton.setMaximumWidth(80)
        self.editButton.setMinimumWidth(80)
        self.editButton.clicked.connect(self.editClicked)
        self.layout.addWidget(self.editButton)
        self.removeButton = QPushButton("Remove")
        self.removeButton.setMaximumWidth(80)
        self.removeButton.setMinimumWidth(80)
        self.removeButton.clicked.connect(self.removeClicked)
        self.layout.addWidget(self.removeButton)

        # save button
        self.saveButton = QPushButton("Save")
        self.saveButton.setMinimumWidth(80)
        self.saveButton.setMaximumWidth(80)
        self.saveButton.clicked.connect(self.saveClicked)
        self.layout.addWidget(self.saveButton)
        self.saveButton.setVisible(False)

        # cancel button
        self.cancelButton = QPushButton("Cancel")
        self.cancelButton.setMinimumWidth(80)
        self.cancelButton.setMaximumWidth(80)
        self.cancelButton.clicked.connect(self.cancelClicked)
        self.layout.addWidget(self.cancelButton)
        self.cancelButton.setVisible(False)

        self.label.setStyleSheet('QLabel { border: 1px solid black; }')
        self.setContentsMargins(0, 0, 0, 0)
        self.layout.setContentsMargins(0, 0, 0, 0)

    def editClicked(self):
        self.removeButton.setVisible(False)
        self.editButton.setVisible(False)
        self.label.setVisible(False)

        self.lineEdit.setVisible(True)
        self.saveButton.setVisible(True)
        self.cancelButton.setVisible(True)

    def removeClicked(self):
        self.removed.emit(self.label.text())

    def saveClicked(self):
        self.updated.emit(self.label.text(), self.lineEdit.text())
        self.label.setText(self.lineEdit.text())
        self.label.setVisible(True)
        self.editButton.setVisible(True)
        self.removeButton.setVisible(True)

        self.lineEdit.setVisible(False)
        self.cancelButton.setVisible(False)
        self.saveButton.setVisible(False)

    def cancelClicked(self):
        self.cancelButton.setVisible(False)
        self.saveButton.setVisible(False)
        self.lineEdit.setVisible(False)

        self.label.setVisible(True)
        self.removeButton.setVisible(True)
        self.editButton.setVisible(True)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    widget = EditableStringWidget('okan')
    widget.show()
    app.exec_()