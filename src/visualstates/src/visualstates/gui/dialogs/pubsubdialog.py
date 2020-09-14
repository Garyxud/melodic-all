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
from PyQt5.QtWidgets import QDialog, QFormLayout, QLabel, QLineEdit, QComboBox, \
    QPushButton, QApplication, QHBoxLayout, QMessageBox
from PyQt5.QtCore import pyqtSignal
from visualstates.configs.rosconfig import RosConfig
from visualstates.configs.rospackage import getAllTypes


class PubSubDialog(QDialog):

    topicAdded = pyqtSignal()
    topicUpdated = pyqtSignal(int)

    def __init__(self, topic=None, config=None, isPublisher=True):
        super(QDialog, self).__init__()
        self.isPublisher = isPublisher
        # set dialog title
        titleText = ''
        if topic is None:
            titleText += 'Add '
        else:
            titleText += 'Edit '
        if self.isPublisher:
            titleText += 'Publisher'
        else:
            titleText += 'Subscriber'
        self.setWindowTitle(titleText)

        self.topic = topic
        self.config = config

        layout = QFormLayout()
        self.setLayout(layout)

        # create gui elements
        methodNameLbl = None
        if self.isPublisher:
            methodNameLbl = QLabel('Method Name')
        else:
            methodNameLbl = QLabel('Variable Name')
        topicLbl = QLabel('Topic')
        typeLbl = QLabel('Type')

        self.methodNameEdit = QLineEdit()
        self.methodNameEdit.setMinimumWidth(200)
        self.topicEdit = QLineEdit()
        self.topicEdit.setMinimumWidth(200)
        self.typeCb = QComboBox()
        self.typeCb.setEditable(True)
        self.typeCb.setMinimumWidth(200)

        layout.addRow(methodNameLbl, self.methodNameEdit)
        layout.addRow(topicLbl, self.topicEdit)
        layout.addRow(typeLbl, self.typeCb)

        buttonLayout = QHBoxLayout()
        cancelBtn = QPushButton('Cancel')
        cancelBtn.clicked.connect(self.cancelClicked)
        buttonLayout.addWidget(cancelBtn)
        saveBtn = QPushButton('Add')
        saveBtn.clicked.connect(self.saveClicked)
        buttonLayout.addWidget(saveBtn)
        layout.addRow(None, buttonLayout)

        for type in getAllTypes():
            self.typeCb.addItem(type, type)

        # if topic is not None activate the edit mode
        if self.topic is not None:
            saveBtn.setText('Update')
            if self.isPublisher:
                self.methodNameEdit.setText(self.topic['methodname'])
            else:
                self.methodNameEdit.setText(self.topic['variablename'])
            self.topicEdit.setText(self.topic['name'])
            # find and set the combobox to the index
            selectedTypeIndex = -1
            for i in range(self.typeCb.count()):
                if self.typeCb.itemText(i) == topic['type']:
                    selectedTypeIndex = i
                    break
            if selectedTypeIndex == -1:
                self.typeCb.addItem(topic['type'], topic['type'])
                self.typeCb.setCurrentIndex(self.typeCb.count()-1)
            else:
                self.typeCb.setCurrentIndex(selectedTypeIndex)

    def cancelClicked(self):
        self.close()

    def saveClicked(self):
        newTopic = False
        if self.topic is None:
            newTopic = True

        methodname = self.methodNameEdit.text()
        name = self.topicEdit.text()
        type = self.typeCb.currentText()
        if methodname == "" or name == "" or type == "" :
            QMessageBox.warning(self, "Fields empty",
                                    "One or more fields are empty and are required")
            return

        for topic in self.config.topics:
            if not newTopic:
                if self.topic['id'] == topic['id']:
                    continue
            if topic['opType'] == RosConfig.PUBLISH:
                if self.isPublisher:
                    if methodname == topic['methodname']:
                        QMessageBox.information(self, "Method name present",
                                                "Method name is already present in the publishers list")
                        return
                else:
                    if methodname == topic['methodname']:
                        QMessageBox.information(self, "Variable name present",
                                                "Variable name is already present in the publishers list as a method name")
                        return
            else:
                if self.isPublisher:
                    if methodname == topic['variablename']:
                        QMessageBox.information(self, "Method name present",
                                                "Method name is already present in the subscribers list as a variable name")
                        return
                else:
                    if methodname == topic['variablename']:
                        QMessageBox.information(self, "Variable name present",
                                                "Variable name is already present in the subscribers list")
                        return

        if newTopic:
            self.topic = {}

        if self.isPublisher:
            self.topic['methodname'] = methodname
            self.topic['opType'] = RosConfig.PUBLISH
        else:
            self.topic['variablename'] = methodname
            self.topic['opType'] = RosConfig.SUBSCRIBE
        self.topic['name'] = name
        self.topic['type'] = type

        if newTopic:
            self.topic['id'] = self.config.getTopicID()
            self.config.topics.append(self.topic)
            self.topicAdded.emit()
        else:
            self.topicUpdated.emit(self.topic['id'])

        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = PubSubDialog()

    dialog.exec_()