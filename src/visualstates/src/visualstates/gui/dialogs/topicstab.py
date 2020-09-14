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
from PyQt5.QtWidgets import QWidget, QPushButton, \
    QVBoxLayout, QHBoxLayout, QLabel,  \
    QGroupBox, QScrollArea, QBoxLayout
from PyQt5.QtCore import Qt
from visualstates.configs.rosconfig import RosConfig
from visualstates.gui.dialogs.pubsubdialog import PubSubDialog


class TopicsTab(QWidget):
    def __init__(self, config):
        super(QWidget, self).__init__()
        self.config = config
        self.count = 0
        self.topicRows = {}
        self.topicUIs = {}

        layout = QVBoxLayout()
        self.setLayout(layout)

        publishersBox = QGroupBox('Publishers')
        pubVLayout = QVBoxLayout()
        publishersBox.setLayout(pubVLayout)
        layout.addWidget(publishersBox)

        addPublisherBtn = QPushButton('Add Publisher')
        addPublisherBtn.setMaximumWidth(200)
        addPublisherBtn.clicked.connect(self.addPublisher)
        pubVLayout.addWidget(addPublisherBtn)

        pubRowLayout = QHBoxLayout()
        pubVLayout.addLayout(pubRowLayout)

        # publisher headers
        pubRowLayout.addSpacing(10)
        titleLblStyleSheet = 'QLabel { font-weight: bold;}'
        methodLbl = QLabel('Method Name')
        methodLbl.setStyleSheet(titleLblStyleSheet)
        methodLbl.setMinimumWidth(200)
        pubRowLayout.addWidget(methodLbl)
        topicLbl = QLabel('Topic')
        topicLbl.setStyleSheet(titleLblStyleSheet)
        topicLbl.setMinimumWidth(200)
        pubRowLayout.addWidget(topicLbl)
        typeLbl = QLabel('Type')
        typeLbl.setStyleSheet(titleLblStyleSheet)
        typeLbl.setMinimumWidth(200)
        pubRowLayout.addWidget(typeLbl)
        actionLbl = QLabel('Actions')
        actionLbl.setStyleSheet(titleLblStyleSheet)
        actionLbl.setMaximumWidth(200)
        actionLbl.setMinimumWidth(200)
        pubRowLayout.addWidget(actionLbl)
        pubRowLayout.addSpacing(10)

        pubScrollArea = QScrollArea()
        pubScrollArea.setWidgetResizable(True)
        # pubScrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        pubScrollArea.setStyleSheet('QScrollArea {border: 0px;}')
        self.pubScrollVlayout = QVBoxLayout()
        self.pubScrollVlayout.setDirection(QBoxLayout.TopToBottom)
        self.pubScrollVlayout.setAlignment(Qt.AlignTop)
        dummyBox = QGroupBox()
        dummyBox.setStyleSheet('QGroupBox {padding: 0px; margin: 0px;}')
        dummyBox.setLayout(self.pubScrollVlayout)
        pubScrollArea.setWidget(dummyBox)
        pubVLayout.addWidget(pubScrollArea)

        subscribersBox = QGroupBox('Subscribers')
        subsVLayout = QVBoxLayout()
        subscribersBox.setLayout(subsVLayout)
        layout.addWidget(subscribersBox)

        addSubscriberBtn = QPushButton('Add Subscriber')
        addSubscriberBtn.setMaximumWidth(200)
        addSubscriberBtn.clicked.connect(self.addSubscriber)
        subsVLayout.addWidget(addSubscriberBtn)

        subsRowLayout = QHBoxLayout()
        subsVLayout.addLayout(subsRowLayout)

        # publisher headers
        subsRowLayout.addSpacing(10)
        titleLblStyleSheet = 'QLabel {font-weight: bold;}'
        varLbl = QLabel('Variable Name')
        varLbl.setStyleSheet(titleLblStyleSheet)
        varLbl.setMinimumWidth(200)
        subsRowLayout.addWidget(varLbl)
        topicLbl = QLabel('Topic')
        topicLbl.setStyleSheet(titleLblStyleSheet)
        topicLbl.setMinimumWidth(200)
        subsRowLayout.addWidget(topicLbl)
        typeLbl = QLabel('Type')
        typeLbl.setStyleSheet(titleLblStyleSheet)
        typeLbl.setMinimumWidth(200)
        subsRowLayout.addWidget(typeLbl)
        actionLbl = QLabel('Actions')
        actionLbl.setStyleSheet(titleLblStyleSheet)
        actionLbl.setMaximumWidth(200)
        actionLbl.setMinimumWidth(200)
        subsRowLayout.addWidget(actionLbl)
        subsRowLayout.addSpacing(10)

        subsScrollArea = QScrollArea()
        subsScrollArea.setWidgetResizable(True)
        # subsScrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        subsScrollArea.setStyleSheet('QScrollArea {border: 0px;}')
        self.subsScrollVlayout = QVBoxLayout()
        self.subsScrollVlayout.setDirection(QBoxLayout.TopToBottom)
        self.subsScrollVlayout.setAlignment(Qt.AlignTop)
        dummyBox = QGroupBox()
        dummyBox.setStyleSheet('QGroupBox {padding: 0px; margin: 0px;}')
        dummyBox.setLayout(self.subsScrollVlayout)
        subsScrollArea.setWidget(dummyBox)
        subsVLayout.addWidget(subsScrollArea)

        # draw by the current config
        for topic in self.config.topics:
            self.addTopic(topic)

    def addPublisher(self):
        dialog = PubSubDialog(config=self.config, isPublisher=True)
        dialog.topicAdded.connect(self.topicAddedHandler)
        dialog.exec_()

    def addSubscriber(self):
        dialog = PubSubDialog(config=self.config, isPublisher=False)
        dialog.topicAdded.connect(self.topicAddedHandler)
        dialog.exec_()

    def addTopic(self, topic):
        rowLayout = QHBoxLayout()
        topicStyle = 'QLabel {border: 1px solid black;}'
        nameLbl = QLabel()
        nameLbl.setStyleSheet(topicStyle)
        nameLbl.setMinimumWidth(200)
        rowLayout.addWidget(nameLbl)
        topicLbl = QLabel(topic['name'])
        topicLbl.setStyleSheet(topicStyle)
        topicLbl.setMinimumWidth(200)
        rowLayout.addWidget(topicLbl)
        typeLbl = QLabel(topic['type'])
        typeLbl.setStyleSheet(topicStyle)
        typeLbl.setMinimumWidth(200)
        rowLayout.addWidget(typeLbl)

        editBtn = QPushButton('Edit')
        editBtn.setMinimumWidth(96)
        editBtn.setMaximumWidth(96)
        editBtn.setObjectName(str(topic['id']))
        editBtn.clicked.connect(self.editHandler)
        rowLayout.addWidget(editBtn)
        removeBtn = QPushButton('Remove')
        removeBtn.setMinimumWidth(96)
        removeBtn.setMaximumWidth(96)
        removeBtn.setObjectName(str(topic['id']))
        removeBtn.clicked.connect(self.removeHandler)
        rowLayout.addWidget(removeBtn)
        UIs = [nameLbl, topicLbl, typeLbl, editBtn, removeBtn, rowLayout]
        self.topicUIs[topic['id']] = UIs
        if topic['opType'] == RosConfig.PUBLISH:
            nameLbl.setText(topic['methodname'])
            self.pubScrollVlayout.addLayout(rowLayout)
        elif topic['opType'] == RosConfig.SUBSCRIBE:
            nameLbl.setText(topic['variablename'])
            self.subsScrollVlayout.addLayout(rowLayout)

    def topicAddedHandler(self):
        addedTopic = self.config.topics[len(self.config.topics)-1]
        self.addTopic(addedTopic)

    def editHandler(self):
        editID = int(self.sender().objectName())
        editTopic = None
        dialog = None
        for topic in self.config.topics:
            if topic['id'] == editID:
                editTopic = topic
                break
        if editTopic is not None:
            if editTopic['opType'] == RosConfig.PUBLISH:
                dialog = PubSubDialog(topic=editTopic, config=self.config, isPublisher=True)
            elif editTopic['opType'] == RosConfig.SUBSCRIBE:
                dialog = PubSubDialog(topic=editTopic, config=self.config, isPublisher=False)
            dialog.topicUpdated.connect(self.topicUpdatedHandler)
            dialog.exec_()

    def topicUpdatedHandler(self, updatedID):
        updatedTopic = None
        for topic in self.config.topics:
            if topic['id'] == updatedID:
                updatedTopic = topic
                break

        if updatedTopic is not None:
            UIs = self.topicUIs[updatedID]
            if updatedTopic['opType'] == RosConfig.PUBLISH:
                UIs[0].setText(updatedTopic['methodname'])
            elif updatedTopic['opType'] == RosConfig.SUBSCRIBE:
                UIs[0].setText(updatedTopic['variablename'])

            UIs[1].setText(updatedTopic['name'])
            UIs[2].setText(updatedTopic['type'])

    def removeHandler(self):
        removeID = int(self.sender().objectName())
        removeItem = None
        for topic in self.config.topics:
            if topic['id'] == removeID:
                removeItem = topic
                break
        if removeItem is not None:
            self.config.topics.remove(removeItem)
            UIs = self.topicUIs[removeID]
            for uiItem in UIs:
                uiItem.deleteLater()