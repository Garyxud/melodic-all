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

from PyQt5.QtWidgets import QDialog, QApplication
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from visualstates.configs.rospackage import getPackagePath


class AboutDialog(QDialog):
    def __init__(self):
        QDialog.__init__(self)
        self.setWindowTitle("JdeRobot VisualStates")
        mainLayout = QVBoxLayout()
        headerLayout = QHBoxLayout()
        logo = QLabel()
        logoPixmap = QPixmap(getPackagePath() + '/resources/jderobot_big.png')
        logo.setPixmap(logoPixmap)
        logo.setStyleSheet("QLabel {background-color:white; padding:10px;}")
        logo.setFixedWidth(160)
        headerLayout.addWidget(logo)
        title = QLabel()
        title.setAlignment(Qt.AlignLeft)
        title.setAlignment(Qt.AlignVCenter)
        title.setText("JdeRobot VisualStates")
        title.setStyleSheet("QLabel {font-size:24pt; background-color:white;}")
        headerLayout.addWidget(title)
        mainLayout.addItem(headerLayout)

        explanation = QLabel()
        explanation.setWordWrap(True)
        explanation.setStyleSheet("QLabel {font-size:14pt; padding:10px; background-color:#FEFFA8;}")
        explanation.setText("JdeRobot VisualStates is an open source visual tool to develop reactive robot behaviors. "
                            "The behaviors are represented by hierarchical finite state machines. "
                            "The tool generates a ROS package that is ready to be run.")
        mainLayout.addWidget(explanation)

        githubLink = QLabel()
        githubLink.setTextFormat(Qt.RichText)
        githubLink.setTextInteractionFlags(Qt.TextBrowserInteraction)
        githubLink.setOpenExternalLinks(True)
        githubLink.setText("<a href=\"https://github.com/JdeRobot/VisualStates\">https://github.com/JdeRobot/VisualStates</a>")
        githubLink.setAlignment(Qt.AlignHCenter)
        githubLink.setStyleSheet("QLabel {color:blue; text-decoration: underline; padding:5px;}")
        githubLink.setFixedHeight(30)
        mainLayout.addWidget(githubLink)

        jderobotLink = QLabel()
        jderobotLink.setTextFormat(Qt.RichText)
        jderobotLink.setTextInteractionFlags(Qt.TextBrowserInteraction)
        jderobotLink.setOpenExternalLinks(True)
        jderobotLink.setFixedHeight(30)
        jderobotLink.setText("<a href=\"https://jderobot.org\">https://jderobot.org</a>")
        jderobotLink.setAlignment(Qt.AlignHCenter)
        jderobotLink.setStyleSheet("QLabel {color:blue; text-decoration: underline; padding:5px;}")
        mainLayout.addWidget(jderobotLink)


        self.setLayout(mainLayout)
        self.setFixedSize(600, 400)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    resource_path = '/media/okan/depo/jderobot/catkin_ws/src/visualstates'
    dialog = AboutDialog()
    dialog.exec_()
