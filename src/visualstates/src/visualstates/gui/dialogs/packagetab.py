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
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QComboBox, QHBoxLayout, QPushButton, \
    QApplication, QLabel, QScrollArea, QBoxLayout, QMessageBox
from PyQt5.QtCore import Qt
from visualstates.configs.rospackage import getAllPackages


class PackageTab(QWidget):
    def __init__(self, config):
        super(QWidget, self).__init__()

        # the list of packages for build and run dependencies
        self.buildDependencies = config.getBuildDependencies()
        self.buildDependenciesUI = []
        self.runDependencies = config.getRunDependencies()
        self.runDependenciesUI = []

        layout = QVBoxLayout()
        self.setLayout(layout)

        buildDependenciesScroll = QScrollArea()
        buildDependenciesScroll.setMinimumHeight(200)
        buildDependenciesScroll.setWidgetResizable(True)
        buildDependenciesScroll.setStyleSheet('QScrollArea {border: 0;}')
        layout.addWidget(buildDependenciesScroll)

        buildDependenciesBox = QGroupBox('Build Dependencies')
        buildDependenciesScroll.setWidget(buildDependenciesBox)
        self.buildLayout = QVBoxLayout()
        self.buildLayout.setDirection(QBoxLayout.TopToBottom)
        self.buildLayout.setAlignment(Qt.AlignTop)
        buildDependenciesBox.setLayout(self.buildLayout)

        buildRowLayout = QHBoxLayout()
        self.buildLayout.addLayout(buildRowLayout)
        self.buildDependenciesCb = QComboBox()
        self.buildDependenciesCb.setEditable(True)
        buildRowLayout.addWidget(self.buildDependenciesCb)
        buildAddButton = QPushButton('Add')
        buildAddButton.setMaximumWidth(80)
        buildAddButton.setObjectName('build')
        buildAddButton.clicked.connect(self.addDependency)
        buildRowLayout.addWidget(buildAddButton)

        runDependenciesScroll = QScrollArea()
        runDependenciesScroll.setMinimumHeight(200)
        runDependenciesScroll.setWidgetResizable(True)
        runDependenciesScroll.setStyleSheet('QScrollArea { border: 0;}')
        layout.addWidget(runDependenciesScroll)

        runDependenciesBox = QGroupBox('Run Dependencies')
        runDependenciesScroll.setWidget(runDependenciesBox)
        self.runLayout = QVBoxLayout()
        self.runLayout.setDirection(QBoxLayout.TopToBottom)
        self.runLayout.setAlignment(Qt.AlignTop)
        runDependenciesBox.setLayout(self.runLayout)

        runRowLayout = QHBoxLayout()
        self.runLayout.addLayout(runRowLayout)
        self.runDependenciesCb = QComboBox()
        self.runDependenciesCb.setEditable(True)
        runRowLayout.addWidget(self.runDependenciesCb)
        runAddButton = QPushButton('Add')
        runAddButton.setMaximumWidth(80)
        runAddButton.setObjectName('run')
        runAddButton.clicked.connect(self.addDependency)
        runRowLayout.addWidget(runAddButton)

        # fill the packages
        packages = getAllPackages()
        packages = sorted(packages)
        for pkg in packages:
            self.runDependenciesCb.addItem(pkg)
            self.buildDependenciesCb.addItem(pkg)

        self.drawDependenciesUI()

    def addDependency(self):
        layout = None
        dependency = None
        removeCallback = None
        dependenciesUI = None
        if self.sender().objectName() == 'build':
            layout = self.buildLayout
            dependency = self.buildDependenciesCb.currentText()
            if len(dependency) > 0:
                if dependency in self.buildDependencies:
                    QMessageBox.information(self, "Dependency present",
                                            "This dependency is already present in the Build Dependencies list")
                    return
                removeCallback = self.removeBuildDependency
                dependenciesUI = self.buildDependenciesUI
                self.buildDependencies.append(dependency)
        elif self.sender().objectName() == 'run':
            layout = self.runLayout
            dependency = self.runDependenciesCb.currentText()
            if len(dependency) > 0:
                if dependency in self.runDependencies:
                    QMessageBox.information(self, "Dependency present",
                                            "This dependency is already present in the Run Dependencies list")
                    return
                removeCallback = self.removeRunDependency
                dependenciesUI = self.runDependenciesUI
                self.runDependencies.append(dependency)

        if dependenciesUI is not None:
            self.addDependencyUIs(layout, dependency, dependenciesUI, removeCallback)

    def addDependencyUIs(self, layout, dependency, dependenciesUI, removeCallback):
        dep = {}
        rowLayout = QHBoxLayout()
        dep['layout'] = rowLayout
        layout.addLayout(rowLayout)
        label = QLabel(dependency)
        label.setStyleSheet('QLabel { border: 1px solid black; }')
        dep['label'] = label
        removeBtn = QPushButton('Remove')
        removeBtn.setMaximumWidth(80)
        removeBtn.clicked.connect(removeCallback)
        removeBtn.setObjectName(dependency)
        dep['btn'] = removeBtn
        rowLayout.addWidget(label)
        rowLayout.addWidget(removeBtn)
        dependenciesUI.append(dep)

    def removeDep(self, dep, dependenciesUI):
        removeItem = None
        for runDep in dependenciesUI:
            if runDep['label'].text() == dep:
                removeItem = runDep
                break

        if removeItem is not None:
            dependenciesUI.remove(removeItem)
            removeItem['label'].deleteLater()
            removeItem['btn'].deleteLater()
            removeItem['layout'].deleteLater()

    def removeRunDependency(self):
        dep = self.sender().objectName()
        self.runDependencies.remove(dep)
        self.removeDep(dep, self.runDependenciesUI)

    def removeBuildDependency(self):
        dep = self.sender().objectName()
        self.buildDependencies.remove(dep)
        self.removeDep(dep, self.buildDependenciesUI)

    def drawDependenciesUI(self):
        # remove previous UIs if exists
        for depUI in self.runDependenciesUI:
            depUI['label'].deleteLater()
            depUI['btn'].deleteLater()
            depUI['layout'].deleteLater()

        for depUI in self.buildDependenciesUI:
            depUI['label'].deleteLater()
            depUI['btn'].deleteLater()
            depUI['layout'].deleteLater()

        # add dependency for given
        for dep in self.runDependencies:
            self.addDependencyUIs(self.runLayout, dep, self.runDependenciesUI, self.removeRunDependency)

        for dep in self.buildDependencies:
            self.addDependencyUIs(self.buildLayout, dep, self.buildDependenciesUI, self.removeBuildDependency)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    widget = PackageTab()
    widget.show()
    app.exec_()

