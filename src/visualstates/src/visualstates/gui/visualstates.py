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
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor, QPixmap
from PyQt5.QtWidgets import QMainWindow, QAction, QDockWidget, QTreeView, QGraphicsView, \
    QWidget, QFileDialog, QLabel, QVBoxLayout, QPushButton, QMessageBox
from .automata.automatascene import AutomataScene, OpType
from ..parsers.filemanager import FileManager
from ..parsers.importmanager import ImportManager
from .tree.treemodel import TreeModel
from ..core.state import State
from ..core.namespace import Namespace
from .transition.timerdialog import TimerDialog
from .dialogs.namespacedialog import NamespaceDialog
from .dialogs.librariesdialog import LibrariesDialog
from .dialogs.rosconfigdialog import RosConfigDialog
from ..configs.rosconfig import RosConfig
from ..generators.cpprosgenerator import CppRosGenerator
from ..generators.pythonrosgenerator import PythonRosGenerator
from ..configs.rospackage import getPackagePath
from .dialogs.aboutdialog import AboutDialog


class VisualStates(QMainWindow):
    def __init__(self, parent=None):
        super(QMainWindow, self).__init__()

        self.setWindowTitle("VisualStates")
        self.configDialog = None

        # root state
        self.globalNamespace = Namespace('', '')
        self.localNamespace = Namespace('', '')
        self.rootState = State(0, "root", True, self.localNamespace)
        self.activeState = self.rootState
        self.activeNamespace = self.localNamespace

        self.statusBar()
        self.createMenu()
        self.createTreeView()
        self.createStateCanvas()

        self.setGeometry(0, 0, 800, 600)
        self.show()

        self.fileManager = FileManager()
        self.importManager = ImportManager()
        self.automataPath = None

        self.libraries = []
        self.config = None

    def createMenu(self):
        # create actions
        newAction = QAction('&New', self)
        newAction.setShortcut('Ctrl+N')
        newAction.setStatusTip('Create New Visual States')
        newAction.triggered.connect(self.newAction)

        openAction = QAction('&Open', self)
        openAction.setShortcut('Ctrl+O')
        openAction.setStatusTip('Open Visual States')
        openAction.triggered.connect(self.openAction)

        importAction = QAction('&Import', self)
        openAction.setShortcut('Ctrl+I')
        importAction.setStatusTip('Import A State')
        importAction.triggered.connect(self.importAction)

        saveAction = QAction('&Save', self)
        saveAction.setShortcut('Ctrl+S')
        saveAction.setStatusTip('Save Visual States')
        saveAction.triggered.connect(self.saveAction)

        saveAsAction = QAction('&Save As', self)
        saveAsAction.setShortcut('Ctrl+Shift+S')
        saveAsAction.setStatusTip('Save Visual States as New One')
        saveAsAction.triggered.connect(self.saveAsAction)

        quitAction = QAction('&Quit', self)
        quitAction.setShortcut('Ctrl+Q')
        quitAction.setStatusTip('Quit Visual States')
        quitAction.triggered.connect(self.quitAction)

        # figures menu
        stateAction = QAction('&State', self)
        stateAction.setStatusTip('Create a state')
        stateAction.triggered.connect(self.stateAction)

        transitionAction = QAction('&Transition', self)
        transitionAction.setStatusTip('Create a transition')
        transitionAction.triggered.connect(self.transitionAction)

        # data menu
        timerAction = QAction('&Timer', self)
        timerAction.setShortcut('Ctrl+M')
        timerAction.setStatusTip('Set timing of states')
        timerAction.triggered.connect(self.timerAction)

        globalNamespaceAction = QAction('&Global Namespace', self)
        globalNamespaceAction.setShortcut('Ctrl+G')
        globalNamespaceAction.setStatusTip('Open Global Namespace')
        globalNamespaceAction.triggered.connect(self.globalNamespaceAction)

        stateNamespaceAction = QAction('&State Namespace', self)
        stateNamespaceAction.setShortcut('Ctrl+T')
        stateNamespaceAction.setStatusTip('Open State Namespace')
        stateNamespaceAction.triggered.connect(self.localNamespaceAction)

        # actions menu
        librariesAction = QAction('&Libraries', self)
        librariesAction.setShortcut('Ctrl+L')
        librariesAction.setStatusTip('Add additional libraries')
        librariesAction.triggered.connect(self.librariesAction)

        configFileAction = QAction('&ROS Config', self)
        configFileAction.setShortcut('Ctrl+R')
        configFileAction.setStatusTip('Edit ROS configuration')
        configFileAction.triggered.connect(self.configFileAction)

        generateCppAction = QAction('&Generate C++', self)
        generateCppAction.setShortcut('Ctrl+U')
        generateCppAction.setStatusTip('Generate C++ code')
        generateCppAction.triggered.connect(self.generateCppAction)

        generatePythonAction = QAction('&Generate Python', self)
        generatePythonAction.setShortcut('Ctrl+Y')
        generatePythonAction.setStatusTip('Generate Python code')
        generatePythonAction.triggered.connect(self.generatePythonAction)

        # help menu
        aboutAction = QAction('&About', self)
        aboutAction.setShortcut('F1')
        aboutAction.setStatusTip('Information about VisualStates')
        aboutAction.triggered.connect(self.aboutAction)

        # create main menu
        menubar = self.menuBar()
        archieveMenu = menubar.addMenu('&File')
        archieveMenu.addAction(newAction)
        archieveMenu.addAction(openAction)
        archieveMenu.addAction(importAction)
        archieveMenu.addAction(saveAction)
        archieveMenu.addAction(saveAsAction)
        archieveMenu.addAction(quitAction)

        figuresMenu = menubar.addMenu('&Figures')
        figuresMenu.addAction(stateAction)
        figuresMenu.addAction(transitionAction)

        dataMenu = menubar.addMenu('&Data')
        dataMenu.addAction(timerAction)
        dataMenu.addAction(globalNamespaceAction)
        dataMenu.addAction(stateNamespaceAction)

        actionsMenu = menubar.addMenu('&Actions')
        actionsMenu.addAction(librariesAction)
        actionsMenu.addAction(configFileAction)
        actionsMenu.addAction(generateCppAction)
        # actionsMenu.addAction(compileCppAction)
        actionsMenu.addAction(generatePythonAction)

        helpMenu = menubar.addMenu('&Help')
        helpMenu.addAction(aboutAction)

    def newAction(self):
        self.automataScene.clearScene()
        self.treeModel.removeAll()

        self.fileManager.setPath("")

        # create new root state
        self.globalNamespace = Namespace('', '')
        self.localNamespace = Namespace('', '')
        self.rootState = State(0, 'root', True, self.localNamespace)

        self.automataScene.setActiveState(self.rootState)

        self.automataScene.resetIndexes()

        self.libraries = []
        self.config = None

    def openAction(self):
        fileDialog = QFileDialog(self)
        fileDialog.setWindowTitle("Open VisualStates File")
        fileDialog.setViewMode(QFileDialog.Detail)
        fileDialog.setNameFilters(['VisualStates File (*.xml)'])
        fileDialog.setDefaultSuffix('.xml')
        fileDialog.setAcceptMode(QFileDialog.AcceptOpen)
        if fileDialog.exec_():
            self.openFile(fileDialog.selectedFiles()[0])

    def openFile(self, fileName):
        (rootState, config, libraries, globalNamespace) = self.fileManager.open(fileName)
        if rootState is not None:
            (self.rootState, self.config, self.libraries, self.globalNamespace) = (rootState, config, libraries, globalNamespace)
            self.automataPath = self.fileManager.fullPath
            self.treeModel.removeAll()
            self.treeModel.loadFromRoot(self.rootState)
            # set the active state as the loaded state
            self.automataScene.setActiveState(self.rootState)
            self.automataScene.setLastIndexes(self.rootState)
        else:
            self.showWarning("Wrong file selected",
                             "The selected file is not a valid VisualStates file")


    def saveAction(self):
        if len(self.fileManager.getFileName()) == 0:
            self.saveAsAction()
        else:
            self.fileManager.save(self.rootState, self.config, self.libraries, self.globalNamespace)

    def saveAsAction(self):
        fileDialog = QFileDialog(self)
        fileDialog.setWindowTitle("Save VisualStates Project")
        fileDialog.setViewMode(QFileDialog.Detail)
        fileDialog.setNameFilters(['VisualStates File (*.xml)'])
        fileDialog.setAcceptMode(QFileDialog.AcceptSave)
        if fileDialog.exec_():
            self.fileManager.setFullPath(fileDialog.selectedFiles()[0])
            self.fileManager.save(self.rootState, self.config, self.libraries, self.globalNamespace)

    def quitAction(self):
        # print('Quit')
        self.close()

    def stateAction(self):
        self.automataScene.setOperationType(OpType.ADDSTATE)

    def transitionAction(self):
        self.automataScene.setOperationType(OpType.ADDTRANSITION)

    def importAction(self):
        fileDialog = QFileDialog(self)
        fileDialog.setWindowTitle("Import VisualStates File")
        fileDialog.setViewMode(QFileDialog.Detail)
        fileDialog.setNameFilters(['VisualStates File (*.xml)'])
        fileDialog.setDefaultSuffix('.xml')
        fileDialog.setAcceptMode(QFileDialog.AcceptOpen)
        if fileDialog.exec_():
            tempPath = self.fileManager.getFullPath()
            file = self.fileManager.open(fileDialog.selectedFiles()[0])
            if file[0] is not None:
                self.fileManager.setPath(tempPath)
                # if the current active state already has an initial state make sure that
                # there will not be any initial state in the imported state
                if self.activeState.getInitialChild() is not None:
                    for childState in file[0].getChildren():
                        childState.setInitial(False)

                # Update importing Namespaces
                importedState, self.config, self.libraries, self.globalNamespace = self.importManager.updateAuxiliaryData(file, self)
                self.treeModel.loadFromRoot(importedState, self.activeState)
                self.automataScene.displayState(self.activeState)
                self.automataScene.setLastIndexes(self.rootState)
            else:
                self.showWarning("Wrong file selected",
                                 "The selected file is not a valid VisualStates file")

    def timerAction(self):
        if self.activeState is not None:
            timerDialog = TimerDialog('Time Step Duration', str(self.activeState.getTimeStep()))
            timerDialog.timeChanged.connect(self.timeStepDurationChanged)
            timerDialog.exec_()

    def globalNamespaceAction(self):
        self.globalNamespaceDialog = NamespaceDialog('Global Namespace', self.globalNamespace)
        self.globalNamespaceDialog.namespaceChanged.connect(self.globalNamespaceChanged)
        self.globalNamespaceDialog.exec_()

    def localNamespaceAction(self):
        self.localNamespaceDialog = NamespaceDialog('Local Namespace', self.activeNamespace)
        self.localNamespaceDialog.namespaceChanged.connect(self.localNamespaceChanged)
        self.localNamespaceDialog.exec_()

    def librariesAction(self):
        librariesDialog = LibrariesDialog('Libraries', self.libraries)
        librariesDialog.librariesChanged.connect(self.librariesChanged)
        librariesDialog.exec_()

    def configFileAction(self):
        if self.config is None:
            self.config = RosConfig()
        self.configDialog = RosConfigDialog('Config', self.config)
        self.configDialog.exec_()

    def showWarning(self, title, msg):
        QMessageBox.warning(self, title, msg)

    def showInfo(self, title, msg):
        QMessageBox.information(self, title, msg)

    def generateCppAction(self):
        stateList = []
        if self.fileManager.hasFile():
            self.getStateList(self.rootState, stateList)
            if self.config is None:
                self.config = RosConfig()
            generator = CppRosGenerator(self.libraries, self.config, stateList, self.globalNamespace)
            generator.generate(self.fileManager.getPath(), self.fileManager.getFileName())
            self.showInfo('C++ Code Generation', 'C++ code generation is successful.')
        else:
            self.showWarning('C++ Generation', 'Please save the project before code generation.')

    # def compileCppAction(self):
    #     # print('compile cpp action')
    #     pass

    def generatePythonAction(self):
        stateList = []
        if self.fileManager.hasFile():
            self.getStateList(self.rootState, stateList)
            if self.config is None:
                self.config = RosConfig()
            generator = PythonRosGenerator(self.libraries, self.config, stateList, self.globalNamespace)
            generator.generate(self.fileManager.getPath(), self.fileManager.getFileName())
            self.showInfo('Python Code Generation', 'Python code generation is successful.')
        else:
            self.showWarning('Python Generation', 'Please save the project before code generation.')

    def aboutAction(self):
        aboutDialog = AboutDialog()
        aboutDialog.exec_()

    def createTreeView(self):
        dockWidget = QDockWidget()
        dockWidget.setAllowedAreas(Qt.LeftDockWidgetArea)
        dockWidget.setFeatures(QDockWidget.NoDockWidgetFeatures)
        dockWidget.setTitleBarWidget(QWidget())
        self.treeView = QTreeView()
        self.treeView.clicked.connect(self.treeItemClicked)
        self.treeModel = TreeModel()
        self.treeView.setModel(self.treeModel)

        self.upButton = QPushButton()
        self.upButton.setText('Up')
        self.upButton.clicked.connect(self.upButtonClicked)

        leftContainer = QWidget()
        leftLayout = QVBoxLayout()
        leftLayout.addWidget(self.treeView)
        leftLayout.addWidget(self.upButton)
        leftContainer.setLayout(leftLayout)

        dockWidget.setWidget(leftContainer)
        self.addDockWidget(Qt.LeftDockWidgetArea, dockWidget)

    def createStateCanvas(self):
        self.stateCanvas = QGraphicsView()
        self.automataScene = AutomataScene()
        self.automataScene.setSceneRect(0, 0, 2000, 2000)
        self.automataScene.activeStateChanged.connect(self.activeStateChanged)
        self.automataScene.activeNamespaceChanged.connect(self.activeNamespaceChanged)
        self.automataScene.stateInserted.connect(self.stateInserted)
        self.automataScene.stateRemoved.connect(self.stateRemoved)
        self.automataScene.stateImported.connect(self.stateImported)
        self.automataScene.transitionInserted.connect(self.transitionInserted)
        self.automataScene.stateNameChangedSignal.connect(self.stateNameChanged)
        self.automataScene.setActiveState(self.rootState)

        self.setCentralWidget(self.stateCanvas)
        self.stateCanvas.setScene(self.automataScene)
        self.stateCanvas.setRenderHint(QPainter.Antialiasing)
        self.stateCanvas.setAcceptDrops(True)

    def stateInserted(self, state):
        if self.activeState != self.rootState:
            parent = self.treeModel.getByDataId(self.activeState.id)
            self.treeModel.insertState(state, QColor(Qt.white), parent)
        else:
            self.treeModel.insertState(state, QColor(Qt.white))

    def stateRemoved(self, state):
        if self.activeState != self.rootState:
            parent = self.treeModel.getByDataId(self.activeState.id)
            self.treeModel.removeState(state.stateData, parent)
        else:
            self.treeModel.removeState(state.stateData)

    def stateImported(self):
        self.importAction()

    def transitionInserted(self, tran):
        # print('transition inserted:' + tran.transitionData.name)
        pass

    def stateNameChanged(self, state):
        dataItem = self.treeModel.getByDataId(state.stateData.id)
        if dataItem != None:
            dataItem.name = state.stateData.name
            self.treeModel.layoutChanged.emit()

    def activeStateChanged(self):
        if self.automataScene.activeState != self.activeState:
            # print('visual states active state changed:' + self.automataScene.activeState.name)
            self.activeState = self.automataScene.activeState
            if self.activeState == self.rootState:
                self.treeView.selectionModel().clearSelection()
            else:
                self.treeView.setCurrentIndex(self.treeModel.indexOf(self.treeModel.getByDataId(self.activeState.id)))

    def activeNamespaceChanged(self):
        if self.automataScene.activeNamespace != self.activeNamespace:
            self.activeNamespace = self.automataScene.activeNamespace

    def upButtonClicked(self):
        if self.activeState != None:
            if self.activeState.parent != None:
                #print(self.activeState.parent.id)
                self.automataScene.setActiveState(self.activeState.parent)

    def getStateById(self, state, id):
        if state.id == id:
            return state
        else:
            result = None
            for child in state.getChildren():
                result = self.getStateById(child, id)
                if result is not None:
                    return result
            return result

    def treeItemClicked(self, index):
        # print('clicked item.id:' + str(index.internalPointer().id))
        state = self.getStateById(self.rootState, index.internalPointer().id)
        if state is not None:
            # set the active state as the loaded state
            self.automataScene.setActiveState(state)

    def timeStepDurationChanged(self, duration):
        if self.activeState is not None:
            self.activeState.setTimeStep(duration)

    def librariesChanged(self, libraries):
        self.libraries = libraries

    def globalNamespaceChanged(self):
        if self.globalNamespaceDialog:
            self.globalNamespace = self.globalNamespaceDialog.getNamespace()

    def localNamespaceChanged(self):
        if self.localNamespaceDialog:
            self.activeNamespace = self.localNamespaceDialog.getNamespace()

    def getStateList(self, state, stateList):
        if len(state.getChildren()) > 0:
            stateList.append(state)

        for s in state.getChildren():
            self.getStateList(s, stateList)
