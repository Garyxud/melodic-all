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
             Pushkal Katara (katarapushkal@gmail.com)

  '''

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QDialog, QLineEdit, QVBoxLayout, QHBoxLayout, QPushButton, \
    QWidget, QApplication, QLabel, QComboBox, QRadioButton, \
    QFormLayout, QTabWidget, QPlainTextEdit, QInputDialog, QFileDialog, QMessageBox
from PyQt5.QtGui import QFontDatabase, QColor, QFontMetrics
from PyQt5.Qsci import QsciScintilla, QsciLexerPython, QsciLexerCPP

class NamespaceDialog(QDialog):
    namespaceChanged = pyqtSignal()

    def __init__(self, name, namespace):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.resize(800,600)

        self.namespace = namespace

        mainLayout = QVBoxLayout()

        self.pythonButton = QRadioButton('Python')
        self.pythonButton.setChecked(True)
        self.pythonButton.clicked.connect(self.pythonClicked)
        self.cppButton = QRadioButton('C++')
        self.cppButton.clicked.connect(self.cppClicked)

        hLayout0 = QHBoxLayout()
        hLayout0.addWidget(self.pythonButton)
        hLayout0.addWidget(self.cppButton)
        container0 = QWidget()
        container0.setLayout(hLayout0)
        mainLayout.addWidget(container0)

        self.language = 'python'

        self.tabWidget = QTabWidget()

        self.functionTab = FunctionsTab(self.namespace.getFunctions())
        self.functionTab.functionsChanged.connect(self.functionsChanged)
        self.tabWidget.addTab(self.functionTab, 'Functions')

        self.variableTab = VariablesTab(self.namespace.getVariables())
        self.variableTab.variablesChanged.connect(self.variablesChanged)
        self.tabWidget.addTab(self.variableTab, 'Variables')

        mainLayout.addWidget(self.tabWidget)

        self.setLayout(mainLayout)

    def functionsChanged(self, functions):
        self.namespace.setFunctions(functions)
        self.namespaceChanged.emit()

    def variablesChanged(self, variables):
        self.namespace.setVariables(variables)
        self.namespaceChanged.emit()

    def setNamespace(self, namespace):
        self.namespace = namespace

    def getNamespace(self):
        return self.namespace

    def pythonClicked(self):
        #TODO
        pass

    def cppClicked(self):
        #TODO
        pass

class FunctionsTab(QDialog):
    functionsChanged = pyqtSignal('QString')

    def __init__(self, functions):
        super(QDialog, self).__init__()

        self.codeDialog = CodeDialog(functions)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(self.codeDialog)
        self.setLayout(verticalLayout)

        self.codeDialog.codeChanged.connect(self.changeFunctions)

    def changeFunctions(self, functions):
        self.functionsChanged.emit(functions)


class VariablesTab(QDialog):
    variablesChanged = pyqtSignal('QString')

    def __init__(self, variables):
        super(QDialog, self).__init__()

        self.codeDialog = CodeDialog(variables)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(self.codeDialog)
        self.setLayout(verticalLayout)

        self.codeDialog.codeChanged.connect(self.changeVariables)

    def changeVariables(self, variables):
        self.variablesChanged.emit(variables)

class CodeDialog(QDialog):
    codeChanged = pyqtSignal('QString')

    def __init__(self, code):
        super(QDialog, self).__init__()

        self.codeEdit = QsciScintilla()
        self.codeEdit.setText(code)
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.codeEdit.setFont(fixedWidthFont)
        fontmetrics = QFontMetrics(fixedWidthFont)
        self.codeEdit.setMarginWidth(0, fontmetrics.width("000"))
        self.codeEdit.setMarginLineNumbers(0, True)
        self.codeEdit.setMarginsBackgroundColor(QColor("#cccccc"))

        self.codeEdit.setBraceMatching(QsciScintilla.SloppyBraceMatch)
        self.codeEdit.setCaretLineVisible(True)
        self.codeEdit.setCaretLineBackgroundColor(QColor("#ffe4e4"))
        lexer = QsciLexerPython()
        lexer.setDefaultFont(fixedWidthFont)
        self.codeEdit.setLexer(lexer)
        self.codeEdit.SendScintilla(QsciScintilla.SCI_SETHSCROLLBAR, 0)
        self.codeEdit.setUtf8(True)

        self.codeEdit.setTabWidth(4)
        self.codeEdit.setIndentationsUseTabs(True)
        self.codeEdit.setIndentationGuides(True)
        self.codeEdit.setTabIndents(True)
        self.codeEdit.setAutoIndent(True)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(self.codeEdit)
        self.setLayout(verticalLayout)

        self.codeEdit.textChanged.connect(self.changeCode)

    def changeCode(self):
        self.codeChanged.emit(self.codeEdit.text())
