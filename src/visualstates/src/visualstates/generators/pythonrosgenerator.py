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
import os
import shutil
from xml.dom import minidom

from visualstates.gui.transition.transitiontype import TransitionType
from visualstates.configs.rospackage import getPackagePath
from visualstates.generators.basegenerator import BaseGenerator


class PythonRosGenerator(BaseGenerator):
    def __init__(self, libraries, config, states, globalNamespace):
        BaseGenerator.__init__(self, libraries, config, states, globalNamespace)

    def generate(self, projectPath, projectName):
        stringList = []
        self.generateImports(stringList)
        self.generateSignalHandling(stringList)
        self.generateGlobalNamespace(stringList, projectName)
        self.generateStateClasses(stringList)
        self.generateTransitionClasses(stringList)
        self.generateMain(stringList)
        sourceCode = ''.join(stringList)
        fp = open(projectPath + os.sep + projectName + '.py', 'w')
        fp.write(sourceCode)
        fp.close()
        os.system('chmod +x "' +projectPath+ '"' + os.sep + projectName + '.py')

        stringList = []
        self.generateCmake(stringList, projectName)
        cmakeString = ''.join(stringList)
        fp = open(projectPath + os.sep + 'CMakeLists.txt', 'w')
        fp.write(cmakeString)
        fp.close()

        xmlDoc = self.generatePackageXml(self.config, projectName)
        xmlStr = xmlDoc.toprettyxml(indent='  ')
        with open(projectPath + os.sep + 'package.xml', 'w') as f:
            f.write(xmlStr)

        # self.copyRuntime(projectPath)

    def generateImports(self, importStr):
        mystr = '''#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys, threading, time, rospy, signal
'''
        importStr.append(mystr)

        typeSet = {''} # create set
        for topic in self.config.getTopics():
            typeStr = topic['type']
            if typeStr not in typeSet:
                if typeStr.find('/') >= 0:
                    types = typeStr.split('/')
                    importStr.append('from ' + types[0] + '.msg import ' + types[1] + '\n')
                else:
                    importStr.append('import ' + typeStr + '\n')
                typeSet.add(typeStr)

        mystr = '''from visualstates.codegen.python.state import State
from visualstates.codegen.python.temporaltransition import TemporalTransition
from visualstates.codegen.python.conditionaltransition import ConditionalTransition
from visualstates.codegen.python.runtimegui import RunTimeGui
from PyQt5.QtWidgets import QApplication

'''
        importStr.append(mystr)
        for lib in self.libraries:
            importStr.append('import ')
            importStr.append(lib)
            importStr.append('\n')
        importStr.append('\n')

        return importStr

    def generateSignalHandling(self, signalStr):
        rootState = self.getRootState()
        signalStr.append('globalNamespace = None\n')
        signalStr.append('state' + str(rootState.id) + ' = None\n')
        signalStr.append('app = None\n\n')
        signalStr.append('def stopRecursive(state):\n')
        signalStr.append('\tstate.stop()\n')
        signalStr.append('\tfor childState in state.states:\n')
        signalStr.append('\t\tstopRecursive(childState)\n\n')
        signalStr.append('def sigintHandler(signal, frame):\n')
        signalStr.append('\tglobal globalNamespace\n')
        signalStr.append('\tglobal state' + str(rootState.id) + '\n')
        signalStr.append('\tglobal app\n')
        signalStr.append('\tif app is not None:')
        signalStr.append('\t\tapp.quit()\n')
        signalStr.append('\tstopRecursive(state' + str(rootState.id) + ')\n')
        signalStr.append('\tglobalNamespace.stop()\n\n')
        signalStr.append('signal.signal(signal.SIGINT, sigintHandler)\n\n')

    def generateStateClasses(self, stateStr):
        for state in self.getAllStates():
            self.generateStateClass(state, stateStr)

    def generateStateClass(self, state, stateStr):
        stateStr.append('class State')
        stateStr.append(str(state.id))
        stateStr.append('(State):\n')

        stateStr.append('\tdef __init__(self, id, initial, globalNamespace, namespace, cycleDuration, parent=None, gui=None):\n')
        stateStr.append('\t\tState.__init__(self, id, initial, cycleDuration, parent, gui)\n')
        stateStr.append('\t\tself.globalNamespace = globalNamespace\n')
        stateStr.append('\t\tself.namespace = namespace\n\n')

        stateStr.append('\tdef runCode(self):\n')
        if len(state.getCode()) > 0:
            for codeLine in state.getCode().split('\n'):
                stateStr.append('\t\t' + codeLine + '\n')
        else:
            stateStr.append('\t\tpass\n')
        stateStr.append('\n\n')

        stateStr.append('class Namespace' + str(state.id) + '():\n')
        stateStr.append('\tdef __init__(self, globalNamespace):\n')
        stateStr.append('\t\tself.globalNamespace = globalNamespace\n')

        if(len(state.namespace.variables) > 0):
            for varLine in state.namespace.variables.split('\n'):
                stateStr.append('\t\t' + varLine + '\n')
        stateStr.append('\n')

        if(len(state.namespace.functions) > 0):
            for funcLine in state.namespace.functions.split('\n'):
                stateStr.append('\t' + funcLine + '\n')
        stateStr.append('\n')

    def generateGlobalNamespace(self, globalNamespaceStr, projectName):
        globalNamespaceStr.append('class GlobalNamespace():\n')
        globalNamespaceStr.append('\tdef __init__(self):\n')
        globalNamespaceStr.append('\t\trospy.init_node("' + projectName + '", anonymous=True, disable_signals=True)\n\n')
        for topic in self.config.getTopics():
            if topic['opType'] == 'Publish':
                typesStr = topic['type']
                types = typesStr.split('/')
                globalNamespaceStr.append('\t\tself.' + topic['methodname'] + 'Pub = rospy.Publisher("' +
                              topic['name'] + '", ' + types[1] + ', queue_size=10)\n')
            elif topic['opType'] == 'Subscribe':
                typesStr = topic['type']
                types = typesStr.split('/')
                globalNamespaceStr.append('\t\tself.' + topic['variablename'] + 'Sub = rospy.Subscriber("' +
                                  topic['name'] + '", ' + types[1] + ', self.' + topic['variablename'] + 'Callback)\n')
                globalNamespaceStr.append('\t\tself.' + topic['variablename'] + ' = ' + types[1] + '()\n')

        # add state variables as part of ros node
        variables = self.globalNamespace.getVariables()
        if len(variables) > 0:
            for varLine in variables.split('\n'):
                globalNamespaceStr.append('\t\t' + varLine + '\n')
            globalNamespaceStr.append('\n')

        globalNamespaceStr.append('\t\ttime.sleep(1) # wait for initialization of the node, subscriber, and publisher\n\n')

        globalNamespaceStr.append('\tdef stop(self):\n')
        globalNamespaceStr.append('\t\trospy.signal_shutdown("exit ROS node")\n\n')

        # define publisher methods and subscriber callbacks
        for topic in self.config.getTopics():
            if topic['opType'] == 'Publish':
                globalNamespaceStr.append('\tdef ' + topic['methodname'] + '(self, _' + topic['methodname'] + '):\n')
                globalNamespaceStr.append('\t\tself.' + topic['methodname'] + 'Pub.publish(_' + topic['methodname'] + ')\n\n')
            elif topic['opType'] == 'Subscribe':
                globalNamespaceStr.append('\tdef ' + topic['variablename'] + 'Callback(self, _' + topic['variablename'] + '):\n')
                globalNamespaceStr.append('\t\tself.' + topic['variablename'] + ' = _' + topic['variablename'] + '\n')
            globalNamespaceStr.append('\n\n')

        # define user functions as part of rosnode
        functions = self.globalNamespace.getFunctions()
        if len(functions) > 0:
            for funcLine in functions.split('\n'):
                globalNamespaceStr.append('\t' + funcLine + '\n')
            globalNamespaceStr.append('\n\n')


    def generateTransitionClasses(self, tranStr):
        for tran in self.getAllTransitions():
            if tran.getType() == TransitionType.CONDITIONAL:
                tranStr.append('class Tran' + str(tran.id) + '(ConditionalTransition):\n')
                tranStr.append('\tdef __init__(self, id, destinationId, globalNamespace, namespace):\n')
                tranStr.append('\t\tConditionalTransition.__init__(self, id, destinationId, globalNamespace, namespace)\n')
                tranStr.append('\tdef checkCondition(self):\n')
                for checkLine in tran.getCondition().split('\n'):
                    tranStr.append('\t\t' + checkLine + '\n')
                tranStr.append('\n')
            elif tran.getType() == TransitionType.TEMPORAL:
                tranStr.append('class Tran' + str(tran.id) + '(TemporalTransition):\n\n')
                tranStr.append('\tdef __init__(self, id, destinationId, elapsedTime, globalNamespace, namespace):\n')
                tranStr.append('\t\tTemporalTransition.__init__(self, id, destinationId, elapsedTime, globalNamespace, namespace)\n')
            tranStr.append('\tdef runCode(self):\n')
            if len(tran.getCode()) > 0:
                for codeLine in tran.getCode().split('\n'):
                    tranStr.append('\t\t' + codeLine + '\n')
                tranStr.append('\n')
            else:
                tranStr.append('\t\tpass\n\n')

    def generateMain(self, mainStr):
        mystr = '''displayGui = False
guiThread = None
gui = None

def readArgs():
\tglobal displayGui
\tfor arg in sys.argv:
\t\tsplitedArg = arg.split('=')
\t\tif splitedArg[0] == '--displaygui':
\t\t\tif splitedArg[1] == 'True' or splitedArg[1] == 'true':
\t\t\t\tdisplayGui = True
\t\t\t\tprint('runtime gui enabled')
\t\t\telse:
\t\t\t\tdisplayGui = False
\t\t\t\tprint('runtime gui disabled')

def runGui():
\tglobal app
\tglobal gui
\tapp = QApplication(sys.argv)
\tgui = RunTimeGui()
\tgui.show()
\tapp.exec_()

'''
        mainStr.append(mystr)

        mainStr.append('if __name__ == "__main__":\n\n')
        mainStr.append('\tglobalNamespace = GlobalNamespace()\n\n')
        mainStr.append('\n')
        mainStr.append('\treadArgs()\n')
        mainStr.append('\tif displayGui:\n')
        mainStr.append('\t\tguiThread = threading.Thread(target=runGui)\n')
        mainStr.append('\t\tguiThread.start()\n\n')

        mainStr.append('\n\tif displayGui:\n')
        mainStr.append('\t\twhile(gui is None):\n')
        mainStr.append('\t\t\ttime.sleep(0.1)\n\n')
        # create runtime gui code
        for state in self.getAllStates():
            mainStr.append('\t\tgui.addState(' + str(state.id) + ', "' + state.name +
                           '", ' + str(state.initial) + ', ' + str(state.x) + ', ' + str(state.y))
            if state.parent == None:
                mainStr.append(', None)\n')
            else:
                mainStr.append(', ' + str(state.parent.id) +')\n')

        mainStr.append('\n')

        for tran in self.getAllTransitions():
            mainStr.append('\t\tgui.addTransition(' + str(tran.id) + ', "' + tran.name + '", ' +
                           str(tran.origin.id) + ', ' + str(tran.destination.id) +
                           ', ' + str(tran.x) + ', ' + str(tran.y) + ')\n')
        mainStr.append('\n')

        mainStr.append('\tif displayGui:\n')
        mainStr.append('\t\tgui.emitLoadFromRoot()\n')
        mainStr.append('\t\tgui.emitActiveStateById(0)\n\n')

        for state in self.getAllStates():
            mainStr.append('\tnamespace' + str(state.id) + ' = Namespace' + str(state.id) +'(globalNamespace)\n')
            mainStr.append('\tstate' + str(state.id) + ' = State' + str(state.id) +
                           '(' + str(state.id) + ', ' + str(state.initial) + ', globalNamespace, ')
            if state.parent == None:
                mainStr.append('None, ' + str(state.getTimeStep()) + ', None, gui)\n')
            else:
                mainStr.append('namespace' + str(state.parent.id)+ ', ' + str(state.getTimeStep()) + ', state' + str(state.parent.id) + ', gui)\n')
        mainStr.append('\n')

        # create and add transitions to their origins
        for tran in self.getAllTransitions():
            if tran.getType() == TransitionType.TEMPORAL:
                mainStr.append('\ttran' + str(tran.id) + ' = Tran' + str(tran.id) +
                               '(' + str(tran.id) + ', ' + str(tran.destination.id) +
                               ', ' + str(tran.getTemporalTime()) + ', globalNamespace, namespace'
                               + str(tran.origin.parent.id) + ')\n')
            elif tran.getType() == TransitionType.CONDITIONAL:
                mainStr.append('\ttran' + str(tran.id) + ' = Tran' + str(tran.id) +
                               '(' + str(tran.id) + ', ' + str(tran.destination.id) + ', globalNamespace, namespace' + str(tran.origin.parent.id) + ')\n')

            mainStr.append('\tstate' + str(tran.origin.id) + '.addTransition(tran' + str(tran.id) + ')\n\n')

        # start threads
        for state in self.states:
            mainStr.append('\tstate' + str(state.id) + '.startThread()\n')

        mainStr.append('\n')
        rootState = self.getRootState()
        mainStr.append('\twhile state' + str(rootState.id) + '.running:\n')
        mainStr.append('\t\ttime.sleep(0.01)\n\n')

    def generateCmake(self, cmakeStr, projectName):
        cmakeStr.append('project(')
        cmakeStr.append(projectName)
        cmakeStr.append(')\n\n')

        cmakeStr.append('cmake_minimum_required(VERSION 2.8.3)\n\n')

        cmakeStr.append('find_package(catkin REQUIRED COMPONENTS visualstates\n')
        for dep in self.config.getBuildDependencies():
            cmakeStr.append('  ' + dep + '\n')
        cmakeStr.append(')\n\n')
        cmakeStr.append('catkin_package()\n')
        cmakeStr.append('include_directories(${catkin_INCLUDE_DIRS})\n')
        cmakeStr.append('install(PROGRAMS ' + projectName + '.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})\n')
        return cmakeStr

    def generatePackageXml(self, config, projectName):
        doc = minidom.Document()
        root = doc.createElement('package')
        nameElement = doc.createElement('name')
        nameElement.appendChild(doc.createTextNode(projectName))
        root.appendChild(nameElement)
        versionElement = doc.createElement('version')
        versionElement.appendChild(doc.createTextNode('0.0.0'))
        root.appendChild(versionElement)
        descElement = doc.createElement('description')
        descElement.appendChild(doc.createTextNode('The ' + projectName + ' package'))
        root.appendChild(descElement)
        maintainerElement = doc.createElement('maintainer')
        maintainerElement.setAttribute('email', 'todo@todo.todo')
        maintainerElement.appendChild(doc.createTextNode('todo'))
        root.appendChild(maintainerElement)
        licenseElement = doc.createElement('license')
        licenseElement.appendChild(doc.createTextNode('TODO (choose one: BSD, MIT, GPLv2, GPLv3 LGPLv3)'))
        root.appendChild(licenseElement)
        btoolDepElement = doc.createElement('buildtool_depend')
        btoolDepElement.appendChild(doc.createTextNode('catkin'))
        root.appendChild(btoolDepElement)
        for bdep in ['visualstates']+config.getBuildDependencies():
            bdepElement = doc.createElement('build_depend')
            bdepElement.appendChild(doc.createTextNode(bdep))
            root.appendChild(bdepElement)

        for rdep in ['visualstates']+config.getRunDependencies():
            rdepElement = doc.createElement('run_depend')
            rdepElement.appendChild(doc.createTextNode(rdep))
            root.appendChild(rdepElement)

        # system dependencies
        rdepElement = doc.createElement('run_depend')
        rdepElement.appendChild(doc.createTextNode('python-qt5-bindings'))
        root.appendChild(rdepElement)

        exportElement = doc.createElement('export')
        root.appendChild(exportElement)
        doc.appendChild(root)

        return doc

    def copyRuntime(self, projectPath):
        if os.path.exists(projectPath + '/codegen'):
            shutil.rmtree(projectPath + '/codegen')
        if os.path.exists(projectPath + '/gui'):
            shutil.rmtree(projectPath + '/gui')
        if os.path.exists(projectPath + '/core'):
            shutil.rmtree(projectPath + '/core')

        shutil.copytree(getPackagePath() + '/lib/python2.7/codegen', projectPath + '/codegen')
        shutil.copytree(getPackagePath() + '/lib/python2.7/gui', projectPath + '/gui')
        shutil.copytree(getPackagePath() + '/lib/python2.7/core', projectPath + '/core')
