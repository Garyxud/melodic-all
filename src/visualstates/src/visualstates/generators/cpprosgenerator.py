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
import stat
from xml.dom import minidom

from visualstates.gui.transition.transitiontype import TransitionType
from visualstates.parsers.cppparser import CPPParser
from visualstates.generators.basegenerator import BaseGenerator


class CppRosGenerator(BaseGenerator):
    def __init__(self, libraries, config, states, globalNamespace):
        BaseGenerator.__init__(self, libraries, config, states, globalNamespace)

    def generate(self, projectPath, projectName):
        # create source dir if not exists
        if not os.path.exists(projectPath + os.sep + 'src'):
            os.makedirs(projectPath + os.sep + 'src')

        stringList = []
        self.generateHeaders(stringList, projectName)
        self.generateGlobalNamespaceClass(stringList, self.config, self.globalNamespace)
        self.generateStateNamespaceClass(stringList)
        self.generateStateClasses(stringList)
        self.generateTransitionClasses(stringList)
        stringList.append('#endif')
        sourceCode = ''.join(stringList)
        fp = open(projectPath + os.sep + 'src' + os.sep + projectName + '.h', 'w')
        fp.write(sourceCode)
        fp.close()

        stringList = []
        self.generateHeadersForCpp(stringList, projectName)
        self.generateStateMethods(stringList)
        self.generateTranMethods(stringList)
        self.generateGlobalNamespaceMethods(stringList, self.config, self.globalNamespace)
        self.generateStateNamespaceMethods(stringList)
        self.generateMain(stringList, projectName)
        sourceCode = ''.join(stringList)

        fp = open(projectPath + os.sep + 'src' + os.sep + projectName + '.cpp', 'w')
        fp.write(sourceCode)
        fp.close()

        stringList = []
        self.generateRunTimeGui(stringList)
        sourceCode = ''.join(stringList)
        fp = open(projectPath + os.sep + projectName + '_runtime.py', 'w')
        fp.write(sourceCode)
        fp.close()
        # make runtime gui python file executable
        os.chmod(projectPath + os.sep + projectName + '_runtime.py', stat.S_IEXEC | stat.S_IXOTH | stat.S_IWRITE | stat.S_IREAD)

        stringList = []
        self.generateCmake(stringList, projectName, self.config)
        cmakeString = ''.join(stringList)
        fp = open(projectPath + os.sep + 'CMakeLists.txt', 'w')
        fp.write(cmakeString)
        fp.close()

        xmlDoc = self.generatePackageXml(self.config, projectName)
        xmlStr = xmlDoc.toprettyxml(indent='  ')
        with open(projectPath + os.sep + 'package.xml', 'w') as f:
            f.write(xmlStr)

    def generateHeaders(self, headers, projectName):
        headers.append('#ifndef ' + projectName + '_H\n')
        headers.append('#define ' + projectName + '_H\n\n')

        headers.append('#include <ros/ros.h>\n')
        headers.append('#include <visualstates/state.h>\n')
        headers.append('#include <visualstates/temporaltransition.h>\n')
        headers.append('#include <visualstates/conditionaltransition.h>\n')

        for lib in self.libraries:
            headers.append('#include <')
            headers.append(lib.strip('\n'))
            headers.append('>\n')

        # generate ros message headers
        typeSet = {''} # create set
        for topic in self.config.getTopics():
            if topic['type'] not in typeSet:
                headers.append('#include <' + topic['type'] + '.h>\n')
                typeSet.add(topic['type'])

        headers.append('\n')

        return headers

    def generateGlobalNamespaceClass(self, classStr, config, globalNamespace):
        classStr.append('class GlobalNamespace {\n')
        classStr.append('private:\n')
        classStr.append('\tros::NodeHandle nh;\n')
        classStr.append('\tros::Rate rate;\n')
        classStr.append('\tpthread_t thread;\n\n')

        for topic in config.getTopics():
            varName = topic['name'].replace('/', '_')
            if varName[0] == '_':
                varName = varName[1:]

            if topic['opType'] == 'Publish':
                classStr.append('\tros::Publisher ' + varName + 'Pub;\n')
            elif topic['opType'] == 'Subscribe':
                classStr.append('\tros::Subscriber ' + varName + 'Sub;\n')
                type = topic['type']
                types = type.split('/')
                if len(types) == 2:
                    classStr.append('\t' + types[0] + '::' + types[1] + ' last' + varName + ';\n')
                    classStr.append('\tvoid ' + varName + 'Callback(const ' + types[0] + '::' + types[1] + '& ' + varName + ');\n')
                else:
                    classStr.append('\t' + type + ' last' + varName + ';\n')
                    classStr.append('\tvoid ' + varName + 'Callback(const ' + type + '& ' + varName + ');\n')
        classStr.append('\n\n')
        classStr.append('public:\n')
        classStr.append('\tGlobalNamespace(int nodeRate);\n')
        classStr.append('\tvoid startThread();\n')
        classStr.append('\tstatic void* threadRunner(void*);\n')
        classStr.append('\tvoid run();\n')
        classStr.append('\tvoid join();\n')
        classStr.append('\tvoid stop();\n\n')

        for topic in config.getTopics():
            varName = topic['name'].replace('/', '_')
            if varName[0] == '_':
                varName = varName[1:]

            if topic['opType'] == 'Subscribe':
                type = topic['type']
                types = type.split('/')
                if len(types) == 2:
                    classStr.append('\t' + types[0] + '::' + types[1] + '& get' + varName + '();\n')
                else:
                    classStr.append('\t' + type + ' get' + varName + '();\n')
            elif topic['opType'] == 'Publish':
                type = topic['type']
                types = type.split('/')
                if len(types) == 2:
                    classStr.append('\tvoid publish' + varName + '(' + types[0] + '::' + types[1] + '& ' + varName + ');\n')
                else:
                    classStr.append('\tvoid publish' + varName + '(' + type + '& ' + varName + ');\n')
        classStr.append('\n')

        # define variables
        types, varNames, initialValues = CPPParser.parseVariables(globalNamespace.variables)
        for i in range(len(types)):
            classStr.append('\t' + types[i] + ' ' + varNames[i] + ';\n')
        classStr.append('\n')

        returnTypes, funcNames, codes = CPPParser.parseFunctions(globalNamespace.functions)
        for i in range(len(returnTypes)):
            classStr.append('\t' + returnTypes[i] + ' ' + funcNames[i] + ';\n')

        classStr.append('};\n\n')

    def generateStateNamespaceClass(self, classStr):
        for state in self.getAllStates():
            classStr.append('class Namespace' + str(state.id) + ' {\n')
            classStr.append('public:\n')
            classStr.append('\tGlobalNamespace* globalNamespace;\n')
            types, varNames, initialValues = CPPParser.parseVariables(state.getNamespace().variables)
            for i in range(len(types)):
                classStr.append(types[i] + ' ' + varNames[i] + ';\n')
            classStr.append('\n')
            classStr.append('public:\n')
            classStr.append('\tNamespace'+str(state.id)+'(GlobalNamespace* _globalNamespace) {\n')
            classStr.append('\t\tglobalNamespace = _globalNamespace;\n')
            for i in range(len(types)):
                classStr.append('\t\t'+varNames[i] + ' = ' + initialValues[i] + ';\n')
            classStr.append('\t}\n')
            returnTypes, funcNames, codes = CPPParser.parseFunctions(state.getNamespace().functions)
            for i in range(len(returnTypes)):
                classStr.append(returnTypes[i] + ' ' + funcNames[i] + ';\n')
            classStr.append('};\n\n')

    def generateStateClasses(self, classStr):
        for state in self.getAllStates():
            classStr.append('class State' + str(state.id) + ' : public State {\n')
            classStr.append('public:\n')
            classStr.append('\tGlobalNamespace* globalNamespace;\n')
            if state.parent is not None:
                classStr.append('\tNamespace' + str(state.parent.id)+ '* stateNamespace;\n')
                classStr.append('\tState' + str(state.id) + '(int id, bool initial, GlobalNamespace* _globalNamespace,'\
                                ' Namespace'+str(state.parent.id)+'* _stateNamespace, int cycleDuration, State* parent, RunTimeGui* gui):\n')
                classStr.append(
                    '\t\tState(id, initial, cycleDuration, parent, gui) {\nglobalNamespace = _globalNamespace;\nstateNamespace = _stateNamespace;\n}\n')
            else:
                classStr.append('\tState' + str(state.id) + '(int id, bool initial, GlobalNamespace* _globalNamespace,\
int cycleDuration, State* parent, RunTimeGui* gui):\n')
                classStr.append(
                    '\t\tState(id, initial, cycleDuration, parent, gui) {\nglobalNamespace = _globalNamespace;\n}\n')
            classStr.append('\tvirtual void runCode();\n')
            classStr.append('};\n\n')

    def generateStateMethods(self, stateStrList):
        for state in self.getAllStates():
            stateStrList.append('void State' + str(state.id) + '::runCode() {\n')
            for codeLine in state.getCode().split('\n'):
                stateStrList.append('\t' + codeLine + '\n')
            stateStrList.append('}\n\n')

    def generateTransitionClasses(self, classStr):
        for tran in self.getAllTransitions():
            if tran.getType() == TransitionType.CONDITIONAL:
                classStr.append('class Tran' + str(tran.id) + ' : public ConditionalTransition {\n')
                classStr.append('\tpublic:\n')
                classStr.append('\tGlobalNamespace* globalNamespace;\n')
                if tran.origin.parent is not None:
                    classStr.append('Namespace' + str(tran.origin.parent.id) + '* stateNamespace;\n\n')
                    classStr.append('\tTran' + str(tran.id) + '(int id, int destId, GlobalNamespace* _globalNamespace, \
Namespace'+str(tran.origin.parent.id)+'* _stateNamespace):\n')
                    classStr.append('ConditionalTransition(id, destId) {\nglobalNamespace = _globalNamespace;\n\
stateNamespace = _stateNamespace;}\n')
                else:
                    classStr.append('\tTran' + str(tran.id) + '(int id, int destId, GlobalNamespace* _globalNamespace):\n')
                    classStr.append('ConditionalTransition(id, destId) {globalNamespace = _globalNamespace;}\n')
                classStr.append('\tvirtual void init();\n')
                classStr.append('\tvirtual bool checkCondition();\n')
                classStr.append('\tvirtual void runCode();\n')
                classStr.append('};\n\n')
            elif tran.getType() == TransitionType.TEMPORAL:
                classStr.append('class Tran' + str(tran.id) + ' : public TemporalTransition {\n')
                classStr.append('\tpublic:\n')
                classStr.append('\tTran' + str(tran.id) + '(int id, int destId, int elapsedTime):TemporalTransition(id, destId, elapsedTime) {}\n')
                classStr.append('\tvirtual void runCode();\n')
                classStr.append('};\n\n')

    def generateTranMethods(self, tranStr):
        for tran in self.getAllTransitions():
            if tran.getType() == TransitionType.CONDITIONAL:
                #todo: currently user does not provide init method
                tranStr.append('void Tran' + str(tran.id) + '::init() {\n')
                tranStr.append('}\n\n')
                tranStr.append('bool Tran' + str(tran.id) + '::checkCondition() {\n')
                for codeLine in tran.getCondition().split('\n'):
                    tranStr.append('\t' + codeLine + '\n')
                tranStr.append('}\n')

            tranStr.append('void Tran' + str(tran.id) + '::runCode() {\n')
            for codeLine in tran.getCode().split('\n'):
                tranStr.append('\t' + codeLine + '\n')
            tranStr.append('}\n\n')

    def generateHeadersForCpp(self, headerStr, projectName):
        headerStr.append('#include "' + projectName + '.h"\n')
        headerStr.append('#include <iostream>\n')
        headerStr.append('#include <string>\n')
        headerStr.append('#include <signal.h>\n')
        headerStr.append('#include <visualstates/runtimegui.h>\n\n')

    def generateGlobalNamespaceMethods(self, rosStr, config, globalNamespace):
        rosStr.append('GlobalNamespace::GlobalNamespace(int nodeRate):rate(nodeRate) {\n')
        for topic in config.getTopics():
            varName = topic['name'].replace('/', '_')
            if varName[0] == '_':
                varName = varName[1:]

            type = topic['type']
            types = type.split('/')
            if topic['opType'] == 'Publish':
                if len(types) == 2:
                    rosStr.append('\t' + varName + 'Pub = nh.advertise<' + types[0] + '::' + types[1] + '>("' + topic['name'] + '", 10);\n')
                else:
                    rosStr.append('\t' + varName + 'Pub = nh.advertise<' + type + '>("' + topic[
                        'name'] + '", 10);\n')
            elif topic['opType'] == 'Subscribe':
                rosStr.append('\t' + varName + 'Sub = nh.subscribe("' + topic['name'] + '", 10, &GlobalNamespace::'+varName+'Callback, this);\n')

        # set inital values of variables
        types, varNames, initialValues = CPPParser.parseVariables(globalNamespace.variables)
        for i in range(len(types)):
            if initialValues[i] is not None:
                rosStr.append('\t' + varNames[i] + ' = ' + initialValues[i] + ';\n')

        rosStr.append('}\n\n')

        rosStr.append('void* GlobalNamespace::threadRunner(void* owner) {\n')
        rosStr.append('\t((GlobalNamespace*)owner)->run();\n')
        rosStr.append('}\n\n')

        rosStr.append('void GlobalNamespace::startThread() {\n')
        rosStr.append('\tpthread_create(&thread, NULL, &GlobalNamespace::threadRunner, this);\n')
        rosStr.append('}\n\n')

        rosStr.append('void GlobalNamespace::run() {\n')
        rosStr.append('\twhile(nh.ok()) {\n')
        rosStr.append('\t\tros::spinOnce();\n')
        rosStr.append('\t\trate.sleep();\n')
        rosStr.append('\t}\n')
        rosStr.append('}\n\n')

        rosStr.append('void GlobalNamespace::join() {\n')
        rosStr.append('\tpthread_join(thread, NULL);\n')
        rosStr.append('}\n\n')

        rosStr.append('void GlobalNamespace::stop() {\n')
        rosStr.append('\tnh.shutdown();\n')
        rosStr.append('}\n\n')

        for topic in config.getTopics():
            varName = topic['name'].replace('/', '_')
            if varName[0] == '_':
                varName = varName[1:]

            if topic['opType'] == 'Subscribe':
                type = topic['type']
                types = type.split('/')
                if len(types) == 2:
                    rosStr.append('void GlobalNamespace::' + varName + 'Callback(const ' + types[0] + '::' + types[1] + '& ' + varName + ') {\n')
                else:
                    rosStr.append('void GlobalNamespace::' + varName + 'Callback(const ' + type + '& ' + varName + ') {\n')
                rosStr.append('\tlast' + varName + ' = ' + varName + ';\n')
                rosStr.append('}\n\n')

                if len(types) == 2:
                    rosStr.append(types[0] + '::' + types[1] + '& GlobalNamespace::get' + varName + '() {\n')
                else:
                    rosStr.append(type + '& GlobalNamespace::get' + varName + '() {\n')
                rosStr.append('\treturn last' + varName + ';\n')
                rosStr.append('}\n\n')

            elif topic['opType'] == 'Publish':
                type = topic['type']
                types = type.split('/')
                if len(types) == 2:
                    rosStr.append('void GlobalNamespace::publish' + varName + '(' + types[0] + '::' + types[
                        1] + '& ' + varName + ') {\n')
                else:
                    rosStr.append('void GlobalNamespace::publish' + varName + '(' + type + '& ' + varName + ') {\n')
                rosStr.append('\t' + varName + 'Pub.publish(' + varName + ');\n')
                rosStr.append('}\n\n')

        returnTypes, funcNames, codes = CPPParser.parseFunctions(globalNamespace.functions)
        for i in range(len(returnTypes)):
            rosStr.append(returnTypes[i] + ' GlobalNamespace::' + funcNames[i] + '\n')
            rosStr.append(codes[i])
            rosStr.append('\n\n')

    def generateStateNamespaceMethods(self, strList):
        for state in self.getAllStates():
            namespace = state.getNamespace()
            returnTypes, funcNames, codes = CPPParser.parseFunctions(namespace.functions)
            for i in range(len(returnTypes)):
                strList.append(returnTypes[i] + ' Namespace'+str(state.id)+'::' + funcNames[i] + '\n')
                strList.append(codes[i])
                strList.append('\n\n')

    def parentString(self, state):
        if state.parent is None:
            return 'NULL'
        else:
            return 'state'+str(state.parent.id)

    def generateMain(self, mainStr, projectName):
        for state in self.states:
            mainStr.append('State* state' + str(state.id) + ' = NULL;\n')
        mainStr.append('GlobalNamespace* globalNamespace = NULL;\n\n')

        mainStr.append('void signalCallback(int signum) {\n')
        for state in self.states:
            mainStr.append('\tstate' + str(state.id) + '->stop();\n')
        mainStr.append('\tglobalNamespace->stop();\n')
        mainStr.append('}\n\n')

        mainStr.append('int main(int argc, char* argv[]) {\n')
        mainStr.append('\tros::init(argc, argv,"' + projectName + '_node");\n')
        mainStr.append('\tRunTimeGui* runTimeGui = new RunTimeGui();\n\n')
        mainStr.append('\tglobalNamespace = new GlobalNamespace(100);\n')
        mainStr.append('\tglobalNamespace->startThread();\n\n')
        # create state instances
        for state in self.getAllStates():
            mainStr.append('\tNamespace' + str(state.id) + ' stateNamespace' + str(state.id) + '(globalNamespace);\n')
            if state.parent is not None:
                mainStr.append('\tState* state' + str(state.id) + ' = new State' + str(state.id) + '(' +
                               str(state.id) + ', ' + str(state.initial).lower() + ', globalNamespace, &stateNamespace' + str(state.parent.id) + ', ' + str(
                    state.getTimeStep()) +
                               ', ' + self.parentString(state) + ', runTimeGui);\n')
            else:
                mainStr.append('\tstate' + str(state.id) + ' = new State' + str(state.id) + '(' +
                               str(state.id) + ', ' + str(state.initial).lower() + ', globalNamespace, ' + str(state.getTimeStep()) +
                               ', ' + self.parentString(state) + ', runTimeGui);\n')
        mainStr.append('\n')

        # create transition instances
        for tran in self.getAllTransitions():
            if tran.getType() == TransitionType.CONDITIONAL:
                mainStr.append('\tTransition* tran' + str(tran.id) + ' = new Tran' + str(tran.id) + '(' + str(tran.id) +
                           ', ' + str(tran.destination.id) + ', globalNamespace, &stateNamespace'+str(tran.origin.parent.id)+');\n')
            elif tran.getType() == TransitionType.TEMPORAL:
                mainStr.append('\tTransition* tran' + str(tran.id) + ' = new Tran' + str(tran.id) + '(' + str(tran.id) +
                               ', ' + str(tran.destination.id) + ', ' + str(tran.getTemporalTime()) + ');\n')

            mainStr.append('\tstate' + str(tran.origin.id) + '->addTransition(tran' + str(tran.id) + ');\n')
        mainStr.append('\n')

        for state in self.states:
            mainStr.append('\tstate' + str(state.id) + '->startThread();\n')
        mainStr.append('\n')

        mainStr.append('\tsignal(SIGINT, signalCallback);\n')

        for state in self.states:
            mainStr.append('\tstate' + str(state.id) + '->join();\n')
        mainStr.append('\tglobalNamespace->join();\n')
        mainStr.append('}\n')

    def generateRunTimeGui(self, guiStr):
        guiStr.append('#!/usr/bin/python\n')
        guiStr.append('# -*- coding: utf-8 -*-\n')
        #guiStr.append('import visualstates.codegen.python\n')

        guiStr.append('import sys\n')
        guiStr.append('from PyQt5.QtWidgets import QApplication\n')
        guiStr.append('from visualstates.codegen.python.runtimegui import RunTimeGui\n')
        guiStr.append('import rospy\n')
        guiStr.append('from std_msgs.msg import String\n\n')
        guiStr.append('gui = None\n\n')
        guiStr.append('def callback(string_msg):\n')
        guiStr.append('\tglobal gui\n')
        guiStr.append('\tstate_id = int(string_msg.data)\n')
        guiStr.append('\tgui.emitRunningStateById(state_id)\n\n\n')
        guiStr.append('def runGui():\n')
        guiStr.append('\tglobal gui\n')
        guiStr.append('\trospy.init_node("runtime_gui", anonymous=True)\n')
        guiStr.append('\trunningStateSubs = rospy.Subscriber("/runtime_gui", String, callback)\n\n')
        guiStr.append('\tapp = QApplication(sys.argv)\n')
        guiStr.append('\tgui = RunTimeGui()\n\n')

        # create runtime state code
        for state in self.getAllStates():
            guiStr.append('\tgui.addState(' + str(state.id) + ', "' + state.name +
                           '", ' + str(state.initial) + ', ' + str(state.x) + ', ' + str(state.y))
            if state.parent is None:
                guiStr.append(', None)\n')
            else:
                guiStr.append(', ' + str(state.parent.id) + ')\n')
        guiStr.append('\n')

        for tran in self.getAllTransitions():
            guiStr.append('\tgui.addTransition(' + str(tran.id) + ', "' + tran.name + '", ' +
                           str(tran.origin.id) + ', ' + str(tran.destination.id) +
                           ', ' + str(tran.x) + ', ' + str(tran.y) + ')\n')
        guiStr.append('\n')

        guiStr.append('\tgui.emitLoadFromRoot()\n')
        guiStr.append('\tgui.emitActiveStateById(0)\n')
        guiStr.append('\tgui.show()\n')
        guiStr.append('\tapp.exec_()\n\n')

        guiStr.append('if __name__ == "__main__":\n')
        guiStr.append('\trunGui()\n\n')


    def generateCmake(self, cmakeStr, projectName, config):
        cmakeStr.append('project(')
        cmakeStr.append(projectName)
        cmakeStr.append(')\n\n')

        cmakeStr.append('cmake_minimum_required(VERSION 2.8.3)\n\n')

        cmakeStr.append('find_package(catkin REQUIRED COMPONENTS roscpp visualstates\n')
        for dep in config.getBuildDependencies():
            cmakeStr.append('  ' + dep + '\n')
        cmakeStr.append(')\n\n')
        myStr = '''
include_directories(
    ${catkin_INCLUDE_DIRS}
)

'''
        cmakeStr.append(myStr)

        cmakeStr.append('catkin_package()\n')
        cmakeStr.append('add_executable(' + projectName + ' src/' + projectName + '.cpp)\n')
        cmakeStr.append('target_link_libraries(' + projectName + ' ${catkin_LIBRARIES} visualStatesRunTime)\n')
        cmakeStr.append('install(TARGETS ' + projectName + ' RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})\n\n')
        cmakeStr.append('catkin_install_python(PROGRAMS ' + projectName + '_runtime.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})\n\n')
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

        exportElement = doc.createElement('export')
        root.appendChild(exportElement)
        doc.appendChild(root)

        return doc
