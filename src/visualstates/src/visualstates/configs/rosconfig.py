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

from visualstates.generators.pythonrosgenerator import PythonRosGenerator

class RosConfig(object):

    PUBLISH = 'Publish'
    SUBSCRIBE = 'Subscribe'

    def __init__(self):
        self.topics = []
        self.buildDependencies = []
        self.runDependencies = []

    def updateROSConfig(self, newConfig):
        self.updateTopics(newConfig.topics)
        self.updateBuildDependencies(newConfig.buildDependencies)
        self.updateRunDependencies(newConfig.runDependencies)

    def getTopics(self):
        return self.topics

    def isTopicByName(self, topicName):
        topicNames = map(lambda x: x['name'], self.topics)
        if topicName in topicNames:
            return False
        else:
            return True

    def addTopic(self, id, topic):
        newTopic = {}
        newTopic['id'] = id
        if topic['opType'] == RosConfig.PUBLISH:
            newTopic['methodname'] = topic['methodname']
        elif topic['opType'] == RosConfig.SUBSCRIBE:
            newTopic['variablename'] = topic['variablename']
        newTopic['name'] = topic['name']
        newTopic['type'] = topic['type']
        newTopic['opType'] = topic['opType']
        self.topics.append(newTopic)

    def updateTopics(self, topics):
        for topic in topics:
            if topic not in self.topics:
                self.addTopic(self.getTopicID(), topic)

    def getTopicID(self):
        if self.topics:
            return max(map(lambda x: x['id'], self.topics)) + 1
        else:
            return 0

    def setBuildDependencies(self, dependencies):
        """Dependencies coming as strings"""
        self.buildDependencies = []
        dependStrs = dependencies.split('\n')
        for dStr in dependStrs:
            if len(dStr.strip()) > 0:
                self.buildDependencies.append(dStr.strip())

    def getBuildDependencies(self):
        return self.buildDependencies

    def updateBuildDependencies(self, dependencies):
        """Dependencies coming as List"""
        for dep in dependencies:
            if dep not in self.buildDependencies:
                self.buildDependencies.append(dep)

    def getRunDependencies(self):
        return self.runDependencies

    def updateRunDependencies(self, dependencies):
        """Dependencies coming as List"""
        for dep in dependencies:
            if dep not in self.runDependencies:
                self.runDependencies.append(dep)

    def createNode(self, doc):
        cfgElement = doc.createElement('config')

        bDependencies = doc.createElement('buildDependencies')
        for bDepend in self.buildDependencies:
            dependElement = doc.createElement('dependency')
            dependElement.appendChild(doc.createTextNode(bDepend))
            bDependencies.appendChild(dependElement)
        cfgElement.appendChild(bDependencies)

        rDependencies = doc.createElement('runDependencies')
        for rDepend in self.runDependencies:
            dElement = doc.createElement('dependency')
            dElement.appendChild(doc.createTextNode(rDepend))
            rDependencies.appendChild(dElement)
        cfgElement.appendChild(rDependencies)

        tElements = doc.createElement('topics')
        for t in self.topics:
            tElement = doc.createElement('topic')
            tElement.setAttribute('id', str(t['id']))
            if t['opType'] == RosConfig.PUBLISH:
                methodElement = doc.createElement('methodname')
                methodElement.appendChild(doc.createTextNode(t['methodname']))
                tElement.appendChild(methodElement)
            elif t['opType'] == RosConfig.SUBSCRIBE:
                varElement = doc.createElement('variablename')
                varElement.appendChild(doc.createTextNode(t['variablename']))
                tElement.appendChild(varElement)
            nameElement = doc.createElement('name')
            nameElement.appendChild(doc.createTextNode(t['name']))
            tElement.appendChild(nameElement)
            typeElement = doc.createElement('type')
            typeElement.appendChild(doc.createTextNode(t['type']))
            tElement.appendChild(typeElement)
            opElement = doc.createElement('opType')
            opElement.appendChild(doc.createTextNode(t['opType']))
            tElement.appendChild(opElement)

            tElements.appendChild(tElement)

        cfgElement.appendChild(tElements)

        return cfgElement

    def loadNode(self, node):
        self.buildDependencies = []
        bDependencies = node.getElementsByTagName('buildDependencies')[0]
        for bDependency in bDependencies.getElementsByTagName('dependency'):
            if len(bDependency.childNodes) > 0:
                self.buildDependencies.append(bDependency.childNodes[0].nodeValue)
            # print('bdepend:' + bDependency.childNodes[0].nodeValue)

        self.runDependencies = []
        rDependencies = node.getElementsByTagName('runDependencies')[0]
        for rDependency in rDependencies.getElementsByTagName('dependency'):
            if len(rDependency.childNodes) > 0:
                self.runDependencies.append(rDependency.childNodes[0].nodeValue)
            # print('rdepend:' + rDependency.childNodes[0].nodeValue)

        self.topics = []
        tElements = node.getElementsByTagName('topics')[0]
        for t in tElements.getElementsByTagName('topic'):
            topic = {}
            topic['id'] = int(t.getAttribute('id'))
            topic['name'] = t.getElementsByTagName('name')[0].childNodes[0].nodeValue
            topic['type'] = t.getElementsByTagName('type')[0].childNodes[0].nodeValue
            topic['opType'] = t.getElementsByTagName('opType')[0].childNodes[0].nodeValue
            if topic['opType'] == RosConfig.PUBLISH:
                methodnames = t.getElementsByTagName('methodname')
                if len(methodnames) > 0:
                    topic['methodname'] = methodnames[0].childNodes[0].nodeValue
                else:
                    topic['methodname'] = self.getVarName(topic['name'])
            elif topic['opType'] == RosConfig.SUBSCRIBE:
                varnames = t.getElementsByTagName('variablename')
                if len(varnames) > 0:
                    topic['variablename'] = varnames[0].childNodes[0].nodeValue
                else:
                    topic['variablename'] = self.getVarName(topic['name'])

            self.topics.append(topic)

    def getVarName(self, varName):
        varName = varName.replace('/', '_')
        if varName[0] == '_':
            varName = varName[1:]
        return varName
