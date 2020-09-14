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

   Authors : Pushkal Katara (katarapushkal@gmail.com)

  '''
class Namespace:
    def __init__(self, functions, variables):
        self.functions = functions
        self.variables = variables

    def createNode(self, doc, globalNamespace=False):
        if globalNamespace:
            namespaceElement = doc.createElement('global_namespace')
        else:
            namespaceElement = doc.createElement('namespace')
        functionsElement = doc.createElement('functions')
        functionsElement.appendChild(doc.createTextNode(self.functions))
        namespaceElement.appendChild(functionsElement)
        variablesElement = doc.createElement('variables')
        variablesElement.appendChild(doc.createTextNode(self.variables))
        namespaceElement.appendChild(variablesElement)
        return namespaceElement

    def parseElement(self, elementName, parentElement):
        elements = parentElement.getElementsByTagName(elementName)
        if len(elements) > 0:
            if len(elements[0].childNodes) > 0:
                return elements[0].childNodes[0].nodeValue
        return ''

    def parse(self, namespaceElement):
        self.functions = self.parseElement('functions', namespaceElement)
        self.variables = self.parseElement('variables', namespaceElement)

    def getVariables(self):
        return self.variables

    def getVariablesAsText(self):
        variablesText = self.getAsText(self.variables)
        return variablesText

    def addVariables(self, variables):
        self.variables += '\n' + variables

    def setVariables(self, variables):
        self.variables = variables

    def getFunctions(self):
        return self.functions

    def getFunctionsAsText(self):
        textFunctions = self.getAsText(self.functions)
        return textFunctions

    def getAsText(self, string):
        #TODO
        myStr = ''
        for i in range(len(string)):
            myStr += string[i]
            if i < len(string):
                myStr += ''
        return myStr

    def addFunctions(self, functions):
        self.functions += '\n\n' + functions

    def setFunctions(self, functions):
        self.functions = functions
