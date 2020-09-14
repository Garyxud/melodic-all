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
import rospkg
import os
from xml.dom import minidom
from PyQt5.QtCore import QCoreApplication, QSettings

# we keep module variables not to calculate these lists every time gui needs them
rospackmodule = None
rosTypes = None
rosPackages = None


def setupSettings():
    QCoreApplication.setOrganizationName('JdeRobot')
    QCoreApplication.setOrganizationDomain('jderobot.org')
    QCoreApplication.setApplicationName('VisualStates')
    return QSettings()


def readWorkspaces():
    workspaces = []
    settings = setupSettings()
    count = settings.beginReadArray('workspaces')
    for i in range(count):
        settings.setArrayIndex(i)
        workspaces.append(settings.value('dir'))
    settings.endArray()
    return workspaces


def writeWorkspaces(workspaces):
    global rosPackages
    global rosTypes
    settings = setupSettings()
    settings.beginWriteArray('workspaces')
    for i in range(len(workspaces)):
        settings.setArrayIndex(i)
        settings.setValue('dir', workspaces[i])
    settings.endArray()
    rosTypes = None
    rosPackages = None


def getRosMessageTypes(rosDir=None):
    if rosDir is None:
        if 'ROS_DISTRO' in os.environ:
            rosDir = '/opt/ros/' + os.environ['ROS_DISTRO']
        else:
            rosDir = '/opt/ros/kinetic'

    messageDir = rosDir + '/include'
    if os.path.exists(messageDir):
        allContents = os.listdir(messageDir)
        messages = []
        for entry in allContents:
            if os.path.isdir(messageDir + '/' + entry):
                if entry.find('_msgs') >= 0:
                    messages.append(entry)

        types = []
        for msg in messages:
            typeDir = msg
            for entry in os.listdir(messageDir + '/' + msg):
                if os.path.isfile(messageDir + '/' + msg + '/' + entry):
                    if entry.find('.h') >= 0 and entry[0].isupper():
                        type = {}
                        type['typeDir'] = typeDir
                        type['type'] = entry[:entry.find('.h')]
                        types.append(type)
        return types
    else:
        return False


def getPackagePath():
    global rospackmodule
    if rospackmodule is None:
        rospackmodule = rospkg.RosPack()
    return rospackmodule.get_path('visualstates')


def getAllPackages():
    global rospackmodule
    global rosPackages
    if rosPackages is None:
        if rospackmodule is None:
            rospackmodule = rospkg.RosPack()
        rosPackages = rospackmodule.list()

        workspaces = readWorkspaces()
        for dir in workspaces:
            packages = getPackages(dir)
            rosPackages += packages

    return rosPackages


def getAllTypes():
    global rosTypes
    if rosTypes is None:
        messageTypes = getRosMessageTypes()

        workspaces = readWorkspaces()
        for dir in workspaces:
            messageTypes += getRosMessageTypes(dir + '/devel')

        concatTypes = []
        for type in messageTypes:
            concatType = type['typeDir'] + '/' + type['type']
            concatTypes.append(concatType)

        rosTypes = sorted(concatTypes)

    return rosTypes


def getPackages(workspaceDir):
    packageNames = []
    packageFiles = []
    for root, dirs, files in os.walk(workspaceDir + '/src'):
        visibleDirs = []
        for dir in dirs:
            if not dir.startswith('.'):
                visibleDirs.append(dir)
        dirs[:] = visibleDirs
        for file in files:
            if file == 'package.xml':
                packageFiles.append(root + '/' + file)

    for pkgFile in packageFiles:
        doc = minidom.parse(pkgFile)
        nameElement = doc.getElementsByTagName('package')[0].getElementsByTagName('name')[0]
        if len(nameElement.childNodes) > 0:
            packageNames.append(nameElement.childNodes[0].nodeValue)
    return packageNames


if __name__ == '__main__':
    # print('packagePath:' + getPackagePath())
    # print('packages:' + str(getAllPackages()))
    print(getPackages('/media/okan/depo/jderobot/catkin_ws'))


