#!/usr/bin/env python
# -*- coding: euc-jp -*-

##
# @file GlobalFactory.py
# @brief generic Factory template class
# @date $Date$
# @author Noriaki Ando <n-ando@aist.go.jp> and Shinji Kurihara
#
# Copyright (C) 2009
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#


import OpenRTM_aist


class Factory:

  FACTORY_OK     = 0
  FACTORY_ERROR  = 1
  ALREADY_EXISTS = 2
  NOT_FOUND      = 3
  INVALID_ARG    = 4
  UNKNOWN_ERROR  = 5


  def __init__(self):
    self._creators = {}


  ## bool hasFactory(const Identifier& id)
  def hasFactory(self, id):
    if not self._creators.has_key(id):
      return False
    return True


  ## std::vector<Identifier> getIdentifiers()
  def getIdentifiers(self):
    idlist = []

    for id in self._creators.keys():
      idlist.append(id)
    idlist.sort()
    return idlist


  ## ReturnCode addFactory(const Identifier& id,
  ##                       Creator creator,
  ##                       Destructor destructor)
  def addFactory(self, id, creator, destructor):
    if not creator:
      return self.INVALID_ARG

    if self._creators.has_key(id):
      return self.ALREADY_EXISTS
    
    self._creators[id] = creator
    return self.FACTORY_OK


  ## ReturnCode removeFactory(const Identifier& id)
  def removeFactory(self, id):
    if not self._creators.has_key(id):
      return self.NOT_FOUND

    del self._creators[id]
    return self.FACTORY_OK


  ## AbstractClass* createObject(const Identifier& id)
  def createObject(self, id):
    if not self._creators.has_key(id):
      print "Factory.createObject return None id: ", id
      return None
    return self._creators[id]()


  ## void deleteObject(const Identifier& id, AbstractClass*& obj)
  def deleteObject(self, obj, id=None):
    if id:
      if not self._creators.has_key(id):
        return

    del obj
    return

    
gfactory = None

class GlobalFactory(Factory):
  def __init__(self):
    Factory.__init__(self)
    pass


  def instance():
    global gfactory
    
    if gfactory is None:
      gfactory = GlobalFactory()

    return gfactory

  instance = staticmethod(instance)


  def __del__(self):
    pass
