#!/usr/bin/env python
# -*- Python -*-

#
# \file CORBA_SeqUtil.py
# \brief test for CORBA sequence utility template functions
# \date $Date: 2007/09/03$
# \author Shinji Kurihara
#
# Copyright (C) 2006
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys
sys.path.insert(1,"../")

import unittest
import OpenRTM_aist

import CORBA_SeqUtil

class FunctorAdd:
  def __init__(self, lists):
    self._list = lists

  def __call__(self, obj):
    self._list.append(obj)

class Property:
  def __init__(self,id_):
    self.id =id_
  
class FunctorFind:
  def __init__(self, id):
    self._id = id

  def __call__(self, prof):
    return self._id == prof.id
    


class TestCORBA_SeqUtil(unittest.TestCase):
  def setUp(self):
    pass
  
  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    return

  def test_for_each(self):
    list_ = [0,1,2]
    func_ = FunctorAdd(list_)
    add_ = [3,4,5,6,7]
    CORBA_SeqUtil.for_each(add_,func_)
    self.assertEqual(list_, [0,1,2,3,4,5,6,7])
    

  def test_find(self):
    prof = [Property(100),Property(200),Property(300)]
    index = CORBA_SeqUtil.find(prof,FunctorFind(300))
    self.assertEqual(index, 2)

    index = CORBA_SeqUtil.find(prof,FunctorFind(3000))
    self.assertEqual(index, -1)

  
  def test_push_back(self):
    list_ = [0,1,2,3]
    CORBA_SeqUtil.push_back(list_,100)
    self.assertEqual(list_, [0,1,2,3,100])

  
  def test_push_back_list(self):
    list1 = [0,1,2]
    list2 = [3,4,5,6,7]
    CORBA_SeqUtil.push_back_list(list1,list2)
    self.assertEqual(list1, [0,1,2,3,4,5,6,7])


  def test_insert(self):
    list_ = [0,1,2]
    CORBA_SeqUtil.insert(list_, "INS", 1)
    self.assertEqual(list_, [0,"INS",1,2])

    CORBA_SeqUtil.insert(list_, 10, 100)
    self.assertEqual(list_, [0,"INS",1,2,10])


  def test_front(self):
    list_ = [3,4,5,6]
    self.assertEqual(CORBA_SeqUtil.front(list_),3)
    

  def test_back(self):
    list_ = [3,4,5,6]
    self.assertEqual(CORBA_SeqUtil.back(list_),6)


  def test_erase(self):
    list_ = [9,8,7,6]
    CORBA_SeqUtil.erase(list_, 10)
    self.assertEqual(list_, [9,8,7,6])

    CORBA_SeqUtil.erase(list_, 1)
    self.assertEqual(list_, [9,7,6])


  def test_erase_if(self):
    prof = [Property(100),Property(200),Property(300)]
    CORBA_SeqUtil.erase_if(prof,FunctorFind(999))
    self.assertEqual(len(prof), 3)
    
    CORBA_SeqUtil.erase_if(prof,FunctorFind(200))
    self.assertEqual(len(prof), 2)
    

  def test_clear(self):
    list_ = [0,1,2,3,4,5]
    CORBA_SeqUtil.clear(list_)
    self.assertEqual(list_,[])


  def test_refToVstring(self):
    pass


############### test #################
if __name__ == '__main__':
        unittest.main()
