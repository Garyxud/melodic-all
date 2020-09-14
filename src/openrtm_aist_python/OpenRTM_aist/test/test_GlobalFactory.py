#!/usr/bin/env python
# -*- coding: euc-jp -*-

#
# \file test_GlobalFactory.py
# \brief test for RTComponent factory class
# \date $Date: $
# \author Shinji Kurihara
#
# Copyright (C) 2003-2005
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#

import sys
sys.path.insert(1,"../")

import OpenRTM_aist
import unittest

from GlobalFactory import *

class Test:
	def __init__(self):
		pass

	def test(self):
		return True

class TestGlobalFactory(unittest.TestCase):

	def setUp(self):
		self.factory = GlobalFactory.instance()
		self.factory.addFactory("test",Test,OpenRTM_aist.Delete)
		return

	def tearDown(self):
		self.factory.removeFactory("test")
		return

	def test_isinstance(self):
		self.assertEqual(self.factory,GlobalFactory.instance())

	def test_hasFactory(self):
		# addFactoryにて登録したファクトリオブジェクトの問い合わせ。
		self.assertEqual(self.factory.hasFactory("test"),True)
		# addFactoryにて登録していないファクトリオブジェクトの問い合わせ。
		self.assertEqual(self.factory.hasFactory("testtest"),False)
		# addFactoryにて登録していないファクトリオブジェクトの問い合わせ(空文字)。
		self.assertEqual(self.factory.hasFactory(""),False)
		return

	def test_getIdentifiers(self):
		# ファクトリが登録済みの場合の問い合わせ。
		self.assertEqual(self.factory.getIdentifiers(),["test"])
		GlobalFactory.instance().addFactory("test2",Test,OpenRTM_aist.Delete)
		self.assertEqual(self.factory.getIdentifiers(),["test","test2"])
		# ファクトリが登録されていない場合の問い合わせ。
		self.factory.removeFactory("test")
		self.factory.removeFactory("test2")
		self.assertEqual(self.factory.getIdentifiers(),[])
		return
	
	def test_addFactory(self):
		# creatorを指定しない場合、INVALID_ARGが返されるか?
		self.assertEqual(GlobalFactory.instance().addFactory("test",None,OpenRTM_aist.Delete),
				 GlobalFactory.INVALID_ARG)

		# 既に登録済みのIDにてaddFactory()をコールした場合、ALREADY_EXISTSが返されるか?
		self.assertEqual(GlobalFactory.instance().addFactory("test",Test,OpenRTM_aist.Delete),
				 GlobalFactory.ALREADY_EXISTS)

		# idとcreatorを指定してaddFactory()をコールした場合、FACTORY_OKが返されるか?
		self.assertEqual(GlobalFactory.instance().addFactory("test1",Test,OpenRTM_aist.Delete),
				 GlobalFactory.FACTORY_OK)
		self.factory.removeFactory("test1")


		return

	def test_removeFactory(self):
		# 登録していないIDでコールした場合、NOT_FOUNDが返されるか?
		self.assertEqual(self.factory.removeFactory("testtest"),
				 GlobalFactory.NOT_FOUND)
				 
		# 登録済みのIDでコールした場合、FACTORY_OKが返されるか?
		self.assertEqual(self.factory.removeFactory("test"),
				 GlobalFactory.FACTORY_OK)

		# ファクトリが正しく削除されたか?
		self.assertEqual(self.factory.getIdentifiers(),[])
		return

	def test_createObject(self):
		# 登録していないIDでコールした場合、Noneが返されるか?
		self.assertEqual(self.factory.createObject("testtest"),
				 None)
		# 登録済みのIDでコールした場合、正しいオブジェクトが返されるか?
		self.assertEqual(self.factory.createObject("test").test(),True)
		return

	def test_deleteObject(self):
		# 登録していないIDでコールした場合
		self.factory.deleteObject(self.factory.createObject("test"),"testtest")
		# IDを指定しないでコールした場合
		self.factory.deleteObject(self.factory.createObject("test"))
		return

############### test #################
if __name__ == '__main__':
        unittest.main()
