#!/usr/bin/env python
# -*- Python -*-

#
# \file test_StringUtil.py
# \brief test for String operation utility
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

import unittest

from StringUtil import *
import OpenRTM_aist

class TestStringUtil(unittest.TestCase):

  def setUp(self):
    pass

  def tearDown(self):
    OpenRTM_aist.Manager.instance().shutdownManager()
    pass
  
  def test_isEscaped(self):
    self.assertEqual(isEscaped("test\\\\test",5), True, "Result failed.")

    
  def test_escape(self):
    self.assertEqual(escape("test\ttest"), "test\\ttest", "Result failed.")
    self.assertEqual(escape("test\ntest"), "test\\ntest", "Result failed.")
    self.assertEqual(escape("test\ftest"), "test\\ftest", "Result failed.")
    self.assertEqual(escape("test\rtest"), "test\\rtest", "Result failed.")


  def test_unescape(self):
    self.assertEqual(unescape("test\\ttest"), "test\ttest", "Result failed.")
    self.assertEqual(unescape("test\\ntest"), "test\ntest", "Result failed.")
    self.assertEqual(unescape("test\\ftest"), "test\ftest", "Result failed.")
    self.assertEqual(unescape("test\\rtest"), "test\rtest", "Result failed.")


  def test_eraseBlank(self):
    _str=[" test"]
    eraseBlank(_str)
    self.assertEqual(_str[0], "test", "Result failed.")

    _str=["   test"]
    eraseBlank(_str)
    self.assertEqual(_str[0], "test", "Result failed.")

    _str=["\t\ttest"]
    eraseBlank(_str)
    self.assertEqual(_str[0], "test", "Result failed.")

    _str=[""]
    eraseBlank(_str)
    self.assertEqual(_str[0], "", "Result failed.")

    _str=["\t\n test"]
    eraseBlank(_str)
    self.assertEqual(_str[0], "\ntest")

    _str=["\t\\t test"]
    eraseBlank(_str)
    self.assertEqual(_str[0], "\\ttest", "Result failed.")
    
  def test_eraseHeadBlank(self):
    _str=[" test"]
    eraseHeadBlank(_str)
    self.assertEqual(_str[0], "test", "Result failed.")

    _str=["   test"]
    eraseHeadBlank(_str)
    self.assertEqual(_str[0], "test", "Result failed.")

    _str=["\t\ttest"]
    eraseHeadBlank(_str)
    self.assertEqual(_str[0], "test", "Result failed.")

    _str=[""]
    eraseHeadBlank(_str)
    self.assertEqual(_str[0], "", "Result failed.")

    _str=["\t\n test"]
    eraseHeadBlank(_str)
    self.assertEqual(_str[0], "\n test", "Result failed.")

    _str=["\t\\t test"]
    eraseHeadBlank(_str)
    self.assertEqual(_str[0], "\\t test", "Result failed.")


    # failed case
    # _str=["\\t\\ttest"]
    # eraseHeadBlank(_str)
    # self.assertEqual(_str[0], "test", "Result failed.")


  def test_eraseTailBlank(self):
    _str=["test "]
    eraseTailBlank(_str)
    self.assertEqual(_str[0], "test", "Result failed.")

    _str=["test   "]
    eraseTailBlank(_str)
    self.assertEqual(_str[0], "test", "Result failed.")

    _str=["test\t\t"]
    eraseTailBlank(_str)
    self.assertEqual(_str[0], "test", "Result failed.")

    _str=["test\t\n\t"]
    eraseTailBlank(_str)
    self.assertEqual(_str[0], "test\t\n", "Result failed.")

    _str=["test\\t"]
    eraseTailBlank(_str)
    self.assertEqual(_str[0], "test\\t", "Result failed.")

    # failed case
    # _str=["test\\t\\t"]
    # eraseTailBlank(_str)
    # self.assertEqual(_str[0], "test", "Result failed.")

  def test_normalize(self):
    _str = [" NORMALIZE "]
    normalize(_str)
    self.assertEqual(_str[0],"normalize")

    _str = [" \t  \tNORmALIZE "]
    normalize(_str)
    self.assertEqual(_str[0],"normalize")

    _str = [" nORMALIZE\t \t\t\t"]
    normalize(_str)
    self.assertEqual(_str[0],"normalize")


  def test_replaceString(self):
    _str= ["replace"]
    replaceString(_str,"l", "r")
    self.assertEqual(_str[0], "reprace", "Result failed.")

    _str= ["replace"]
    replaceString(_str,"re", "")
    self.assertEqual(_str[0], "place", "Result failed.")

    _str= ["replace"]
    replaceString(_str,"e", "a")
    self.assertEqual(_str[0], "raplaca", "Result failed.")


  def test_split(self):
    _str = split("test0,test1,test2", ",")
    self.assertEqual(_str[0], "test0", "Result failed.")
    self.assertEqual(_str[1], "test1", "Result failed.")
    self.assertEqual(_str[2], "test2", "Result failed.")

    _str = split("test0.test1.test2", ".")
    self.assertEqual(_str[0], "test0", "Result failed.")
    self.assertEqual(_str[1], "test1", "Result failed.")
    self.assertEqual(_str[2], "test2", "Result failed.")

    _str = split("test0/test1/test2", "/")
    self.assertEqual(_str[0], "test0", "Result failed.")
    self.assertEqual(_str[1], "test1", "Result failed.")
    self.assertEqual(_str[2], "test2", "Result failed.")

    _str = split("test0 test1 test2", " ")
    self.assertEqual(_str[0], "test0", "Result failed.")
    self.assertEqual(_str[1], "test1", "Result failed.")
    self.assertEqual(_str[2], "test2", "Result failed.")


  def test_toBool(self):
    ret = toBool("yes", "yes", "no", True)
    self.assertEqual(ret, True, "Result failed.")

    ret = toBool("no", "yes", "no", True)
    self.assertEqual(ret, False, "Result failed.")

    ret = toBool("Yes", "YES", "NO", True)
    self.assertEqual(ret, True, "Result failed.")

    ret = toBool("No", "YES", "NO", True)
    self.assertEqual(ret, False, "Result failed.")

  def test_includes(self):
    self.assertEqual(includes(["abc","abcde","ABC"],"abc"),True)
    self.assertEqual(includes(["abc","abcde","ABC"],"a"),False)
    self.assertEqual(includes(["abc","abcde","ABC"],"ABC"),True)
    self.assertEqual(includes("abc,abcde,ABC","ABC"),True)
    self.assertEqual(includes("abc,abcde,ABC","AbC",False),False)
    self.assertEqual(includes(["abc","abcde","AbC"],"ABC"),True)
    self.assertEqual(includes(["abc","abcde","AbC"],"ABC",False),False)


  def test_isAbsolutePath(self):
    self.assertEqual(isAbsolutePath("/usr/loca/bin"), True, "Result failed.")
    self.assertEqual(isAbsolutePath("c:\\"), True, "Result failed.")
    self.assertEqual(isAbsolutePath("\\\\localhost"), True, "Result failed.")
    # failed case
    # self.assertEqual(isAbsolutePath("\\localhost"), True, "Result failed.")


  def test_isURL(self):
    self.assertEqual(isURL("http://www.google.co.jp"), True, "Result failed.")

    # failed case
    # self.assertEqual(isURL("www.google.co.jp"), True, "Result failed.")
    # self.assertEqual(isURL("http:://www.google.co.jp"), True, "Result failed.")


  def test_otos(self):
    self.assertEqual(otos(123), "123", "Result failed.")
    self.assertEqual(otos("123"), "123", "Result failed.")
    self.assertEqual(otos(123456789123456789), "123456789123456789", "Result failed.")
    self.assertEqual(otos(0.123), "0.123", "Result failed.")

  
  def test_stringTo(self):
    int_ = [0]
    long_ = [0]
    float_ = [0.0]
    list_ = [[0.0,0.0,0.0,0.0]]
    str_ = [""]
    stringTo(int_,"123")
    stringTo(long_,"123")
    stringTo(float_,"0.123")
    stringTo(list_,"0,1.1,2.2,3.3")
    stringTo(str_,"hoge")
    print list_[0]
    self.assertEqual(int_[0], 123, "Result failed.")
    self.assertEqual(long_[0], 123, "Result failed.")
    self.assertEqual(float_[0], 0.123, "Result failed.")
    self.assertEqual(list_[0], [0,1.1,2.2,3.3], "Result failed.")
    self.assertEqual(str_[0], "hoge", "Result failed.")

    # failed case
    # self.assertEqual(stringTo("int",0.123), 0.123, "Result failed.")


  def test_unique_sv(self):
    sv = ["test0","test1","test0","test2","test0"]
    sv_ret = unique_sv(sv)
    self.assertEqual(sv_ret[0], "test0", "Result failed.")
    self.assertEqual(sv_ret[1], "test1", "Result failed.")
    self.assertEqual(sv_ret[2], "test2", "Result failed.")
    self.assertEqual(len(sv_ret), 3, "Result failed.")

    # failed case:  len(sv_ret) is 4
    # 
    # sv = ["test0","test1","test0","test2","TEST0"]
    # sv_ret = unique_sv(sv)
    # self.assertEqual(len(sv_ret), 3, "Result failed.")

    
  def test_flatten(self):
    sv = ["test0","test1","test2"]
    sv_ret = flatten(sv)
    self.assertEqual(sv_ret, "test0, test1, test2","Result failed.")

    # failed case:  Space is included.
    # self.assertEqual(sv_ret, "test0,test1,test2","Result failed.")


  def test_toArgv(self):
    pass

  

############### test #################
if __name__ == '__main__':
        unittest.main()
