#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  @file RtmDtdValidator.py
#  @brief XML validator based on specified DTD file
#  @date $Date: 2005-05-12 09:06:19 $
#  @author K.Kitagaki
# 
#  Copyright (C) 2004-2005
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id$
#

# dtdValidator.py -- xml with DTD Validator
# (C)K.Kitagaki 2004.8.26
#
# # usage
# import dtdValidator
# validator = dtdValidator.DtdValidator()
# errors = validator.parse("filename.xml")
#
# for (filename, line, col, msg) in errors:
#      print filename, line, col, msg
#
# -xmlをDTDに基き検証する。

from _xmlplus.parsers.xmlproc import xmlval, xmlapp, errors

# XML validator
class DtdValidator:
    def __init__(self):
        self.parser = xmlval.XMLValidator()
        self.errors = ErrorRecorder(self.parser)

    def parse(self, fileName):
        sysid = fileName
#        print sysid
        self.parser.reset()
        self.errors.reset()
        self.parser.set_error_handler(self.errors)
        self.parser.parse_resource(sysid)
##         for (sysid, line, col, msg) in self.errors.errors:
##             print sysid, line, col, msg
        return self.errors.errors

# XML ErrorHandler
class ErrorRecorder(xmlapp. ErrorHandler):
    def __init__(self, locator, warnings=1):
        xmlapp.ErrorHandler.__init__(self, locator)
        self.show_warnings=warnings
        self.reset()

    def warning(self,msg):
        if self.show_warnings:
            self.__add_error(msg)

    def error(self,msg):
        self.__add_error(msg)

    def fatal(self,msg):
        self.__add_error(msg)

    def reset(self):
        self.errors=[]

    def __add_error(self,msg):
        self.errors.append((self.locator.get_current_sysid(),
                            self.locator.get_line(),
                            self.locator.get_column(),
                            msg))
