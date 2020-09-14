#!/usr/bin/env python
# -*- Python -*-

hoge_spec = ["implementation_id", "hoge",
             "type_name",         "hoge",
             "description",       "hoge component",
             "version",           "1.0",
             "vendor",            "Shinji Kurihara",
             "category",          "example",
             "activity_type",     "DataFlowComponent",
             "max_instance",      "10",
             "language",          "Python",
             "lang_type",         "script",
             ""]

class HOGE:
    def __init__(self):
        pass

def echo(*args):
    print "hello HOGE"

        

"""
import sys
argv = sys.argv[1:]
for x in argv:
	try:
		f = open(x)
		print"---",x,"---"
		for i in range(15):
			print i+1,f.readline(),
		print "---",x,"---\n"
		f.close()
	except IOError:
		print "Can't find file:", x		

"""

		
