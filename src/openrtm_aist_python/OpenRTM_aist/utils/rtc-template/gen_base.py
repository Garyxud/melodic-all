#!/usr/bin/env python
# -*- python -*-
#
#  @file gen_base.py
#  @brief rtc-template source code generator base class
#  @date $Date: 2007/01/11 07:43:16 $
#  @author Noriaki Ando <n-ando@aist.go.jp>
# 
#  Copyright (C) 2005
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id: gen_base.py,v 1.4 2007/01/11 07:43:16 n-ando Exp $
# 

#
#  $Log: gen_base.py,v $
#  Revision 1.4  2007/01/11 07:43:16  n-ando
#  A trivial fix.
#
#  Revision 1.3  2005/09/08 09:24:06  n-ando
#  - A bug fix for merge function.
#
#  Revision 1.2  2005/09/06 14:37:29  n-ando
#  rtc-template's command options and data structure for ezt (Easy Template)
#  are changed for RTComponent's service features.
#  Now rtc-template can generate services' skeletons, stubs and
#  implementation files.
#  The implementation code generation uses omniidl's IDL parser.
#
#

import os
import re
import time
import ezt
import StringIO

class gen_base:
	
	def check_overwrite(self, fname):
		"""
		Check file exist or not.
		"""
		msg = " already exists. Overwrite or merge? (y/n/m)"
		if (os.access(fname, os.F_OK)):
			ans = raw_input("\"" + fname + "\"" + msg)
			if (ans == "y" or ans == "Y"):
				return file(fname, "w"), None
			elif (ans == "m" or ans == "M"):
				f = file(fname, "r")
				lines = []
				for l in f.readlines():
					lines.append(l.rstrip())
				f.close()
				oldfname = fname + ".old." + time.strftime("%y%m%d%H%M%S")
				os.rename(fname, oldfname)
				return file(fname, "w"), lines
			else:
				return None, None
		else:
			return file(fname, "w"), None
		return
	
	def replace_tags(self, lines, data):
		in_tag = False
		tag_name = ""
		ret_lines = ""
		
		for l in lines:
			m = re.search("<rtc-template block=\"(.*?)\">", l)
			if m:
				in_tag = True
				tag_name   = m.group(1)
				ret_lines += l + "\n"
				continue

			m = re.search("</rtc-template>", l)
			if m:
				in_tag = False
				if data.has_key(tag_name):
					ret_lines += data[tag_name] + "\n"
				ret_lines += l + "\n"
				tag_name = ""
				continue

			if in_tag != True:
				ret_lines += l + "\n"

		return ret_lines


	def gen_tags(self, tags):
		for key in tags.keys():
			s = StringIO.StringIO()
			t = ezt.Template(compress_whitespace = 0)
			t.parse(tags[key])
			t.generate(s, self.data)
			tags[key] = s.getvalue()
		return

	def gen(self, fname, temp_txt, data, tags):
		f, lines = self.check_overwrite(fname)
		if not f:      # overwrite: No
			return

		if not lines:  # overwrite: Yes
			s = StringIO.StringIO()
			t = ezt.Template(compress_whitespace = 0)
			t.parse(temp_txt)
			t.generate(s, data)
			taged_txt = s.getvalue().splitlines()
		else:          # overwrite: Merge mode
			taged_txt = lines

		# replace tags
		gen_txt = self.replace_tags(taged_txt, tags)
		f.write(gen_txt)
		f.close()
		print "  File \"" + fname + "\"" " was generated."
		return
		


if __name__ == "__main__":
	hoge = """
 protected:
  // <rtc-template block="inport_declar">
  // </rtc-template>

  // <rtc-template block="outport_declar">
  // </rtc-template>
"""
	data = {"inport_declar": "  hoge;\n  dara;\n  munya;",
			"outport_declar": "  1;\n  2;\n  3;"}
	g = gen_base()
	print g.replace_tags(hoge.splitlines(), data)
