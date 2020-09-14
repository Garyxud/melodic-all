#!/usr/bin/env python
# -*- python -*-
# -*- condig shift_jis -*-
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
#  $Id$
# 

import os
import re
import time
import yat
import StringIO
class gen_base:
	
	def check_overwrite(self, fname, wmode="w"):
		"""
		Check file exist or not.
		"""
		msg = " already exists. Overwrite or merge? (y/n/m)"
		if (os.access(fname, os.F_OK)):
			ans = raw_input("\"" + fname + "\"" + msg)
			if (ans == "y" or ans == "Y"):
				return file(fname, wmode), None
			elif (ans == "m" or ans == "M"):
				f = file(fname, "r")
				lines = []
				for l in f.readlines():
					lines.append(l.rstrip())
				f.close()
				oldfname = fname + ".old." + time.strftime("%y%m%d%H%M%S")
				os.rename(fname, oldfname)
				return file(fname, wmode), lines
			else:
				return None, None
		else:
			return file(fname, wmode), None
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
			t = yat.Template(tags[key])
			text=t.generate(self.data)
			tags[key] = text
		return

	def gen(self, fname, temp_txt, data, tags):
		f, lines = self.check_overwrite(fname)
		if not f:      # overwrite: No
			return

		if not lines:  # overwrite: Yes
			t = yat.Template(temp_txt)
			taged_txt = t.generate(self.data)
		else:          # overwrite: Merge mode
			taged_txt = lines

		# replace tags
		gen_txt = self.replace_tags(taged_txt.split("\n"), tags)
		f.write(gen_txt)
		f.close()
		print "  File \"" + fname + "\"" " was generated."
		return
