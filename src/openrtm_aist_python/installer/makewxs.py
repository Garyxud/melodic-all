#!/usr/bin/env python
#
# @brief WiX wxsd file generator
# @date $Date: 2008-02-26 13:58:13 $
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id: makewxs.py 1686 2010-01-18 13:36:16Z n-ando $
#

import yaml
import yat
import sys
import uuid
import getopt
import os

def replace_uuid(text):
    token0 = text.split("__GUID__")
    text0 = token0[0]
    for i in range(1, len(token0)):
        u = str(uuid.uuid1()).upper()
        text0 += u + token0[i]

    token1 = text0.split("__UUID")
    text1 = token1[0]
    for i in range(1, len(token1)):
        u = "_" + str(uuid.uuid1()).replace("-", "")
        text1 += u + token1[i]
    return text1

class file_list:
    def __init__(self, comp, path, files):
        self.comp = comp
        self.path = self.check_path(path)
        self.files = self.to_basename(files)
        self.shortnames = {}
        self.shortext = {}
        self.output = ""
        self.head = """%s:
  GUID: %s
  Files:
"""
        self.fitem = """    - Id: %s
      ShortName: %s
      Name: %s
      Source: %s
"""
        self.count = -1

    def check_path(self, path):
        if path[-1] != "\\" or path[-1] != "/":
            return path + "\\"

    def to_basename(self, files):
        out = []
        for f in files:
            out.append(os.path.basename(f))
        return out

    def to_shortname(self, fname):
        try:
            name, ext = fname.rsplit(".", 1)
        except:
            name = fname
            ext  = ""
        if name != None and len(name) > 8:
            short_name = name[:5] + self.sn_num(name[:5])
        else:
            short_name = name
        if ext != None and len(ext) > 3:
            short_ext = ext[:3]
        else:
            short_ext = ext
        if short_ext == "":
            return short_name
        return short_name + "." + short_ext

    def id(self):
        self.count += 1
        return self.comp + '%04d' % (self.count)
        
    def sn_num(self, name):
        if self.shortnames.has_key(name):
            self.shortnames[name] += 1
        else:
            self.shortnames[name] = 0
        return "%03d" % (self.shortnames[name])

    def se_num(self, ext):
        if self.shortext.has_key(ext):
            self.shortext[ext] += 1
        else:
            self.shortext[ext] = 0
        return "%01d" % (self.shortext[ext])

    def write(self, text):
        self.output += text

    def escape(self, text):
        return text.replace("\\", "\\\\")

    def yaml(self):
        self.write(self.head % (self.comp, str(uuid.uuid1()).upper()))
        for fname in self.files:
            self.write(self.fitem % (self.escape(self.id()),
                                     self.escape(self.to_shortname(fname)),
                                     self.escape(fname),
                                     self.escape(self.path + fname)))
        return self.output


class make_wxs:
    def __init__(self, outfile, infile, yaml_files):
        self.outfile = outfile
        self.template = self.load_template(infile)
        self.dict = self.load_yaml(yaml_files)

    def load_template(self, template):
        fd = open(template, 'r')
        t = yat.Template(fd.read(), "{% ", " %}")
        fd.close()
        return t

    def load_yaml(self, yaml_files):
        yaml_text = ""
        for f in yaml_files:
            fd = open(f, "r")
            yaml_text += replace_uuid(fd.read())
            fd.close()
        return yaml.load(yaml_text)

    def generate(self):
        of = open(self.outfile, 'w')
        of.write(self.template.generate(self.dict))
        of.close()


def usage():
    print """makewxs.py cmd options
commands:
  flist: make file list to be included wxs file
  wxs  : make wxs file from a input template file and yaml files 
examples:
  makewxs.py flist -c ComponentName -p Path -o OutputFilename file_names...
  makewxs.py wxs -o Output.wxs -i InputTempalte input_yaml_files...
"""


def main(argv):
    if len(argv) == 0:
        usage()
        sys.exit(-1)

    cmd = argv[0]
    out = None
    if cmd == "flist":
        opts, args = getopt.getopt(argv[1:], "c:p:o:", [])
        if opts == None:
            usage()
            sys.exit(-1)
            return
        for o, a in opts:
            if o in ("-c"):
                comp = a
            if o in ("-p"):
                path = a
            if o in ("-o"):
                out = a
        fl = file_list(comp, path, args)
        if out == None:
            f = sys.stdout
        else:
            f = open(out, "w")
        f.write(fl.yaml())
        f.close()
    elif cmd == "wxs":
        opts, args = getopt.getopt(argv[1:], "o:i:", [])
        if opts == None:
            usage()
            sys.exit(-1)
            return
        for o, a in opts:
            if o in ("-o"):
                outfile = a
            if o in ("-i"):
                infile = a
        wxs = make_wxs(outfile, infile, args)
        wxs.generate()
    elif cmd == "":
        pass
    else:
        pass
        

if __name__ == "__main__":
    main(sys.argv[1:])
