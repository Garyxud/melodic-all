#!/usr/bin/env python
#
# @brief Visual Studio solution generator
# @date $Date: 2008-03-06 06:46:37 $
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2008-2011
#     Noriaki Ando, Tsuyoto Katami
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

import sys
import re
import yaml
import yat

#------------------------------------------------------------
# Generic vcproj template
#------------------------------------------------------------
vcversions = {"VC8": {"sln": "9.00", "vc": "2005"},
              "VC9": {"sln": "10.00", "vc": "2008"},
              "VC10": {"sln": "11.00", "vc": "2010"},
              "VC11": {"sln": "12.00", "vc": "2012"},
              "VC12": {"sln": "13.00", "vc": "2013"},
              "VC14": {"sln": "14.00", "vc": "2015"},
              }
sln_template = """Microsoft Visual Studio Solution File, Format Version %s
# Visual Studio %s
[for proj in Projects]
Project("{[SolutionGUID]}") = "[proj.Name]", "[proj.FileName]", "{[proj.GUID]}"
	ProjectSection(ProjectDependencies) = postProject
[if-any proj.Depend]
[for dep in proj.Depend]
		{[dep]} = {[dep]}
[endfor]
[endif]
	EndProjectSection
EndProject
[endfor]
Global
	GlobalSection(SolutionConfigurationPlatforms) = preSolution
[for conf in Configurations]
		[conf] = [conf]

[endfor]
	EndGlobalSection
	GlobalSection(ProjectConfigurationPlatforms) = postSolution
[for proj in Projects]
[for conf in Configurations]
		{[proj.GUID]}.[conf].ActiveCfg = [conf]

		{[proj.GUID]}.[conf].Build.0   = [conf]

[endfor]
[endfor]
	EndGlobalSection
	GlobalSection(SolutionProperties) = preSolution
		HideSolutionNode = FALSE
	EndGlobalSection
EndGlobal
"""


sln_yaml = """
SolutionGUID: 8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942
Configurations:
  - Release|Win32
  - Release|x64
  - Debug|Win32
  - Debug|x64
"""


def usage():
    print """
Usage:
  slntool.py --dep dep_file [--outfile outfile] vcproj_files...

Options:
    --vcversion: Visual C++'s version [VC8|VC9|VC10|VC11|VC12|VC14]
    --dep: dependency file
    --out or --output: output file name

Example:
 * make solution from all of found vcprojs
  slntool.py --dep dep_file --outfile MyApp.sln \
    `find ./ --name '*.vcproj'`
 * output solution to stdout
  slntool.py --dep dep_file *.vcproj

Depfile examples: The rule is similar to Makefile's dependency rule
  All of entries should be Projectname.
  [target Projectname]: [dependent projectsname...]

In the following App.dep file, "App" dpends on "Lib1" and "Lib2".
"Lib2" depends on "Lib1", "Lib1" has no dependency.
For this solution, "App", "Lib1" and "Lib2"'s project files are necessary.

-- App.dep --
App: Lib1 Lib2
Lib2: Lib1
-------------

"""


def get_projinfo(fname,vcversion="VC8"):
    name = None
    guid = None
    regexs = {"VC8": {"guid":'^.*?ProjectGUID=\"{(.*)}\"',"name":'^.*?Name=\"(.*)\"'}, 
              "VC9": {"guid":'^.*?ProjectGUID=\"{(.*)}\"',"name":'^.*?Name=\"(.*)\"'}, 
              "VC10": {"guid":'^.*?<ProjectGuid>{(.*)}</ProjectGuid>',"name":'^.*<ProjectName>(.*)</ProjectName>'},
              "VC11": {"guid":'^.*?<ProjectGuid>{(.*)}</ProjectGuid>',"name":'^.*<ProjectName>(.*)</ProjectName>'},
              "VC12": {"guid":'^.*?<ProjectGuid>{(.*)}</ProjectGuid>',"name":'^.*<ProjectName>(.*)</ProjectName>'},
              "VC14": {"guid":'^.*?<ProjectGuid>{(.*)}</ProjectGuid>',"name":'^.*<ProjectName>(.*)</ProjectName>'},
             }
    re_guid = re.compile(regexs[vcversion]["guid"])
    re_name = re.compile(regexs[vcversion]["name"])
    fd = open(fname, "r")
    pj = fd.readlines()
    for t in pj:
        n = re_name.match(t)
        g = re_guid.match(t)

        if name == None and n:
            name = n.group(1)
        if guid == None and g:
            guid = g.group(1)

        if name and guid:
            break
    fd.close()
    return {"Name": name, "GUID": guid, "FileName": fname}

def get_dependencies(fname):
    if fname == None: return {}
    depdic = {}
    fd = open(fname, "r")
    for l in fd.readlines():
        (key, val) = l.split(":")
        vals = val.split()
        depdic[key] = vals
    return depdic

def parse_args(argv):
    argc = len(argv)
    depfile = None
    outfile = None
    vcversion = "VC8"
    flist = []
    i = 0
    while i < argc:
        opt = argv[i]
        if opt == "--dep":
            i += 1
            if i < argc: depfile = argv[i]
            else: raise InvalidOption(opt + " needs value")
        elif opt == "--output" or opt == "--out":
            i += 1
            if i < argc: outfile = argv[i]
            else: raise InvalidOption(opt + " needs value")
        elif opt == "--vcversion":
            i += 1
            if i < argc: vcversion = argv[i]
            else: raise InvalidOption(opt + " needs value")
            if not vcversions.has_key(vcversion):
                allowedvers = vcversions.keys().__repr__()
                raise InvalidOption("allowed vcversions are " + allowedvers)
        else:
            while i < argc and argv[i][:2] != "--":
                flist.append(argv[i])
                i += 1
        i += 1
    return (vcversion, depfile, outfile, flist)

def get_slnyaml(depfile, projfiles, vcversion="VC8"):
    depdict = get_dependencies(depfile)
    projs = []
    projlist = """Projects:
"""
    for f in projfiles:
        pj = get_projinfo(f, vcversion)
        if depdict.has_key(pj["Name"]):
            pj["Depend"] = depdict[pj["Name"]]
        projs.append(pj)
    def depsort(d0, d1):
        """
        d0  < d1: return -1 
        d0 == d1: return  0 
        d0  > d1: return  1 
        """
        d0_depends = d0.has_key("Depend")
        d1_depends = d1.has_key("Depend")
        if not d0_depends and not d1_depends:
            # both d0, d1 has no dependency 
            return 0

        if not d0_depends and d1_depends:
            # only "d1" has dependency: d0 < d1
            return -1 

        if d0_depends and not d1_depends:
            # only "d0" has dependency: d1 < d0
            return 1 

        # d0 and d1 has dependency
        d0_in_dep = depdict.has_key(d0["Name"])
        d1_in_dep = depdict.has_key(d1["Name"])
        if not d0_in_dep and not d1_in_dep:
            return 0
        if not d0_in_dep and d1_in_dep:
            return -1
        if d0_in_dep and not d1_in_dep:
            return 1
        
        # both d0 and d1 have several dependency
        if depdict[d0["Name"]].count(d1["Name"]) > 0:
            return 1
        if depdict[d1["Name"]].count(d0["Name"]) > 0:
            return -1
        return 0

    projs.sort(depsort)
    for pj in projs:
        list = """  - Name: %s
    FileName: %s
    GUID: &%s %s
    Depend:
""" % (pj["Name"], pj["FileName"], pj["Name"], pj["GUID"])
        if pj.has_key("Depend"):
            for dep in pj["Depend"]:
                dep = """      - *%s
""" % (dep)
                list += dep
        projlist += list
    yaml_text = sln_yaml + projlist
    return yaml_text

def gen_solution(version, yaml_text):

    dict = yaml.load(yaml_text)
    t = yat.Template(sln_template 
                     % (vcversions[version]["sln"],
                        vcversions[version]["vc"]))
    return t.generate(dict).replace("\r\n", "\n").replace("\n", "\r\n")


class SlnToolException:
    pass

class InvalidOption(SlnToolException):
    def __init__(self, msg):
        self.msg = "Error: InvalidOption:\n    "
        self.msg += msg

#------------------------------------------------------------
# main function
#------------------------------------------------------------
def main(argv):
    if len(argv) == 0:
        usage()
        sys.exit(-1)
    try:
        res = parse_args(argv)
    except SlnToolException, e:
        print "\n" + e.msg + "\n"
        usage()
        sys.exit(-1)

    version = res[0]
    depfile = res[1]
    outfile = res[2]
    flist   = res[3]
    #sln_text = gen_solution(version, get_slnyaml(depfile, flist))
    sln_text = gen_solution(version, get_slnyaml(depfile, flist, version))

    if outfile == None:
        fd = sys.stdout
    else:
        fd = open(outfile, "wb")

    fd.write(sln_text)

#------------------------------------------------------------
# tests
#------------------------------------------------------------
def test_getprojinfo():
    for f in sys.argv[1:]:
        print get_projinfo(f)

def test_getdep():
    print get_dependencies(sys.argv[1])

def test_getslnyaml():
    print gen_solution(get_slnyaml("dep.yaml", sys.argv[1:]))

#------------------------------------------------------------
# entry point
#------------------------------------------------------------
if __name__ == "__main__":
    main(sys.argv[1:])
    
