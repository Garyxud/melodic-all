#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import lxml.etree
import lxml.builder
import os
import rospkg
import subprocess
import sys

class InvalidURDFException(Exception):
    pass

def printf(*args):
    print "[urdf_to_xacro.py] " + ' '.join(args)

class GazeboModelPathResolver(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.paths = []
        self.rospkgs = {}
        self.cached_path = {}
        self.load_path_from_env()
        self.load_path_from_plugin()

    def add_path(self, path):
        self.paths.insert(0, path)

    def add_ros_pkg(self, pkg, path):
        if pkg in self.rospkgs:
            self.rospkgs[pkg].insert(0, path)
        else:
            self.rospkgs[pkg] = [path]

    def load_path_from_env(self):
        env_str = os.environ.get("GAZEBO_MODEL_PATH", None)
        if env_str is None:
            return
        for path in env_str.split(':'):
            if os.path.isdir(path):
                self.add_path(path)

    def load_path_from_plugin(self):
        cmd = "rospack plugins --attrib=gazebo_model_path gazebo_ros"
        lines = subprocess.check_output(cmd, shell=True)
        if lines:
            for line in lines.split(os.linesep):
                if not line:
                    continue
                pkg, path = line.split()
                if os.path.isdir(path):
                    self.add_path(path)
                    if pkg not in self.rospkgs:
                        self.rospkgs[pkg] = self.rospack.get_path(pkg)

    def parse_model_path(self, path):
        key = None
        if path.startswith("model://"):
            spath = path[len("model://"):].split('/')
            key = spath[0]
            path = '/'.join(spath[1:])
        return key, path

    def search_path(self, key):
        for base_path in self.paths:
            path = os.path.join(base_path, key)
            if os.path.isdir(path):
                self.cached_path[key] = path

    def pack_ros_path(self, path):
        for pkg, pkg_path in self.rospkgs.items():
            if path.startswith(pkg_path):
                return "package://" + pkg + path[len(pkg_path):]
        return path

    def resolve_path(self, path):
        key, path = self.parse_model_path(path)
        if not key:
            return path
        if key not in self.cached_path:
            self.search_path(key)
        if key in self.cached_path:
            path = os.path.join(self.cached_path[key], path)
        else:
            raise Exception("path %s is not found" % path)
        return self.pack_ros_path(path)

_gazebo = GazeboModelPathResolver()

def resolve_gazebo_model_path(path):
    return _gazebo.resolve_path(path)

class URDF2XACRO(object):
    def __init__(self, urdf_path, xacro_path, force_rename=None):
        printf("loading", urdf_path)
        self.root = lxml.etree.parse(urdf_path,
                                     parser=lxml.etree.XMLParser(remove_blank_text=True))
        self.xacro_path = xacro_path
        self.force_rename = None

    def find_root_link(self):
        links = self.root.xpath("//robot/link")
        if len(links) == 0:
            raise InvalidURDFException("No link found in model")
        elif len(links) == 1:
            return links[0]
        else:
            link_names = [l.get("name") for l in links]
            joints = self.root.xpath("//robot/joint")
            for j in joints:
                child_link_name = j.find("child").get("link")
                if child_link_name in link_names:
                    link_names.remove(child_link_name)
            if len(link_names) != 1:
                raise InvalidURDFException("Links are not connected with joints: %s" % link_names)
            return self.root.xpath("//robot/link[@name='%s']" % link_names[0])[0]

    def add_namespace(self, ns):
        ns_str = "${%s}_" % ns

        links = self.root.xpath("//robot/link")
        for l in links:
            l.set("name", ns_str + l.get("name"))

        gazebo = self.root.xpath("//robot/gazebo")
        for g in gazebo:
            if g.get("reference") is not None:
                g.set("reference", ns_str + g.get("reference"))

        joints = self.root.xpath("//robot/joint")
        for j in joints:
            j.set("name", ns_str + j.get("name"))
            j_parent = j.find("parent")
            j_parent.set("link", ns_str +  j_parent.get("link"))
            j_child = j.find("child")
            j_child.set("link", ns_str + j_child.get("link"))

        transmissions = self.root.xpath("//robot/transmission")
        for t in transmissions:
            t.set("name", ns_str + t.get("name"))
            t_joint = t.find("joint")
            t_joint.set("name", ns_str + t_joint.get("name"))
            t_actuator = t.find("actuator")
            t_actuator.set("name", ns_str + t_actuator.get("name"))

    def replace_model_path(self):
        try:
            for param in ["visual", "collision", "inertial"]:
                for m in self.root.xpath("//robot/link/%s/geometry/mesh" % param):
                    if "filename" in m.attrib:
                        modelpath = m.attrib["filename"]
                        if self.force_rename and modelpath.startswith("model://"):
                            pkgpath = self.force_rename + modelpath[len("model://"):]
                        else:
                            pkgpath = resolve_gazebo_model_path(modelpath)
                        printf(modelpath, "->", pkgpath)
                        m.attrib["filename"] = pkgpath
        except:
            pass

    def inject_macro(self):
        self.add_namespace("name")
        root_link = self.find_root_link()
        robot = self.root.getroot()
        robot_name = robot.get("name")
        first_joint = robot.find("joint")
        virtual_joint = lxml.etree.Element("joint",
                                           attrib={"name": "${name}_root_parent_joint", "type": "fixed"})
        virtual_joint.append(lxml.etree.Element("parent", attrib={"link": "${parent}"}))
        virtual_joint.append(lxml.etree.Element("child", attrib={"link": root_link.get("name")}))
        virtual_joint.append(lxml.etree.Element("insert_block", attrib={"name": "origin"}))
        if first_joint is not None:
            first_joint.addprevious(virtual_joint)
        else:
            robot.append(virtual_joint)
        macro = lxml.etree.Element("macro",
                                   attrib={"name": robot_name, "params": "name parent *origin"})
        for e in robot.getchildren():
            macro.append(e)
        for e in robot.getchildren():
            robot.remove(e)
        robot.append(macro)

    def save(self):
        out_path = os.path.abspath(self.xacro_path)
        if not os.path.exists(os.path.dirname(out_path)):
            os.makedirs(os.path.dirname(out_path))
        xmlstring = lxml.etree.tostring(self.root,
                                        encoding="utf-8",
                                        xml_declaration=True,
                                        pretty_print=True,
                                        with_comments=True)
        with open(out_path, "w") as f:
            f.write(xmlstring)
        printf("saved to", out_path)
    def convert(self):
        self.replace_model_path()
        self.inject_macro()
        self.save()

if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(description="xacrify urdf file")
    p.add_argument("urdf", type=str, help="path to input urdf file")
    p.add_argument("xacro", type=str, help="path to output xacro file")
    p.add_argument("-f", "--force-rename", type=str,
                   default=None, help="force replace model:// tag with specified name")
    args = p.parse_args()

    c = URDF2XACRO(args.urdf, args.xacro, args.force_rename)
    c.convert()
