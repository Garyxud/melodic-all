#!/usr/bin/env python

from xml.dom import minidom
import sys
import tf
from tf.transformations import *
import yaml
import add_sensor_to_urdf
import math

def usage():
    print "Usage:"
    print "  add_eef_to_urdf.py input_urdf input_yaml output_urdf"


def addEEF(xdoc, end_effector_info, limb_info, limb_name):
    if end_effector_info.has_key("parent"):
        parent = end_effector_info["parent"]
    else:
        parent = limb_info[-1].keys()[0].replace("JOINT", "LINK") # not goood!
    if end_effector_info.has_key("translate"):
        pos = end_effector_info["translate"]
    else:
        pos = [0, 0, 0]
    if end_effector_info.has_key("rotate"):
        q = quaternion_about_axis(math.radians(end_effector_info['rotate'][3]), 
                            end_effector_info['rotate'][:3])
        rpy = euler_from_quaternion(q)
    else:
        rpy = [0, 0, 0]
    add_sensor_to_urdf.addLink(xdoc, pos, rpy, parent, limb_name + "_end_coords")
def updateURDF(input_file, yaml_file, output_file):
    xdoc = minidom.parse(input_file)
    with open(yaml_file, "r") as yaml_f:
        yaml_data = yaml.load(yaml_f)
        addEEF(xdoc, yaml_data["rleg-end-coords"], yaml_data["rleg"], "rleg")
        addEEF(xdoc, yaml_data["lleg-end-coords"], yaml_data["lleg"], "lleg")
        addEEF(xdoc, yaml_data["rarm-end-coords"], yaml_data["rarm"], "rarm")
        addEEF(xdoc, yaml_data["larm-end-coords"], yaml_data["larm"], "larm")
        addEEF(xdoc, yaml_data["head-end-coords"], yaml_data["head"], "head")
    with open(output_file, "w") as f:
        f.write(xdoc.toprettyxml())


# argument
# x y z yaw pitch roll    parent child(new link)
# x y z qx  qy    qz   qw parent child(new link)

def main(argv):
    if len(argv) == 3:
        updateURDF(argv[0], argv[1], argv[2])
    else:
        usage()
        exit(1)

if __name__ == "__main__":
    main(sys.argv[1:])
