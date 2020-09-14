#!/usr/bin/env python

from xml.dom import minidom
import sys
import tf
from tf.transformations import *
def usage():
    print "Usage:"
    print "  add_sensor_to_urdf.py x y z roll pitch yaw    parent child(new link) input_urdf output_urdf"

def addLink(xdoc, pos, rpy, parent, child):
    robot = xdoc.getElementsByTagName("robot")[0]
    # creating child joint
    child_joint = xdoc.createElement("joint")
    child_joint.setAttribute("name", child + "_joint")
    child_joint.setAttribute("type", "fixed")
    origin_element = xdoc.createElement("origin")
    origin_element.setAttribute("xyz", "%s %s %s" % (pos[0], pos[1], pos[2]))
    origin_element.setAttribute("rpy", "%s %s %s" % (rpy[0], rpy[1], rpy[2]))
    child_joint.appendChild(origin_element)
    parent_element = xdoc.createElement("parent")
    parent_element.setAttribute("link", parent)
    child_joint.appendChild(parent_element)
    child_element = xdoc.createElement("child")
    child_element.setAttribute("link", child)
    child_joint.appendChild(child_element)
    
    child_link = xdoc.createElement("link")
    child_link.setAttribute("name", child)
    robot.appendChild(child_joint)
    robot.appendChild(child_link)
    return xdoc
def updateURDF(pos, rpy, parent, child, input_file, output_file):
    """
    the link tree will be:
      ${parent}  --> ${child}_joint --> ${child}
    and xml is
    <joint name="${child}_joint" type="fixed">
      <origin xyz="x y z" rpy="r p y" />
      <parent link="${parent}" />
      <child link="${child}" />
    </joint>
    <link name="${child}" />
    """
    xdoc = minidom.parse(input_file)
    xdoc = addLink(xdoc, pos, rpy, parent, child)
    with open(output_file, "w") as f:
        f.write(xdoc.toprettyxml())


# argument
# x y z yaw pitch roll    parent child(new link)
# x y z qx  qy    qz   qw parent child(new link)

def main(argv):
    if len(argv) == 10:
        parent = argv[6]
        child = argv[7]
        input_file = argv[8]
        output_file = argv[9]
        updateURDF([argv[0], argv[1], argv[2]], 
                   [argv[3], argv[4], argv[5]], 
                   parent, child, input_file, output_file)
    else:
        usage()
        exit(1)

if __name__ == "__main__":
    main(sys.argv[1:])
