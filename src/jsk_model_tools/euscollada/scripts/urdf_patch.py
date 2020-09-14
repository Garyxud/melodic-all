#!/usr/bin/env python

""" A script to extract and apply difference between two urdfs """

import argparse
from urdf_parser_py.urdf import URDF
import yaml
import sys

def findJointByName(robot, name):
    for j in robot.joints:
        if j.name == name:
            return j
    return None
    
def jointDiff(original, new):
    """
    only take origin into account
    """
    if hasattr(original, "origin") and hasattr(original.origin, "xyz"):
        pos_original = original.origin.xyz or [0, 0, 0]
    else:
        pos_original = [0, 0, 0]
    if hasattr(original, "origin") and hasattr(original.origin, "rpy"):
        rpy_original = original.origin.rpy or [0, 0, 0]
    else:
        rpy_original = [0, 0, 0]
    pos_new = new.origin.xyz or [0, 0, 0]
    rpy_new = new.origin.rpy or [0, 0, 0]
    if (pos_original[0] == pos_new[0] and 
        pos_original[1] == pos_new[1] and 
        pos_original[2] == pos_new[2]):
        pos_diff = None
    else:
        print "old: ", pos_original
        print "new: ", pos_new
        pos_diff = pos_new
    if (rpy_original[0] == rpy_new[0] and 
        rpy_original[1] == rpy_new[1] and 
        rpy_original[2] == rpy_new[2]):
        rpy_diff = None
    else:
        print "old: ", rpy_original
        print "new: ", rpy_new
        rpy_diff = rpy_new
    if not pos_diff and not rpy_diff:
        return None
    elif pos_diff and rpy_diff:
        return {"xyz": pos_diff, "rpy": rpy_diff}        
    elif pos_diff:
        return {"xyz": pos_diff}
    elif rpy_diff:
        return {"rpy": rpy_diff}
    return None
    
def runDiff(original_file, new_file, output_file):
    original_robot = URDF.from_xml_file(original_file)
    new_robot = URDF.from_xml_file(new_file)
    # only joint and link are considered
    diffs = dict()
    for j in original_robot.joints:
        new_j = findJointByName(new_robot, j.name)
        # check origin difference
        if new_j:
            diff = jointDiff(j, new_j)
            if diff:
                diffs[j.name] = diff
    with open(output_file, "w") as f:
        f.write(yaml.dump(diffs))
        print yaml.dump(diffs)

def runPatch(input_file, patch_yaml, output_file):
    input_robot = URDF.from_xml_file(input_file)
    patch_param = yaml.load(open(patch_yaml))
    for joint_name in patch_param.keys():
        diff = patch_param[joint_name]
        if diff.has_key("xyz"):
            j = input_robot.joint_map[joint_name]
            j.origin.xyz = diff["xyz"]
        if diff.has_key("rpy"):
            j = input_robot.joint_map[joint_name]
            j.origin.rpy = diff["rpy"]
    if output_file:
        with open(output_file, "w") as f:
            f.write(input_robot.to_xml_string())
    else:
        print input_robot.to_xml_string()
        
def parser():
    p = argparse.ArgumentParser(description="Get and apply urdf patch")
    p.add_argument("mode", help="diff or patch")
    p.add_argument("arg0", help="""If mode is diff, specify original urdf. 
    If mode is patch, specify original urdf.""")
    p.add_argument("arg1", help="""If mode is diff, specify new urdf.
    If mode is patch, specify patch yaml file.
    """)
    p.add_argument("output", help="output file. You can omit this argument in diff mode and in that case the new urdf is output in STDOUT.", nargs="?")
    return p

if __name__ == "__main__":
    p = parser()
    args = p.parse_args()
    if args.mode == "diff":
        if not args.output:
            p.print_help()
            sys.exit(1)
        runDiff(args.arg0, args.arg1, args.output)
    elif args.mode == "patch":
        runPatch(args.arg0, args.arg1, args.output)
    else:
        raise Exception("unknown mode %s" % (args.mode))
