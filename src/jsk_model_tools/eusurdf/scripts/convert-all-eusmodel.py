#! /usr/bin/env python

import sys
import os
import os.path
import commands
import re


def convert_all_eusmodel ():

    urdfmodel_dir_path = commands.getoutput('rospack find eusurdf') + "/models/"
    eusmodel_dir_path = commands.getoutput('rospack find euslisp') + "/jskeus/eus/models/"
    eusmodel_file_path_list = commands.getoutput('ls ' + eusmodel_dir_path + '*.l -1')

    for eusmodel_file_path in eusmodel_file_path_list.split('\n'):
        eusmodel_file_name = os.path.basename(eusmodel_file_path)
        print "[convert-all-eusmodel] converting %s" % eusmodel_file_name
        regexp_search_res = re.compile("(.*)\-object\.l").search(eusmodel_file_name)
        if regexp_search_res:
            eusmodel_function_name = regexp_search_res.group(1)
            os.system('rosrun roseus roseus "(progn (load \\"package://eusgazebo/euslisp/eusgazebo-util.l\\") (load \\"package://euslisp/jskeus/eus/models/%s\\") (irteus2urdf-for-gazebo (%s) :urdf-dir \\"%s\\") (exit))"' % (eusmodel_file_name, eusmodel_function_name, urdfmodel_dir_path))
            os.system('rosrun roseus roseus "(progn (load \\"package://pr2eus_moveit/euslisp/eus2scene.l\\") (load \\"package://euslisp/jskeus/eus/models/%s\\") (generate-scene-file-from-eusobject (%s) \\"%s\\") (exit))"' % (eusmodel_file_name, eusmodel_function_name, urdfmodel_dir_path+eusmodel_function_name+"/model.scene"))

if __name__ == '__main__':
    convert_all_eusmodel()
