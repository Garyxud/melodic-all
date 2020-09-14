#! /usr/bin/env python

import sys
import os
import commands


def make_fixed_model (name, eusurdf_package_path=commands.getoutput('rospack find eusurdf'), overwrite=True):
    urdf_dir_path = eusurdf_package_path + '/models/' + name
    fixed_urdf_dir_path = urdf_dir_path + '_fixed'
    urdf_path = urdf_dir_path + '/' + 'model.urdf'
    fixed_urdf_path = fixed_urdf_dir_path + '/' + 'model.urdf'

    if overwrite:
        os.system("rm -rf %s" % fixed_urdf_dir_path)
    else:
        if os.path.exists(fixed_urdf_dir_path):
            print '[ERROR] the same name fixed model already exits'
            exit(1)

    os.system("cp -r %s %s" % (urdf_dir_path, fixed_urdf_dir_path))

    root_link_name = commands.getoutput('grep "link name=".*">" %s  | head -n 1 | sed -e "s/  <link name=\\\"\(.*\)\\\">/\\1/"' % fixed_urdf_path)
    os.system('sed -i -e \"s@</robot>@  <!-- Used for fixing robot to Gazebo \'base_link\' -->\\n  <link name=\\\"world\\\"/>\\n  <joint name=\\\"fixed\\\" type=\\\"fixed\\\">\\n    <parent link=\\\"world\\\"/>\\n    <child link=\\\"%s\\\"/>\\n  </joint>\\n</robot>@g\" %s' % (root_link_name, fixed_urdf_path))

    os.system('sed -i -e "s@<robot name=\\"\(.*\)\\"@<robot name=\\"\\1_fixed\\"@g\" %s' % fixed_urdf_path)


if __name__ == '__main__':
    if len(sys.argv) > 2:
        make_fixed_model(sys.argv[1], eusurdf_package_path=sys.argv[2])
    elif len(sys.argv) > 1:
        make_fixed_model(sys.argv[1])
