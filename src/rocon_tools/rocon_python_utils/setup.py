#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_python_utils', 'rocon_python_utils.network', 'rocon_python_utils.ros', 'rocon_python_utils.system', 'rocon_python_utils.yaml'],
    package_dir={'': 'src'},
    scripts=['scripts/rocon_find_resource',
             ],
)

setup(**d)
