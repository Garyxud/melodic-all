#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_semantic_version'],
    package_dir={'': 'src'},
#    scripts=['scripts/rospair',
#             ],
)

setup(**d)
