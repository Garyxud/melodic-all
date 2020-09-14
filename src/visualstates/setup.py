#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   scripts=['scripts/main.py'],
   packages=['visualstates', 'visualstates.gui', \
             'visualstates.codegen', 'visualstates.codegen.python', 'visualstates.configs', \
             'visualstates.core', 'visualstates.generators', 'visualstates.gui.automata', \
             'visualstates.gui.dialogs', 'visualstates.gui.state', 'visualstates.gui.transition', \
             'visualstates.gui.tree', 'visualstates.gui.util', 'visualstates.parsers'],
   package_dir={'': 'src'}
)

setup(**d)
