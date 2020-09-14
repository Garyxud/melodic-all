#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

from __future__ import print_function
import os
from rocon_python_utils.yaml import Configuration

##############################################################################
# Tests
##############################################################################

def test_merge():
    print('Testing yaml merging')
    configuration = Configuration.from_file('./configuration.yaml').configure()
    assert configuration.dude == 'joe'
    assert configuration.dudette == 'jane'
    customisation = Configuration.from_file('./customisation.yaml').configure()
    assert customisation.dude == 'tarzan'
    new_configuration = configuration.merge(customisation)
    assert new_configuration.dude == 'tarzan'
    assert new_configuration.dudette == 'jane'
