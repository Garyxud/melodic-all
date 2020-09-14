from __future__ import absolute_import
from __future__ import print_function

import inspect

import logging

logger = logging.getLogger(__name__)


#: Gets current module path via inspection (in case we cannot rely on __file__ like with ros generated __init__)
def get_ros_executed_file():
    """
    Detect if the current package has been loaded from ros devel redirect init script.
    This function needs to be called inside the original __init__.py of your package to work correctly, after importing all generated ROS modules.

    <TODO : Example>

    :param init_file: the fullpath of the init file for this package.
            It will be used to check if it was exec'd from a ros generated __init__.py
    :return: True if import from ros __init__.py was detected. False otherwise.
    """

    s = inspect.stack()
    detected = False
    for f in s:
        logger.debug(f)
        if f[1] == __file__:
            continue  # we skip this frame it concerns this file
        elif f[1] == '<string>':
            continue  # we skip this frame, coming from exec() call
        else:
            for m, v in inspect.getmembers(f[0]):  # inspecting members of that frame
                if m == 'f_globals' and '__execfiles' in v:
                    detected = v.get('__execfile')
                    logger.debug("Detected ROS generated __init__ exec() relay for {0}".format(detected))
                    break  # we break at the first __execfile (ros generated __init__) we find

    return detected
    # We do need to clean up everything to avoid memory reference cycles
    # Ref : https://docs.python.org/2/library/inspect.html#the-interpreter-stack
    # We are assigning in function so cleanup will be done when returning

