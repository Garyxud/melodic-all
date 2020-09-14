REM generated from ros_environment/env-hooks/1.ros_package_path.bat.em

REM python function to generate ROS package path based on all parent workspaces (prepends the separator if necessary)
REM do not use EnableDelayedExpansion here, it messes with the != symbols
setlocal disabledelayedexpansion
echo from __future__ import print_function > _parent_package_path.py
echo import os >> _parent_package_path.py
echo from collections import OrderedDict >> _parent_package_path.py
echo env_name = 'CMAKE_PREFIX_PATH' >> _parent_package_path.py
echo paths = [path for path in os.environ[env_name].split(os.pathsep)] if env_name in os.environ and os.environ[env_name] != '' else [] >> _parent_package_path.py
echo workspaces = [path for path in paths if os.path.exists(os.path.join(path, '.catkin'))] >> _parent_package_path.py
echo workspaces = list(OrderedDict.fromkeys([os.path.normpath(ws) for ws in workspaces])) >> _parent_package_path.py
echo paths = [] >> _parent_package_path.py
echo for workspace in workspaces: >> _parent_package_path.py
echo     filename = os.path.join(workspace, '.catkin') >> _parent_package_path.py
echo     data = '' >> _parent_package_path.py
echo     with open(filename) as f: >> _parent_package_path.py
echo         data = f.read() >> _parent_package_path.py
echo     if data == '': >> _parent_package_path.py
echo         paths.append(os.path.join(workspace, 'share')) >> _parent_package_path.py
echo         if os.path.isdir(os.path.join(workspace, 'stacks')): >> _parent_package_path.py
echo             paths.append(os.path.join(workspace, 'stacks')) >> _parent_package_path.py
echo     else: >> _parent_package_path.py
echo         for source_path in data.split(';'): >> _parent_package_path.py
echo             paths.append(source_path) >> _parent_package_path.py
echo print(os.pathsep.join(paths)) >> _parent_package_path.py
endlocal

setlocal EnableDelayedExpansion

set ROS_PACKAGE_PATH_PARENTS=
for /f %%a in ('@(PYTHON_EXECUTABLE) _parent_package_path.py') do set ROS_PACKAGE_PATH_PARENTS=!ROS_PACKAGE_PATH_PARENTS!%%a

set ROS_PACKAGE_PATH=%ROS_PACKAGE_PATH_PARENTS%

del _parent_package_path.py

REM Make sure the variable survives local scope
endlocal && set ROS_PACKAGE_PATH=%ROS_PACKAGE_PATH%
