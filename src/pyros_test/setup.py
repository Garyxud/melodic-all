from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# ROS PACKAGING
# using distutils : https://docs.python.org/2/distutils
# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'pyros_test',
    ],
    package_dir={
        'pyros_test': 'src/pyros_test',
    }
)
setup(**setup_args)
