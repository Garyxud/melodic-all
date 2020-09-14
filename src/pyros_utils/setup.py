# PYTHON PACKAGING
# using setuptools : http://pythonhosted.org/setuptools/
from setuptools import setup

with open('pyros_utils/_version.py') as vf:
    exec(vf.read())

setup(name='pyros_utils',
    version=__version__,
    description='This is a ROS package, providing useful ROS addons for pyros.',
    url='http://github.com/asmodehn/pyros-utils',
    author='AlexV',
    author_email='asmodehn@gmail.com',
    license='BSD',
    packages=[
        'pyros_utils',
        'pyros_utils.tests',
    ],
    entry_points={
        'console_scripts': [
            'pyros_utils = pyros_utils.__main__:nosemain'
        ]
    },
    # this is better than using package data ( since behavior is a bit different from distutils... )
    include_package_data=True,  # use MANIFEST.in during install.
    install_requires=[
        'catkin_pkg',  # not needed here since this version should not look for package.xml
        'six',
    ],
    test_suite="nose.collector",
    tests_require=[
        'nose>=1.3.7'
    ],
    zip_safe=False,  # TODO testing...
)

