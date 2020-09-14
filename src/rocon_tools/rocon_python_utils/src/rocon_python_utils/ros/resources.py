#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Description
##############################################################################

"""
.. module:: ros.resources
   :platform: Unix
   :synopsis: Helpers for working with ros resource names.

This module contains helpers that lookup or collect an index of ros resource
names for various purposes.

----

"""

##############################################################################
# Imports
##############################################################################

# system
import os

# ros
import rospkg
import roslib.names

from .catkin import package_index_from_package_path

##############################################################################
# Resources
##############################################################################


def find_resource_from_string(resource, rospack=None, extension=None):
    '''
      Convenience wrapper around roslib to find a resource (file) inside
      a package. This function passes off the work to find_resource
      once the input string is split.

      Pass it a shared rospack (:class:`.rospkg.RosPack`) object to accelerate
      the crawling across the ROS_PACKAGE_PATH when you are calling this
      function for many resources consecutively.

      .. code-block:: python

           rospack = rospkg.RosPack()
           for ros_resource_name in ['rocon_interactions/pc.interactions', 'rocon_interactions/demo.interactions']
               filename = find_resource_from_string(ros_resource_name, rospack)
               # do something

      :param str resource: ros resource name (in the form package/filename)
      :param rospack: a caching utility to help accelerate catkin filesystem lookups
      :type rospack: :class:`.rospkg.RosPack`
      :param str extension: file name extension to look for/expect

      :returns: full pathname to the resource
      :rtype: str

      :raises: :exc:`.rospkg.ResourceNotFound` raised if the resource is not found or has an inappropriate extension.
    '''
    unused_package, filename = find_resource_pair_from_string(resource, rospack, extension)
    return filename


def find_resource_pair_from_string(resource, rospack=None, extension=None):
    '''
      Convenience wrapper around roslib to find a resource (file) inside
      a package. This function passes off the work to find_resource
      once the input string is split.

      Pass it a shared rospack (:class:`.rospkg.RosPack`) object to accelerate
      the crawling across the ROS_PACKAGE_PATH when you are calling this
      function for many resources consecutively.

      .. code-block:: python

           rospack = rospkg.RosPack()
           for ros_resource_name in ['rocon_interactions/pc.interactions', 'rocon_interactions/demo.interactions']
               filename = find_resource_from_string(ros_resource_name, rospack)
               # do something

      :param str resource: ros resource name (in the form package/filename)
      :param rospack: a caching utility to help accelerate catkin filesystem lookups
      :type rospack: :class:`.rospkg.RosPack`
      :param str extension: file name extension to look for/expect

      :returns: (package, full pathname) to the resource
      :rtype: (str, str)

      :raises: :exc:`.rospkg.ResourceNotFound` raised if the resource is not found or has an inappropriate extension.
    '''
    if extension is not None:
        filename_extension = os.path.splitext(resource)[-1]
        if filename_extension == '':  # no ext given
            resource += ".%s" % extension
        elif filename_extension != "." + extension and filename_extension != extension:
            raise rospkg.ResourceNotFound("resource with invalid filename extension specified [%s][%s]" % (resource, extension))
    try:
        package, filename = package_resource_name(resource)
    except rospkg.ResourceNotFound:
        raise rospkg.ResourceNotFound("resource with invalid ros name specified [%s][%s]" % (resource, extension))
    if not package:
        raise rospkg.ResourceNotFound("resource could not be split with a valid leading package name [%s]" % (resource))
    return (package, find_resource(package, filename, rospack))


def package_resource_name(name):
    """
    Split a name into its package and resource name parts, e.g.

     - 'std_msgs/String -> std_msgs, String'
     - 'gopher_gazebo/gocart.xacro -> gopher_gazebo, gocart.xacro'
    -  'gopher_gazebo/urfd/gocart.xacro -> gopher_gazebo, urdf/gocart.xacro'

    This emulates what the original roslib.names.package_resource_name() function did, but also caters
    for the third example above where its a full relative path inside the package so as to disambiguate
    against multiple matches.

    @param name: package resource name, e.g. 'std_msgs/String'
    @type  name: str
    @return: package name, resource name
    @rtype: str
    @raise rospkg.ResourceNotFound: if name is invalid (cannot split into two arguments)
    """
    if roslib.names.PRN_SEPARATOR in name:
        val = tuple(name.split(roslib.names.PRN_SEPARATOR, 1))
        if len(val) <= 1:
            raise rospkg.ResourceNotFound("invalid name [%s]" % name)
        else:
            return val
    else:
        return '', name


def find_resource(package, filename, rospack=None):
    '''
      Convenience wrapper around roslib to find a resource (file) inside
      a package. It checks the output, and provides the appropriate
      error if there is one.

      :param str package: ros package
      :param str filename: some file inside the specified package
      :returns: absolute path to the file
      :rtype: str

      :raises: :exc:`.rospkg.ResourceNotFound` : raised if there is nothing found or multiple objects found.
    '''
    # try an explicit lookup first by concatenating package and filename paths, it's faster and avoids ambiguities
    resolved_path = _find_resource_explicitly(package, filename)
    if resolved_path is not None:
        return resolved_path
    # must be a 'convenient' resource path.
    try:
        resolved = roslib.packages.find_resource(package, filename, rospack=rospack)
        if not resolved:
            raise rospkg.ResourceNotFound("cannot locate [%s] in package [%s]" % (filename, package))
        elif len(resolved) == 1:
            return resolved[0]
        elif len(resolved) > 1:
            raise rospkg.ResourceNotFound("multiple resources named [%s] in package [%s]:%s\nPlease specify full path instead" % (filename, package, ''.join(['\n- %s' % r for r in resolved])))
    except rospkg.ResourceNotFound:
        raise rospkg.ResourceNotFound("[%s] is not a package or launch file name [%s]" % (package, package + '/' + filename))
    return None


def _find_resource_explicitly(package, filename, rospack=None):
    """
    When find resource starts, it will try and link up the package and filename explictly via
    an os.path.join and see if that works as it is faster and can avoid ambiguities.

    :param str package:
    :param str filename: either a ros resource name part, or relative to the package root
    :param rospkg.RosPack rospack: the package finder
    :returns: the absolute path or None
    :rtype: str or None
    """
    if rospack is None:
        rospack = rospkg.RosPack()
    # lookup package as it *must* exist
    pkg_path = rospack.get_path(package)
    explicit_filename_path = os.path.join(pkg_path, filename)
    if os.path.isfile(explicit_filename_path):
        return explicit_filename_path
    else:
        return None

def resource_index_from_package_exports(export_tag, package_paths=None, package_whitelist=None, package_blacklist=[]):
    '''
      Scans the package path looking for exports and grab the ones we are interested in.

      :param str export_tag: export tagname
      :param package_paths: list of package paths to scan over
      :type package_paths: str[]
      :param package_whitelist: list of packages to include (and no other)
      :type package_paths: str[]
      :param package_blacklist: list of packages to exclude
      :type package_paths: str[]

      :returns: the dictionary of resource and its absolute path
      :rtype: dict { resource_name : os.path }
    '''
    package_index = _get_package_index(package_paths)
    resources = {}
    invalid_resources = {}
    for package in package_index.values():

        if package_whitelist:
            if package.name not in package_whitelist:
                continue
        elif package.name in package_blacklist:
            continue
        for export in package.exports:
            if export.tagname == export_tag:
                filename_relative_path = export.content
                resource_name = package.name + '/' + os.path.splitext(os.path.basename(filename_relative_path))[0]
                resource_filename = os.path.join(os.path.dirname(package.filename), filename_relative_path)
                if not os.path.isfile(resource_filename):
                    invalid_resources[resource_name] = resource_filename
                else:
                    resources[resource_name] = (resource_filename, package)
    return (resources, invalid_resources)


def _get_package_index(package_paths):
    # should make use of rospkg.get_ros_paths here.
    # http://docs.ros.org/independent/api/rospkg/html/rospkg_environment.html
    ros_package_path = package_paths if package_paths else os.getenv('ROS_PACKAGE_PATH', '')
    ros_package_path = [x for x in ros_package_path.split(':') if x]
    package_index = package_index_from_package_path(ros_package_path)
    return package_index
