#!/usr/bin/env python

""" Python unittest script for camera_info_manager module.

Requires a rostest environment, allowing test cases to make
set_camera_info service calls, where needed.
"""
# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

PKG='camera_info_manager_py'
import rospkg
import sys
import os
import stat
import unittest

from camera_info_manager import *

g_package_name = PKG
g_test_name = "test_calibration"
g_package_filename = "/tests/" + g_test_name +".yaml"
g_package_url = "package://" + g_package_name + g_package_filename
g_package_name_url = "package://" + g_package_name + "/tests/${NAME}.yaml"

g_test_home = "/tmp"                   # unit test ${HOME} setting
g_ros_home = g_test_home + "/.ros"     # unit test ${ROS_HOME} setting
g_camera_name = "camera"
g_default_yaml = g_ros_home + "/camera_info/" + g_camera_name + ".yaml"
g_default_url = "file://${ROS_HOME}/camera_info/${NAME}.yaml"

def delete_tmp_camera_info_directory():
    """ Delete the default camera info directory in /tmp.

    Do not complain if it does not exist.
    """
    os.system("rm -rf " + g_ros_home + "/camera_info")

def delete_file(filename):
    """ Delete a file, not complaining if it does not exist.

    :param filename: path to file.
    """
    try:
        os.remove(filename)
    except OSError:             # OK if file did not exist
        pass

def expected_calibration():
    """ These data must match the contents of test_calibration.yaml."""

    ci = CameraInfo()
    ci.width = 640
    ci.height = 480

    # set distortion coefficients
    ci.distortion_model = "plumb_bob"
    ci.D = [-0.26129794156876202, 0.053510647147691104,
             -0.004329961180682111, 0.0002979023290858089, 0]

    # set camera matrix
    ci.K = [259.79888071407669, 0.0, 332.0316187674498, 0.0,
            258.00868558667878, 252.46066959143357, 0.0, 0.0, 1.0]

    # set rectification matrix
    ci.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    # set projection matrix
    ci.P = [259.79888071407669, 0.0, 332.0316187674498, 0.0, 0.0,
            258.00868558667878, 252.46066959143357, 0.0, 0.0, 0.0, 1.0, 0.0]

    return ci

g_camera_info_manager = None

def init_camera_info_manager(cname='camera', url=''):
    """ Return an initialized CameraInfoManager instance for testing.

    The `set_camera_info` service does not get freed immediately when
    a test terminates and the :py:cls:`CameraInfoManager` object goes
    out of scope.  That creates a problem when allocating new
    instances for any tests that follow.

    As a work-around, this function manages a single global instance,
    filling in the desired values, as requested.  Its state emulates
    that of a new call to the CameraInfoManager constructor.
    """
    global g_camera_info_manager
    if g_camera_info_manager is None: # first time here?
        g_camera_info_manager = CameraInfoManager(cname=cname, url=url)
    else:
        g_camera_info_manager.setCameraName(cname)
        g_camera_info_manager.setURL(url)
        g_camera_info_manager.camera_info = None
    return g_camera_info_manager

def set_calibration(calib):
    """ Issue SetCameraInfo service request."""
    rospy.wait_for_service('set_camera_info')
    try:
        proxy = rospy.ServiceProxy('set_camera_info', SetCameraInfo)
        rsp = proxy(calib)
        return rsp
    except rospy.ServiceException, e:
        print("Service call failed: " + str(e))
        return None

class TestCameraInfoManager(unittest.TestCase):
    """Unit tests for Python camera_info_manager.
    """

    # camera name tests

    def test_valid_camera_names(self):
        """Test that valid camera names are accepted."""
        cinfo = init_camera_info_manager()

        # a list of valid names to try:
        names = ["a", "1", "_",
                 "A1", "9z",
                 "abcdefghijklmnopqrstuvwxyz",
                 "ABCDEFGHIJKLMNOPQRSTUVWXYZ",
                 "0123456789",
                 "0123456789abcdef",
                 "axis_00408c8ae301_local"
                 "08144361026320a0_640x480_mono8"]
        for cn in names:
            self.assertTrue(cinfo.setCameraName(cn))
            self.assertEqual(cinfo.getCameraName(), cn)

    def test_invalid_camera_names(self):
        """Test that invalid camera names are rejected."""
        cinfo = init_camera_info_manager()

        # a list of invalid names to try:
        names = ["", "-21", "C++",
                 "axis-00408c8ae301.local",
                 "file:///tmp/url.yaml",
                 "file://${INVALID}/xxx.yaml"]
        for cn in names:
            self.assertFalse(cinfo.setCameraName(cn))
            self.assertEqual(cinfo.getCameraName(), g_camera_name)

    def test_gen_camera_name(self):
        """Test camera name generation."""

        # valid strings pass through unchanged
        self.assertEqual(genCameraName("a"), "a")
        self.assertEqual(genCameraName("1"), "1")
        self.assertEqual(genCameraName("_"), "_")
        self.assertEqual(genCameraName("0123456789abcdef"), "0123456789abcdef")
        self.assertEqual(genCameraName("08144361026320a0_640x480_mono8"),
                         "08144361026320a0_640x480_mono8")

        # invalid strings get '_' substitution
        self.assertEqual(genCameraName(""), "_")
        self.assertEqual(genCameraName("-21"), "_21")
        self.assertEqual(genCameraName("C++"), "C__")
        self.assertEqual(genCameraName("file:///tmp/url.yaml"),
                         "file____tmp_url_yaml")
        self.assertEqual(genCameraName("file://${INVALID}/xxx.yaml"),
                         "file_____INVALID__xxx_yaml")
        self.assertEqual(genCameraName("axis-00408c8ae301.local"),
                         "axis_00408c8ae301_local")

    # URL parsing and validation

    def test_url_substitution_no_variables(self):
        """ Test that URLs with no variables are handled correctly."""
        strings = ["",
                   "file:///tmp/url.yaml",
                   g_package_url,
                   "xxx://nonsense"]
        for url in strings:
            self.assertEqual(resolveURL(url, g_camera_name), url)

    def test_url_substitution_camera_name(self):
        """ Test URL ${NAME} variable resolution."""
        cn = g_camera_name
        os.environ["ROS_HOME"] = g_ros_home

        # test variable substitution
        self.assertEqual(resolveURL(g_package_name_url, g_test_name),
                         g_package_url)
        self.assertEqual(resolveURL(g_default_url, cn),
                         "file://" + g_default_yaml)
        name_url = ("package://" + g_package_name +
                    "/tests/${NAME}_calibration.yaml")
        self.assertEqual(resolveURL(name_url, 'test'), g_package_url)
        test_name = "camera_1024x768"
        self.assertEqual(resolveURL(name_url, test_name),
                         "package://" + g_package_name +
                         "/tests/" + test_name + "_calibration.yaml")
        self.assertEqual(resolveURL(name_url, ''),
                         "package://" + g_package_name +
                         "/tests/_calibration.yaml")

    def test_url_substitution_ros_home(self):
        """ Test URL ${ROS_HOME} variable resolution."""
        name_url = "file://${ROS_HOME}/camera_info/test_camera.yaml"

        # resolve ${ROS_HOME} with neither environment variable nor
        # $HOME defined (should leave the string unresolved)
        os.environ["ROS_HOME"] = 'x'    # ensure existence before deleting
        os.environ["HOME"] = 'x'
        del os.environ["ROS_HOME"]
        del os.environ["HOME"]
        exp_url = name_url              # leaves variable unresolved
        self.assertEqual(resolveURL(name_url, g_camera_name), exp_url)

        # resolve ${ROS_HOME} with environment variable undefined, but
        # $HOME defined
        os.environ['HOME'] = g_test_home # set $HOME value for test
        exp_url = "file://" + g_test_home + "/.ros/camera_info/test_camera.yaml"
        self.assertEqual(resolveURL(name_url, g_camera_name), exp_url)

        # resolve ${ROS_HOME} with environment variable defined
        os.environ["ROS_HOME"] = "/my/ros/home"
        exp_url = "file:///my/ros/home/camera_info/test_camera.yaml";
        self.assertEqual(resolveURL(name_url, g_camera_name), exp_url)

        # try /tmp
        os.environ["ROS_HOME"] = "/tmp"
        exp_url = "file:///tmp/camera_info/test_camera.yaml";
        self.assertEqual(resolveURL(name_url, g_camera_name), exp_url)

        # try the unit test default value
        os.environ["ROS_HOME"] = g_ros_home
        exp_url = "file:///tmp/.ros/camera_info/test_camera.yaml";
        self.assertEqual(resolveURL(name_url, g_camera_name), exp_url)

    def test_url_substitution_strange_dollar_signs(self):
        """ Test URL variable resolution with strange '$' characters."""

        # test for "$$" in the URL (NAME should be resolved)
        name_url = "file:///tmp/$${NAME}.yaml"
        exp_url = "file:///tmp/$" + g_camera_name + ".yaml"
        self.assertEqual(resolveURL(name_url, g_camera_name), exp_url)

        # test for "$" in middle of string
        name_url = "file:///$whatever.yaml"
        self.assertEqual(resolveURL(name_url, g_camera_name), name_url)

        # test for "$$" in middle of string
        name_url = "file:///something$$whatever.yaml"
        self.assertEqual(resolveURL(name_url, g_camera_name), name_url)

        # test for "$$" at end of string
        name_url = "file:///$$"
        self.assertEqual(resolveURL(name_url, g_camera_name), name_url)

    def test_valid_url_parsing(self):
        """ Test valid URL parsing."""

        self.assertEqual(parseURL(""), URL_empty)

        self.assertEqual(parseURL("file:///"), URL_file)
        self.assertEqual(parseURL("file:///tmp/url.yaml"), URL_file)
        self.assertEqual(parseURL("FILE:///tmp/url.yaml"), URL_file)

        self.assertEqual(parseURL(g_package_url), URL_package)
        self.assertEqual(parseURL("packAge://camera_info_manager/x"),
                         URL_package)
        self.assertEqual(parseURL("package://no_such_package/calibr.yaml"),
                         URL_package)

    def test_invalid_url_parsing(self):
        """ Test invalid URL parsing."""

        self.assertEqual(parseURL("file://"), URL_invalid)
        self.assertEqual(parseURL("flash:///"), URL_invalid)
        self.assertEqual(parseURL("html://ros.org/wiki/camera_info_manager"),
                         URL_invalid)
        self.assertEqual(parseURL("package://"), URL_invalid)
        self.assertEqual(parseURL("package:///"), URL_invalid)
        self.assertEqual(parseURL("package://calibration.yaml"), URL_invalid)
        self.assertEqual(parseURL("package://camera_info_manager_py/"),
                         URL_invalid)

    def test_get_package_filename(self):
        """ Test getPackageFileName() function."""

        # resolve known file in this package
        filename = getPackageFileName(g_package_url)
        rp = rospkg.RosPack()
        pkgPath = rp.get_path(g_package_name)
        expected_filename = pkgPath + g_package_filename
        self.assertEqual(filename, expected_filename)

        # resolve non-existent package
        filename = getPackageFileName("package://no_such_package/"
                                      + g_package_filename)
        self.assertEqual(filename, "")

    # calibration data handling

    def test_get_missing_info(self):
        """ Test ability to detect missing CameraInfo."""
        cinfo = init_camera_info_manager()
        self.assertRaises(CameraInfoMissingError, cinfo.isCalibrated)
        self.assertRaises(CameraInfoMissingError, cinfo.getCameraInfo)

    def test_get_info_without_environment(self):
        """ Test ability to detect missing CameraInfo when neither
        ${ROS_HOME} nor $HOME are defined."""

        # undefine the environment variables
        os.environ["ROS_HOME"] = 'x'    # ensure existence before deleting
        os.environ["HOME"] = 'x'
        del os.environ["ROS_HOME"]
        del os.environ["HOME"]

        # run the test
        cinfo = init_camera_info_manager()
        self.assertRaises(CameraInfoMissingError, cinfo.isCalibrated)
        self.assertRaises(CameraInfoMissingError, cinfo.getCameraInfo)
        self.assertEqual(cinfo.camera_info, None)
        cinfo.loadCameraInfo()
        self.assertEqual(parseURL(cinfo.getURL()), URL_empty)
        self.assertEqual(cinfo.getCameraInfo(), CameraInfo())
        self.assertFalse(cinfo.isCalibrated())

        # restore test $HOME
        os.environ['HOME'] = g_test_home

    def test_set_camera_name_info_invalidation(self):
        """ Test that setCameraName() invalidates camera info correctly."""

        # after loading camera info, it is uncalibrated, but not missing
        os.environ["ROS_HOME"] = g_ros_home
        delete_file(g_default_yaml)     # remove default URL file
        cinfo = init_camera_info_manager()
        cinfo.loadCameraInfo()
        self.assertFalse(cinfo.isCalibrated())
        self.assertEqual(cinfo.getCameraInfo(), CameraInfo())

        # setting the same camera name changes nothing
        cinfo.setCameraName(g_camera_name)
        self.assertFalse(cinfo.isCalibrated())
        self.assertEqual(cinfo.getCameraInfo(), CameraInfo())

        # setting a new camera name causes it to become missing
        cinfo.setCameraName('xxx')
        self.assertRaises(CameraInfoMissingError, cinfo.isCalibrated)
        self.assertRaises(CameraInfoMissingError, cinfo.getCameraInfo)

    def test_set_url_info_invalidation(self):
        """ Test that setURL() invalidates camera info correctly."""

        # after loading camera info, it is uncalibrated, but not missing
        os.environ["ROS_HOME"] = g_ros_home
        delete_file(g_default_yaml)     # remove default URL file
        cinfo = init_camera_info_manager()
        cinfo.loadCameraInfo()
        self.assertFalse(cinfo.isCalibrated())
        self.assertEqual(cinfo.getCameraInfo(), CameraInfo())

        # setting the same URL changes nothing
        my_url = cinfo.getURL()
        self.assertTrue(cinfo.setURL(my_url))
        self.assertFalse(cinfo.isCalibrated())
        self.assertEqual(cinfo.getCameraInfo(), CameraInfo())

        # setting a new URL causes it to become missing
        self.assertTrue(cinfo.setURL(g_package_url))
        self.assertRaises(CameraInfoMissingError, cinfo.isCalibrated)
        self.assertRaises(CameraInfoMissingError, cinfo.getCameraInfo)

    def test_load_calibration_file(self):
        """ Test loadCalibrationFile() function. """

        # try with an actual file in this directory
        rp = rospkg.RosPack()
        pkgPath = rp.get_path(g_package_name)
        filename = pkgPath + g_package_filename
        ci = loadCalibrationFile(filename, g_camera_name)
        self.assertEqual(ci, expected_calibration())

        # an empty file should return a null calibration
        filename = pkgPath + "/tests/empty.yaml"
        ci = loadCalibrationFile(filename, g_camera_name)
        self.assertEqual(ci, CameraInfo())

        # a non-existent file should return a null calibration
        os.environ["ROS_HOME"] = g_ros_home
        delete_file(g_default_yaml)
        ci = loadCalibrationFile(g_default_yaml, g_camera_name)
        self.assertEqual(ci, CameraInfo())

        # try file name with ${ROS_HOME} unresolved
        filename = "${ROS_HOME}/camera_info/" + g_camera_name + ".yaml"
        ci = loadCalibrationFile(filename, g_camera_name)
        self.assertEqual(ci, CameraInfo())

    def test_get_uncalibrated_info(self):
        """ Test ability to provide uncalibrated CameraInfo"""
        os.environ["ROS_HOME"] = g_ros_home
        delete_file(g_default_yaml)
        cinfo = init_camera_info_manager()
        cinfo.loadCameraInfo()
        self.assertFalse(cinfo.isCalibrated())
        self.assertEqual(cinfo.getCameraInfo(), CameraInfo())

    def test_get_calibrated_info(self):
        """ Test ability to provide calibrated CameraInfo"""
        cinfo = init_camera_info_manager(url=g_package_url)
        cinfo.loadCameraInfo()
        self.assertTrue(cinfo.isCalibrated())
        self.assertEqual(cinfo.getCameraInfo(), expected_calibration())

    # test saving of calibration data

    def test_save_calibration_file(self):
        """ Test saveCalibrationFile() function. """
        fname = g_default_yaml  # define some shorter names
        cname = g_camera_name

        # first save to non-existent file in non-existent directory
        os.environ["ROS_HOME"] = g_ros_home
        delete_tmp_camera_info_directory()
        exp = expected_calibration()
        self.assertTrue(saveCalibrationFile(exp, fname, cname))
        self.assertEqual(exp, loadCalibrationFile(fname, cname))

        # now remove the file, but not the directory        
        delete_file(fname)
        self.assertTrue(saveCalibrationFile(exp, fname, cname))
        self.assertEqual(exp, loadCalibrationFile(fname, cname))

        # make the file and directory non-writable, try to write a
        # null calibration (which should not work)
        #
        # DISABLE this part of the test, it does not work on Jenkins,
        # probably because it is running as root (issue #6)
        #os.chmod(fname, stat.S_IREAD)
        #dirname = os.path.dirname(fname)
        #os.chmod(dirname, stat.S_IREAD|stat.S_IEXEC)
        #self.assertFalse(saveCalibrationFile(CameraInfo(), fname, cname))
        #os.chmod(dirname, stat.S_IREAD|stat.S_IWRITE|stat.S_IEXEC)
        #os.chmod(fname, stat.S_IREAD)
        #self.assertEqual(exp, loadCalibrationFile(fname, cname))

        # clean up the mess this test created
        delete_tmp_camera_info_directory()

    def test_set_calibration(self):
        """ Test ability to set calibrated CameraInfo."""
        os.environ["ROS_HOME"] = g_ros_home
        delete_tmp_camera_info_directory()
        cinfo = init_camera_info_manager()
        exp = expected_calibration()
        resp = set_calibration(exp)

        # use assertTrue() instead of assertNotNone() so the test
        # works for Python versions before 2.7
        self.assertTrue(resp != None)
        self.assertTrue(resp.success)

        # only check results if the service succeeded, avoiding
        # confusing and redundant failure messages
        if resp.success:
            self.assertTrue(cinfo.isCalibrated())
            self.assertEqual(exp, cinfo.getCameraInfo())

    def test_save_calibration_default(self):
        """ Test ability to save calibrated CameraInfo in default URL."""
        os.environ["ROS_HOME"] = g_ros_home
        delete_tmp_camera_info_directory()

        # create instance to save calibrated data
        cinfo = init_camera_info_manager()
        cinfo.loadCameraInfo()
        self.assertFalse(cinfo.isCalibrated())

        # issue calibration service request
        exp = expected_calibration()
        resp = set_calibration(exp)
        self.assertTrue(cinfo.isCalibrated())

        # use assertTrue() instead of assertNotNone() so the test
        # works for Python versions before 2.7
        self.assertTrue(resp != None)
        self.assertTrue(resp.success)

        # create a new instance with default URL, checking that it has
        # the expected calibration
        cinfo2 = init_camera_info_manager()
        cinfo2.loadCameraInfo()
        self.assertTrue(cinfo2.isCalibrated())
        self.assertEqual(exp, cinfo2.getCameraInfo())

def run_tests():
    # run the tests in this thread
    import rosunit
    try:
        rosunit.unitrun(PKG, 'test_camera_info_manager',
                        TestCameraInfoManager)
    finally:
        rospy.signal_shutdown('test complete') # terminate the test node

if __name__ == '__main__':

    rospy.init_node("test_camera_info_manager")

    # create asynchronous thread for running the tests
    import threading
    test_th = threading.Thread(name='test_thread', target=run_tests)
    test_th.start()

    # spin in the main thread: required for service callbacks
    rospy.spin()
