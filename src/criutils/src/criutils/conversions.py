#! /usr/bin/env python
import rospy
import numpy as np
import baldor as br
# Messages
from geometry_msgs.msg import (Point, Quaternion, Pose, Vector3, Transform,
                                                                        Wrench)
from sensor_msgs.msg import RegionOfInterest


def from_dict(transform_dict):
  """
  Converts a dictionary with the fields `rotation` and `translation`
  into a homogeneous transformation.

  Parameters
  ----------
  transform_dict:  dict
    The dictionary to be converted

  Returns
  -------
  array: array_like
    The resulting numpy array
  """
  T = br.quaternion.to_transform(np.array(transform_dict['rotation']))
  T[:3,3] = np.array(transform_dict['translation'])
  return T

def from_point(msg):
  """
  Convert a `geometry_msgs/Point` ROS message into a numpy array.

  Parameters
  ----------
  msg: geometry_msgs/Point
    The ROS message to be converted

  Returns
  -------
  array: np.array
    The resulting numpy array
  """
  return from_vector3(msg)

def from_pose(msg):
  """
  Convert a `geometry_msgs/Pose` ROS message into a numpy array (4x4
  homogeneous transformation).

  Parameters
  ----------
  msg: geometry_msgs/Pose
    The ROS message to be converted

  Returns
  -------
  array: np.array
    The resulting numpy array
  """
  T = br.quaternion.to_transform(from_quaternion(msg.orientation))
  T[:3,3] = from_point(msg.position)
  return T

def from_quaternion(msg):
  """
  Convert a `geometry_msgs/Quaternion` ROS message into a numpy array.

  Parameters
  ----------
  msg: geometry_msgs/Quaternion
    The ROS message to be converted

  Returns
  -------
  array: np.array
    The resulting numpy array
  """
  return np.array([msg.w, msg.x, msg.y, msg.z])

def from_roi(msg):
  """
  Convert a `sensor_msgs/RegionOfInterest` ROS message into a list with the
  two corners of the ROI.

  Parameters
  ----------
  msg: sensor_msgs/RegionOfInterest
    The ROS message to be converted

  Returns
  -------
  result: list
    The resulting list
  """
  top_left = np.array([msg.x_offset, msg.y_offset])
  bottom_right = top_left + np.array([msg.width, msg.height])
  return [top_left, bottom_right]

def from_transform(msg):
  """
  Convert a `geometry_msgs/Transform` ROS message into a numpy array (4x4
  homogeneous transformation).

  Parameters
  ----------
  msg: geometry_msgs/Transform
    The ROS message to be converted

  Returns
  -------
  array: np.array
    The resulting numpy array
  """
  T = br.quaternion.to_transform(from_quaternion(msg.rotation))
  T[:3,3] = from_vector3(msg.translation)
  return T

def from_vector3(msg):
  """
  Convert a `geometry_msgs/Vector3` ROS message into a numpy array.

  Parameters
  ----------
  msg: geometry_msgs/Vector3
    The ROS message to be converted

  Returns
  -------
  array: np.array
    The resulting numpy array
  """
  return np.array([msg.x, msg.y, msg.z])

def from_wrench(msg):
  """
  Convert a `geometry_msgs/Wrench` ROS message into a numpy array.

  Parameters
  ----------
  msg: geometry_msgs/Wrench
    The ROS message to be converted

  Returns
  -------
  array: np.array
    The resulting numpy array
  """
  array = np.zeros(6)
  array[:3] = from_vector3(msg.force)
  array[3:] = from_vector3(msg.torque)
  return array

def to_quaternion(array):
  """
  Convert a numpy array WXYZ into a `geometry_msgs/Quaternion` ROS message.

  Parameters
  ----------
  array: np.array
    The quaternion XYZW as numpy array

  Returns
  -------
  msg: geometry_msgs/Quaternion
    The resulting ROS message
  """
  return Quaternion(array[1], array[2], array[3], array[0])

def to_point(array):
  """
  Convert a numpy array XYZ into a `geometry_msgs/Point` ROS message.

  Parameters
  ----------
  array: np.array
    The position XYZ as numpy array

  Returns
  -------
  msg: geometry_msgs/Point
    The resulting ROS message
  """
  return Point(*array)

def to_pose(T):
  """
  Convert a homogeneous transformation (4x4) into a `geometry_msgs/Pose` ROS
  message.

  Parameters
  ----------
  T: np.array
    The homogeneous transformation

  Returns
  -------
  msg: geometry_msgs/Pose
    The resulting ROS message
  """
  q = br.transform.to_quaternion(T)
  return Pose(to_point(T[:3,3]), to_quaternion(q))

def to_roi(top_left, bottom_right):
  """
  Generate a `sensor_msgs/RegionOfInterest` ROS message using the two given
  corners

  Parameters
  ----------
  top_left: array_like
    The top left corner of the ROI
  bottom_right: array_like
    The bottom right corner of the ROI

  Returns
  -------
  msg: sensor_msgs/RegionOfInterest
    The resulting ROS message
  """
  msg = RegionOfInterest()
  msg.x_offset = round(top_left[0])
  msg.y_offset = round(top_left[1])
  msg.width = round(abs(bottom_right[0]-top_left[0]))
  msg.height = round(abs(bottom_right[1]-top_left[1]))
  return msg

def to_transform(T):
  """
  Convert a homogeneous transformation (4x4) into a `geometry_msgs/Transform`
  ROS message.

  Parameters
  ----------
  T: np.array
    The homogeneous transformation

  Returns
  -------
  msg: geometry_msgs/Transform
    The resulting ROS message
  """
  translation = Vector3(*T[:3,3])
  q = br.transform.to_quaternion(T)
  rotation = to_quaternion(q)
  return Transform(translation, rotation)

def to_vector3(array):
  """
  Convert a numpy array XYZ into a `geometry_msgs/Vector3` ROS message.

  Parameters
  ----------
  array: array_like
    The vector XYZ as numpy array

  Returns
  -------
  msg: geometry_msgs/Vector3
    The resulting ROS message
  """
  return Vector3(*array)


def to_wrench(array):
  """
  Convert a numpy array into a `geometry_msgs/Wrench` ROS message.

  Parameters
  ----------
  array: array_like
    The wrench :math:`[f_x, f_y, f_z, t_x, t_y, t_z]` as numpy array

  Returns
  -------
  msg: geometry_msgs/Wrench
    The resulting ROS message
  """
  msg = Wrench()
  msg.force = to_vector3(array[:3])
  msg.torque = to_vector3(array[3:])
  return msg
