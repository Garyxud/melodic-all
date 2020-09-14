#! /usr/bin/env python
import cv2
import rospy
import numpy as np


BLUE      = (255,0,0)
GREEN     = (0,255,0)
RED       = (0,0,255)
MAGENTA   = (255,0,255)
YELLOW    = (0,255,255)
ORANGE    = (0,140,255)
WHITE     = (255,255,255)


def draw_circles(image, circles, color=(0,255,0), thickness=1):
  """
  Draw simple or filled circles with a given center and radius.

  Parameters
  ----------
  image: array_like
    Image where the circles are drawn
  circles: list
    List with the circle information. Each item must be
                                                  `(center_u, center_v, radius)`
  color: tuple
    Circles color
  thickness: float
    Thickness of the circle outline. Negative thickness means that a filled
    circle is to be drawn.
  """
  for circle in np.uint16(np.around(circles)):
    center = tuple(circle[:2])
    radius = circle[2]
    cv2.circle(image, center, radius, color=color, thickness=thickness)

def fix_stereo_camera_model(model):
  """
  Fix the computation of the reprojection matrix `Q` in the
  `StereoCameraModel <https://github.com/ros-perception/vision_opencv/blob/kinetic/image_geometry/src/image_geometry/cameramodels.py#L257>`_
  class.
  """
  Tx = model.right.P[0,3]
  fx = model.right.P[0,0]
  B = (-Tx / fx)
  lCx = model.left.P[0,2]
  lCy = model.left.P[1,2]
  rCx = model.right.P[0,2]
  # Q is:
  #    [ 1, 0,  0,  -lCx        ]
  #    [ 0, 1,  0,  -lCy        ]
  #    [ 0, 0,  0,  Fx          ]
  #    [ 0, 0, 1/B, (rCx-lCx)/B ]
  model.Q = np.zeros((4,4), dtype=np.float64)
  model.Q[0,0] = 1.0
  model.Q[0,3] = -lCx
  model.Q[1,1] = 1.0
  model.Q[1,3] = -lCy
  model.Q[2,3] = fx
  model.Q[3,2] = 1./B
  model.Q[3,3] = (rCx-lCx)/B
