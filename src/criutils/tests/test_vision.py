#! /usr/bin/env python
import unittest
import numpy as np
import criutils as cu


class Test_vision(unittest.TestCase):
  def test_draw_circles(self):
    image = np.zeros((512,512,3), np.uint8)
    circles = [(447,63,63)]                 # center_u, center_v, radius
    cu.vision.draw_circles(image, circles)
    self.assertTrue(np.sum(image) > 0)

  def test_(self):
    # Populate the stereo camera model
    from sensor_msgs.msg import CameraInfo
    from image_geometry import StereoCameraModel
    lmsg = CameraInfo()
    rmsg = CameraInfo()
    for m in (lmsg, rmsg):
      m.width = 640
      m.height = 480
    # Left camera info
    lmsg.D =  [-0.363528858080088, 0.16117037733986861, -8.1109585007538829e-05,
                                                  -0.00044776712298447841, 0.0]
    lmsg.K =  [430.15433020105519, 0.0, 311.71339830549732, 0.0,
                          430.60920415473657, 221.06824942698509, 0.0, 0.0, 1.0]
    lmsg.R =  [0.99806560714807102, 0.0068562422224214027, 0.061790256276695904,
            -0.0067522959054715113, 0.99997541519165112, -0.0018909025066874664,
              -0.061801701660692349, 0.0014700186639396652, 0.99808736527268516]
    lmsg.P =  [295.53402059708782, 0.0, 285.55760765075684, 0.0, 0.0,
                295.53402059708782, 223.29617881774902, 0.0, 0.0, 0.0, 1.0, 0.0]
    # Right
    rmsg.D =  [-0.3560641041112021, 0.15647260261553159,
                              -0.00016442960757099968, -0.00093175810713916221]
    rmsg.K =  [428.38163131344191, 0.0, 327.95553847249192, 0.0,
                          428.85728580588329, 217.54828640915309, 0.0, 0.0, 1.0]
    rmsg.R =  [0.9982082576219119, 0.0067433328293516528, 0.059454199832973849,
            -0.0068433268864187356, 0.99997549128605434, 0.0014784127772287513,
            -0.059442773257581252, -0.0018826283666309878, 0.99822993965212292]
    rmsg.P =  [295.53402059708782, 0.0, 285.55760765075684, -26.507895206214123,
          0.0, 295.53402059708782, 223.29617881774902, 0.0, 0.0, 0.0, 1.0, 0.0]
    cam = StereoCameraModel()
    cam.fromCameraInfo(lmsg, rmsg)
    # Test the function
    cu.vision.fix_stereo_camera_model(cam)
