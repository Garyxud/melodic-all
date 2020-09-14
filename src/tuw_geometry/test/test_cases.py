#!/usr/bin/env python
import unittest
import tuw_geometry as tuw

class TestPoint2D(unittest.TestCase):

    def runTest(self):
        p = tuw.Point2D(3,2)
        self.assertEqual(p.x, 3, 'incorrect x')
        self.assertEqual(p.y, 2, 'incorrect y')
        self.assertEqual(p.h, 1, 'incorrect h')
        p = tuw.Point2D(3,2,3)
        self.assertEqual(p.x, 3, 'incorrect x')
        self.assertEqual(p.y, 2, 'incorrect y')
        self.assertEqual(p.h, 3, 'incorrect h')
        
class TestPose2D(unittest.TestCase):

    def runTest(self):
        pose = tuw.Pose2D(3,2,0)
        p0 = tuw.Point2D(0,0)
        
        self.assertEqual(pose.x, 3, 'incorrect x')
        self.assertEqual(pose.y, 2, 'incorrect y')
        # self.assertEqual(pose.h, 1, 'incorrect h')

        # do some things to my_var which might change its value...

        # self.assertTrue(my_var)

if __name__ == '__main__':
    unittest.main()
