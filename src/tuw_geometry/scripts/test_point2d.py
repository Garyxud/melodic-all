#!/usr/bin/env python
import tuw_geometry as tuw
import cv2

if __name__ == '__main__':
    
    pnt0 = tuw.Point2D()
    pnt1 = tuw.Point2D(3,2)
    pnt2 = tuw.Point2D(3,2,4)
    pose = tuw.Pose2D(3,2,4)
    pnt2.set(1,2,1)
    
    print (pnt1)
    print (pnt1)
    print pnt1.inside(0,0,4,4)
    print pnt1.inside(0,0,2,2)
    print (pnt2)
    print (pose)
    
    p = pose.position()
    print (p)
    print p[0]
    print p.x
    print pose.theta
    print pose.str("%lfm, %lfm, %lfrad")