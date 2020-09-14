#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  @file RtmLineUtil.py
#  @brief rtc-link line management classes
#  @date $Date: 2005-05-12 09:06:19 $
#  @author Tsuyoshi Tanabe, Noriaki Ando <n-ando@aist.go.jp>
# 
#  Copyright (C) 2004-2005
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id$
# 

import sys
import string

BLANK = 25

class LineUtil:
  def __init__(self, parent, g_inp, g_outp, startx, starty, endx, endy):
    self.g_inp  = g_inp   # GRtcIn
    self.g_outp  = g_outp   # GRtcOut
    self.startx = startx
    self.starty = starty
    self.endx   = endx
    self.endy   = endy
    self.coordT = None
    self.parent = parent

  def drawLineP1(self) :
    #print "drawLineP1:"
    width = self.startx - self.endx
    hight = self.starty - self.endy
    inp = self.g_inp.getConfig('position')
    if inp == 'Left' or inp == 'Right' :
      if abs(width) <= abs(hight) :
        self.coordT = [(self.startx, self.starty),
                       (self.endx + width/2, self.starty),
                       (self.endx + width/2, self.endy),
                       (self.endx, self.endy)]
      else :
        self.coordT = [(self.startx, self.starty), (self.endx, self.endy)]
    if inp == 'Top' or inp == 'Bottom' :
      if abs(width) >= abs(hight) :
        self.coordT = [(self.startx, self.starty),
                       (self.startx, self.endy + hight/2),
                       (self.endx, self.endy + hight/2),
                       (self.endx, self.endy)]
      else :
        self.coordT = [(self.startx, self.starty), (self.endx, self.endy)]

  def drawLineP2(self, inpBox, outpBox) :
    #print "drawLineP2:"
    inp = self.g_inp.getConfig('position')
    outp = self.g_outp.getConfig('position')
    if inp == 'Left' and outp == 'Right' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx-BLANK, self.starty),
                     (self.startx-BLANK, outpBox[3]+((inpBox[1]-outpBox[3])/2)),
                     (self.endx+BLANK, outpBox[3]+((inpBox[1]-outpBox[3])/2)),
                     (self.endx+BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Right' and outp == 'Left' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx+BLANK, self.starty),
                     (self.startx+BLANK, outpBox[3]+((inpBox[1]-outpBox[3])/2)),
                     (self.endx-BLANK, outpBox[3]+((inpBox[1]-outpBox[3])/2)),
                     (self.endx-BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Top' and outp == 'Bottom' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty-BLANK),
                     (outpBox[2]+((inpBox[0]-outpBox[2])/2), self.starty-BLANK),
                     (outpBox[2]+((inpBox[0]-outpBox[2])/2), self.endy+BLANK),
                     (self.endx, self.endy+BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Bottom' and outp == 'Top' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty+BLANK),
                     (outpBox[2]+((inpBox[0]-outpBox[2])/2), self.starty+BLANK),
                     (outpBox[2]+((inpBox[0]-outpBox[2])/2), self.endy-BLANK),
                     (self.endx, self.endy-BLANK),
                     (self.endx, self.endy)]

  def drawLineP3(self, inpBox, outpBox) :
    #print "drawLineP3:"
    inp = self.g_inp.getConfig('position')
    outp = self.g_outp.getConfig('position')
    if inp == 'Left' and outp == 'Right' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx-BLANK, self.starty),
                     (self.startx-BLANK, inpBox[3]+((outpBox[1]-inpBox[3])/2)),
                     (self.endx+BLANK, inpBox[3]+((outpBox[1]-inpBox[3])/2)),
                     (self.endx+BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Right' and outp == 'Left' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx+BLANK, self.starty),
                     (self.startx+BLANK, inpBox[3]+((outpBox[1]-inpBox[3])/2)),
                     (self.endx-BLANK, inpBox[3]+((outpBox[1]-inpBox[3])/2)),
                     (self.endx-BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Top' and outp == 'Bottom' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty-BLANK),
                     (inpBox[2]+((outpBox[0]-inpBox[2])/2), self.starty-BLANK),
                     (inpBox[2]+((outpBox[0]-inpBox[2])/2), self.endy+BLANK),
                     (self.endx, self.endy+BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Bottom' and outp == 'Top' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty+BLANK),
                     (inpBox[2]+((outpBox[0]-inpBox[2])/2), self.starty+BLANK),
                     (inpBox[2]+((outpBox[0]-inpBox[2])/2), self.endy-BLANK),
                     (self.endx, self.endy-BLANK),
                     (self.endx, self.endy)]

  def drawLineP4(self, inpBox, outpBox) :
    #print "drawLineP4:"
    inp = self.g_inp.getConfig('position')
    outp = self.g_outp.getConfig('position')

    if inp == 'Left' or inp == 'Right' :
      if inpBox[3] >= outpBox[3] :
        hight = inpBox[3]+BLANK
      else :
        hight = outpBox[3]+BLANK
    elif inp == 'Top' or inp == 'Bottom' :
      if inpBox[2] >= outpBox[2] :
        width = inpBox[2]+BLANK
      else :
        width = outpBox[2]+BLANK  

    if inp == 'Left' and outp == 'Right' :  
      self.coordT = [(self.startx, self.starty),
                     (self.startx-BLANK, self.starty),
                     (self.startx-BLANK, hight),
                     (self.endx+BLANK, hight),
                     (self.endx+BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Right' and outp == 'Left' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx+BLANK, self.starty),
                     (self.startx+BLANK, hight),
                     (self.endx-BLANK, hight),
                     (self.endx-BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Top' and outp == 'Bottom' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty-BLANK),
                     (width, self.starty-BLANK),
                     (width, self.endy+BLANK),
                     (self.endx, self.endy+BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Bottom' and outp == 'Top' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty+BLANK),
                     (width, self.starty+BLANK),
                     (width, self.endy-BLANK),
                     (self.endx, self.endy-BLANK),
                     (self.endx, self.endy)]

  def drawLineP5(self, inpBox, outpBox) :    
    #print "drawLineP5:"
    inp = self.g_inp.getConfig('position')
    outp = self.g_outp.getConfig('position')
    if inp == 'Left' and outp == 'Left' :
      if self.endx - (self.startx-BLANK) < BLANK :
        pos_x = self.endx-BLANK
      else :
        pos_x = self.startx-BLANK
      self.coordT = [(self.startx, self.starty),
                     (pos_x, self.starty),
                     (pos_x, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Right' and outp == 'Right' :
      if (self.startx+BLANK) - self.endx < BLANK :
        pos_x = self.endx+BLANK
      else :
        pos_x = self.startx+BLANK  
      self.coordT = [(self.startx, self.starty),
                     (pos_x, self.starty),
                     (pos_x, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Top' and outp == 'Top' :
      if self.endy - (self.starty-BLANK) < BLANK :
        pos_y = self.endy-BLANK
      else :
        pos_y = self.starty-BLANK
      self.coordT = [(self.startx, self.starty),
                     (self.startx, pos_y),
                     (self.endx, pos_y),
                     (self.endx, self.endy)]
    elif inp == 'Bottom' and outp == 'Bottom' :
      if (self.starty+BLANK) - self.endy < BLANK :
        pos_y = self.endy+BLANK
      else :
        pos_y = self.starty+BLANK
      self.coordT = [(self.startx, self.starty),
                     (self.startx, pos_y),
                     (self.endx, pos_y),
                     (self.endx, self.endy)]

  def drawLineP6(self, inpBox, outpBox) :    
    #print "drawLineP6:"
    inp = self.g_inp.getConfig('position')
    outp = self.g_outp.getConfig('position')

    if inp == 'Left' or inp == 'Right' :
      if inpBox[3] >= outpBox[3] :
        hight = inpBox[3]+BLANK
      else :
        hight = outpBox[3]+BLANK
    elif inp == 'Top' or inp == 'Bottom' :
      if inpBox[2] >= outpBox[2] :
        width = inpBox[2]+BLANK
      else :
        width = outpBox[2]+BLANK  

    if inp == 'Left' and outp == 'Left' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx-BLANK, self.starty),
                     (self.startx-BLANK, hight),
                     (self.endx-BLANK, hight),
                     (self.endx-BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Right' and outp == 'Right' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx+BLANK, self.starty),
                     (self.startx+BLANK, hight),
                     (self.endx+BLANK, hight),
                     (self.endx+BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Top' and outp == 'Top' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty-BLANK),
                     (width, self.starty-BLANK),
                     (width, self.endy-BLANK),
                     (self.endx, self.endy-BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Bottom' and outp == 'Bottom' :    
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty+BLANK),
                     (width, self.starty+BLANK),
                     (width, self.endy+BLANK),
                     (self.endx, self.endy+BLANK),
                     (self.endx, self.endy)]

  def drawLineP7(self, inpBox, outpBox) :    
    #print "drawLineP7:"
    inp = self.g_inp.getConfig('position')
    outp = self.g_outp.getConfig('position')
    if (inp == 'Top' and (outp == 'Left' or outp == 'Right')) or \
       (inp == 'Bottom' and (outp == 'Left' or outp == 'Right')) :
      if (inp == 'Top' and self.starty-self.endy >= BLANK) or \
         (inp == 'Bottom' and self.endy-self.starty >= BLANK) :
        self.coordT = [(self.startx, self.starty),
                       (self.startx, self.endy),
                       (self.endx, self.endy)]
      else :
        self.drawLineP8(inpBox, outpBox)  
    elif (inp == 'Left' and (outp == 'Top' or outp == 'Bottom')) or \
         (inp == 'Right' and (outp == 'Top' or outp == 'Bottom')) :
      if (inp == 'Left' and self.startx-self.endx >= BLANK) or \
         (inp == 'Right' and self.endx-self.startx >= BLANK) : 
        self.coordT = [(self.startx, self.starty),
                       (self.endx, self.starty),
                       (self.endx, self.endy)]
      else :
        self.drawLineP8(inpBox, outpBox)    

  def drawLineP8(self, inpBox, outpBox) :    
    #print "drawLineP8:"
    inp = self.g_inp.getConfig('position')
    outp = self.g_outp.getConfig('position')
    if inp == 'Top' and outp == 'Left' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty-BLANK),
                     (self.endx-BLANK, self.starty-BLANK),
                     (self.endx-BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Top' and outp == 'Right' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty-BLANK),
                     (self.endx+BLANK, self.starty-BLANK),
                     (self.endx+BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Bottom' and outp == 'Left' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty+BLANK),
                     (self.endx-BLANK, self.starty+BLANK),
                     (self.endx-BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Bottom' and outp == 'Right' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty+BLANK),
                     (self.endx+BLANK, self.starty+BLANK),
                     (self.endx+BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Left' and outp == 'Top' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx-BLANK, self.starty),
                     (self.startx-BLANK, self.endy-BLANK),
                     (self.endx, self.endy-BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Left' and outp == 'Bottom' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx-BLANK, self.starty),
                     (self.startx-BLANK, self.endy+BLANK),
                     (self.endx, self.endy+BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Right' and outp == 'Top' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx+BLANK, self.starty),
                     (self.startx+BLANK, self.endy-BLANK),
                     (self.endx, self.endy-BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Right' and outp == 'Bottom' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx+BLANK, self.starty),
                     (self.startx+BLANK, self.endy+BLANK),
                     (self.endx, self.endy+BLANK),
                     (self.endx, self.endy)]

#    #print "test 1:"
    w1 = abs(self.coordT[2][0] - self.coordT[3][0])
    h1 = abs(self.coordT[2][1] - self.coordT[3][1])
    if w1 == 0:
      tmp = h1/20
      wadd = 0
      hadd = 20
    else:
      tmp = w1/20
      wadd = 20
      hadd = 0

    canvas = self.parent.parent.body.GetCanvas()

    cur = []
    for n in range(int(tmp)):
      shape = canvas.FindShape(self.coordT[2][0]+wadd*n,self.coordT[2][1]+hadd*n)
      if shape != 0:
          cur.append(shape[0])

    for obj in cur :
      tags = obj.parent.tag
      if 'body' in tags :
        self.drawLineP11(inpBox, outpBox)  
        break

  def drawLineP9(self, inpBox, outpBox) :    
    #print "drawLineP9:"
    inp = self.g_inp.getConfig('position')
    outp = self.g_outp.getConfig('position')
    
    if inp == 'Top' :
      if inpBox[1] <= outpBox[1] :
        hight = inpBox[1]-BLANK
      else :
        hight = outpBox[1]-BLANK
    elif inp == 'Bottom' :
      if inpBox[3] >= outpBox[3] :
        hight = inpBox[3]+BLANK 
      else :
        hight = outpBox[3]+BLANK
    elif inp == 'Left' :
      if inpBox[0] <= outpBox[0] :
        width = inpBox[0]-BLANK
      else :
        width = outpBox[0]-BLANK
    elif inp == 'Right' :
      if inpBox[2] >= outpBox[2] :
        width = inpBox[2]+BLANK
      else :
        width = outpBox[2]+BLANK
    
    if (inp == 'Top' or inp == 'Bottom') and outp == 'Right' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, hight),
                     (self.endx+BLANK, hight),
                     (self.endx+BLANK, self.endy),
                     (self.endx, self.endy)]
    elif (inp == 'Top' or inp == 'Bottom') and outp == 'Left' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, hight),
                     (self.endx-BLANK, hight),
                     (self.endx-BLANK, self.endy),
                     (self.endx, self.endy)]
    elif (inp == 'Left' or inp == 'Right') and outp == 'Top' :
      self.coordT = [(self.startx, self.starty),
                     (width, self.starty),
                     (width, self.endy-BLANK),
                     (self.endx, self.endy-BLANK),
                     (self.endx, self.endy)]
    elif (inp == 'Left' or inp == 'Right') and outp == 'Bottom' :
      self.coordT = [(self.startx, self.starty),
                     (width, self.starty),
                     (width, self.endy+BLANK),
                     (self.endx, self.endy+BLANK),
                     (self.endx, self.endy)]

#    #print "test 2:"
    w1 = abs(self.coordT[2][0] - self.coordT[3][0])
    h1 = abs(self.coordT[2][1] - self.coordT[3][1])
    if w1 == 0:
      tmp = h1/20
      wadd = 0
      hadd = 20
    else:
      tmp = w1/20
      wadd = 20
      hadd = 0

    canvas = self.parent.parent.body.GetCanvas()

    cur = []
    for n in range(int(tmp)):
      shape = canvas.FindShape(self.coordT[2][0]+wadd*n,self.coordT[2][1]+hadd*n)
      if shape != 0:
        cur.append(shape[0])

    for obj in cur :
      tags = obj.parent.tag
      if 'body' in tags :
        self.drawLineP11(inpBox, outpBox)  
        break
    
  def drawLineP10(self, inpBox, outpBox) :    
    #print "drawLineP10:"
    inp = self.g_inp.getConfig('position')
    outp = self.g_outp.getConfig('position')
    if inp == 'Top' and outp == 'Left' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty-BLANK),
                     (self.endx-BLANK, self.starty-BLANK),
                     (self.endx-BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Top' and outp == 'Right' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty-BLANK),
                     (self.endx+BLANK, self.starty-BLANK),
                     (self.endx+BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Bottom' and outp == 'Left' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty+BLANK),
                     (self.endx-BLANK, self.starty+BLANK),
                     (self.endx-BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Bottom' and outp == 'Right' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty+BLANK),
                     (self.endx+BLANK, self.starty+BLANK),
                     (self.endx+BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Left' and outp == 'Top' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx-BLANK, self.starty),
                     (self.startx-BLANK, self.endy-BLANK),
                     (self.endx, self.endy-BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Left' and outp == 'Bottom' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx-BLANK, self.starty),
                     (self.startx-BLANK, self.endy+BLANK),
                     (self.endx, self.endy+BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Right' and outp == 'Top' : 
      self.coordT = [(self.startx, self.starty),
                     (self.startx+BLANK, self.starty),
                     (self.startx+BLANK, self.endy-BLANK),
                     (self.endx, self.endy-BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Right' and outp == 'Bottom' : 
      self.coordT = [(self.startx, self.starty),
                     (self.startx+BLANK, self.starty),
                     (self.startx+BLANK, self.endy+BLANK),
                     (self.endx, self.endy+BLANK),
                     (self.endx, self.endy)]

  def drawLineP11(self, inpBox, outpBox) :    
    #print "drawLineP11:"
    inp = self.g_inp.getConfig('position')
    outp = self.g_outp.getConfig('position')
    if inp == 'Top' and outp == 'Right':
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty-BLANK),
                     (inpBox[2]+BLANK, self.starty-BLANK),
                     (inpBox[2]+BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Bottom' and outp == 'Right':
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty+BLANK),
                     (inpBox[2]+BLANK, self.starty+BLANK),
                     (inpBox[2]+BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Top' and outp == 'Left' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty-BLANK),
                     (inpBox[0]-BLANK, self.starty-BLANK),
                     (inpBox[0]-BLANK, self.endy),
                     (self.endx, self.endy)]
    elif inp == 'Bottom' and outp == 'Left' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx, self.starty+BLANK),
                     (inpBox[0]-BLANK, self.starty+BLANK),
                     (inpBox[0]-BLANK, self.endy),
                     (self.endx, self.endy)]  
    elif inp == 'Left' and outp == 'Bottom' :  
      self.coordT = [(self.startx, self.starty),
                     (self.startx-BLANK, self.starty),
                     (self.startx-BLANK, inpBox[3]+BLANK),
                     (self.endx, inpBox[3]+BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Right' and outp == 'Bottom' : 
      self.coordT = [(self.startx, self.starty),
                     (self.startx+BLANK, self.starty),
                     (self.startx+BLANK, inpBox[3]+BLANK),
                     (self.endx, inpBox[3]+BLANK),
                     (self.endx, self.endy)]  
    elif inp == 'Left' and outp == 'Top' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx-BLANK, self.starty),
                     (self.startx-BLANK, inpBox[0]-BLANK),
                     (self.endx, inpBox[0]-BLANK),
                     (self.endx, self.endy)]
    elif inp == 'Right' and outp == 'Top' :
      self.coordT = [(self.startx, self.starty),
                     (self.startx+BLANK, self.starty),
                     (self.startx+BLANK, inpBox[0]-BLANK),
                     (self.endx, inpBox[0]-BLANK),
                     (self.endx, self.endy)]

  def drawLinePetc(self) :
    self.coordT = [(self.startx, self.starty), (self.endx, self.endy)]
    
  #######################################################
  # drawing line
  #######################################################                  
  def drawLine(self) :
    inp = self.g_inp
    outp = self.g_outp
    if outp == None or inp == None:  
      self.drawLinePetc()  
    else :
      inpPos = inp.getConfig('position')
      outpPos = outp.getConfig('position')
      inpBox = [0,0,0,0]
      inpBox[2],inpBox[3] = inp.parent.baseBox.GetBoundingBoxMin()
#      inpBox[2],inpBox[3] = inp.parent.body.GetBoundingBoxMin()
      inpBox[0] = inp.parent.body.GetX() - inpBox[2]/2
      inpBox[1] = inp.parent.body.GetY() - inpBox[3]/2
      inpBox[2] = inpBox[0] + inpBox[2]
      inpBox[3] = inpBox[1] + inpBox[3]
      outpBox = [0,0,0,0]
      outpBox[2],outpBox[3] =  outp.parent.baseBox.GetBoundingBoxMin()
#      outpBox[2],outpBox[3] =  outp.parent.body.GetBoundingBoxMin()
      outpBox[0] = outp.parent.body.GetX() - outpBox[2]/2
      outpBox[1] = outp.parent.body.GetY() - outpBox[3]/2
      outpBox[2] = outpBox[0] + outpBox[2]
      outpBox[3] = outpBox[1] + outpBox[3]
      if (inpBox[2] < outpBox[0] and outpPos == 'Left' and inpPos == 'Right') or \
         (outpBox[3] < inpBox[1] and outpPos == 'Bottom' and inpPos == 'Top') or \
         (inpBox[0] > outpBox[2] and outpPos == 'Right' and inpPos == 'Left') or \
         (inpBox[3] < outpBox[1] and outpPos == 'Top' and inpPos == 'Bottom') : 
        self.drawLineP1()
      elif ((inpBox[1] > outpBox[3]) and ((outpPos == 'Right' and inpPos == 'Left') or \
           (outpPos == 'Left' and inpPos == 'Right'))) or \
           ((inpBox[0] > outpBox[2]) and ((outpPos == 'Bottom' and inpPos == 'Top') or \
           (outpPos == 'Top' and inpPos == 'Bottom'))) :
        self.drawLineP2(inpBox, outpBox)
      elif ((inpBox[3] < outpBox[1]) and ((outpPos == 'Right' and inpPos == 'Left') or \
           (outpPos == 'Left' and inpPos == 'Right'))) or \
           ((inpBox[2] < outpBox[0]) and ((outpPos == 'Bottom' and inpPos == 'Top') or \
           (outpPos == 'Top' and inpPos == 'Bottom'))) :
        self.drawLineP3(inpBox, outpBox)
      elif ((inpBox[2] < outpBox[0]) and (outpPos == 'Right' and inpPos == 'Left')) or \
           ((inpBox[0] > outpBox[2]) and (outpPos == 'Left' and inpPos == 'Right')) or \
           ((inpBox[3] < outpBox[1]) and (outpPos == 'Bottom' and inpPos == 'Top')) or \
           ((inpBox[1] > outpBox[3]) and (outpPos == 'Top' and inpPos == 'Bottom')) :
        self.drawLineP4(inpBox, outpBox)
      elif ((outpPos == 'Left' and inpPos == 'Left') or (outpPos == 'Right' and inpPos == 'Right')) and \
           ((inpBox[1] > outpBox[3]) or (inpBox[3] < outpBox[1])) :
        self.drawLineP5(inpBox, outpBox)
      elif ((outpPos == 'Top' and inpPos == 'Top') or (outpPos == 'Bottom' and inpPos == 'Bottom')) and \
           ((inpBox[2] < outpBox[0]) or (inpBox[0] > outpBox[2])) :
        self.drawLineP5(inpBox, outpBox)
      elif ((outpPos == 'Left' and inpPos == 'Left') or (outpPos == 'Right' and inpPos == 'Right')) and \
           ((inpBox[1] <= outpBox[3]) and (inpBox[3] >= outpBox[1])) :
        self.drawLineP6(inpBox, outpBox)
      elif ((outpPos == 'Top' and inpPos == 'Top') or (outpPos == 'Bottom' and inpPos == 'Bottom')) and \
           ((inpBox[2] >= outpBox[0]) and (inpBox[0] <= outpBox[2])) :
        self.drawLineP6(inpBox, outpBox)
      elif ((inpPos == 'Top' or inpPos == 'Bottom') and outpPos == 'Left' and inpBox[2] < outpBox[0]) or \
           ((inpPos == 'Top' or inpPos == 'Bottom') and outpPos == 'Right' and inpBox[0] > outpBox[2]) :
        self.drawLineP7(inpBox, outpBox)
      elif ((inpPos == 'Left' or inpPos == 'Right') and outpPos == 'Top' and inpBox[3] < outpBox[1]) or \
           ((inpPos == 'Left' or inpPos == 'Right') and outpPos == 'Bottom' and inpBox[1] > outpBox[3]) :
        self.drawLineP7(inpBox, outpBox)
      elif ((inpPos == 'Top' or inpPos == 'Bottom') and outpPos == 'Left' and inpBox[0] > outpBox[2]) or \
           ((inpPos == 'Top' or inpPos == 'Bottom') and outpPos == 'Right' and inpBox[2] < outpBox[0]) :
        self.drawLineP9(inpBox, outpBox)
      elif ((inpPos == 'Left' or inpPos == 'Right') and outpPos == 'Top' and inpBox[1] > outpBox[3]) or \
           ((inpPos == 'Left' or inpPos == 'Right') and outpPos == 'Bottom' and inpBox[3] < outpBox[1]) :
        self.drawLineP9(inpBox, outpBox)
      elif (inpPos == 'Left' and outpPos == 'Top' and inpBox[0] <= outpBox[0] and inpBox[3] >= outpBox[1]) or \
           (inpPos == 'Right' and outpPos == 'Top' and inpBox[2] >= outpBox[2] and inpBox[3] >= outpBox[1])or \
           (inpPos == 'Left' and outpPos == 'Bottom' and inpBox[0] <= outpBox[0] and inpBox[1] <= outpBox[3]) or \
           (inpPos == 'Right' and outpPos == 'Bottom' and inpBox[2] >= outpBox[2] and inpBox[1] <= outpBox[3]) :
        self.drawLineP9(inpBox, outpBox)
      elif (inpPos == 'Top' and outpPos == 'Left' and inpBox[1] <= outpBox[1] and inpBox[2] >= outpBox[0]) or \
           (inpPos == 'Bottom' and outpPos == 'Left' and inpBox[3] >= outpBox[3] and inpBox[2] >= outpBox[0]) or \
           (inpPos == 'Top' and outpPos == 'Right' and inpBox[1] <= outpBox[1] and inpBox[0] <= outpBox[2]) or \
           (inpPos == 'Bottom' and outpPos == 'Right' and inpBox[3] >= outpBox[3] and inpBox[0] <= outpBox[2]) :
        self.drawLineP9(inpBox, outpBox)
      elif (inpPos == 'Top' and (outpPos == 'Left' or outpPos == 'Right')) and (inpBox[0] <= outpBox[2] and inpBox[2] >= outpBox[0]) and \
           (inpBox[1] >= outpBox[3]) :   
        self.drawLineP10(inpBox, outpBox)
      elif (inpPos == 'Bottom' and (outpPos == 'Left' or outpPos == 'Right')) and (inpBox[0] <= outpBox[2] and inpBox[2] >= outpBox[0]) and \
           (inpBox[3] <= outpBox[1]) :   
        self.drawLineP10(inpBox, outpBox)
      elif (inpPos == 'Left' and (outpPos == 'Top' or outpPos == 'Bottom'))  and (inpBox[1] <= outpBox[3] and inpBox[3] >= outpBox[1]) and \
           (inpBox[0] >= outpBox[2]) :  
        self.drawLineP10(inpBox, outpBox)
      elif (inpPos == 'Right' and (outpPos == 'Top' or outpPos == 'Bottom'))  and (inpBox[1] <= outpBox[3] and inpBox[3] >= outpBox[1]) and \
           (inpBox[2] <= outpBox[0]) :
        self.drawLineP10(inpBox, outpBox)
      else : # at once, it joins inport and outport
        self.drawLinePetc()  
    #self.parent.draw.coords(self.body, self.coordT) # redraw line  
#    paraT = ()
#    paraT = (self.body,) + self.coordT
#    apply(self.parent.draw.coords, paraT) # redraw line ( for python v1.5 )
    return self.coordT

