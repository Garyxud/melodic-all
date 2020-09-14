#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-
#
# @brief LRFViewer component
# @date $Date$
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2008
#     Noriaki Ando
#     Task-intelligence Research Group,
#     Intelligent Systems Research Institute,
#     National Institute of
#         Advanced Industrial Science and Technology (AIST), Japan
#     All rights reserved.
#
# $Id$
#

# $Log$
#

from Tix import *
import time
import math

# Import RTM module
import RTC
import OpenRTM_aist
# This module's spesification
# <rtc-template block="module_spec">
lrfviewer_spec = ["implementation_id",     "LRFViewer", 
                  "type_name",         "LRFViewer", 
                  "description",       "Laser Range Finder Viewer component", 
                  "version",           "1.0", 
                  "vendor",            "Noriaki Ando, AIST", 
                  "category",          "example", 
                  "activity_type",     "DataFlowComponent", 
                  "max_instance",      "1", 
                  "language",          "Python", 
                  "lang_type",         "SCRIPT",
                  ""]
# </rtc-template>


#------------------------------------------------------------
# LRFViewer component
#
#------------------------------------------------------------
class LRFViewer(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

    self.range_data = []
    self.start_point = 0
    self.end_point   = 768
    self.angular_res = 0.0
    return

  def onInitialize(self):
    _pose3D = RTC.Pose3D(RTC.Point3D(0.0, 0.0, 0.0),
                         RTC.Orientation3D(0.0, 0.0, 0.0))
    _size3D = RTC.Size3D(0.0, 0.0, 0.0)
    _geometry3D = RTC.Geometry3D(_pose3D, _size3D)
    _rangerConfig = RTC.RangerConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    self._d_rangeData = RTC.RangeData(RTC.Time(0,0),
                                      [],
                                      RTC.RangerGeometry(_geometry3D, []),
                                      _rangerConfig)

    self._rangeDataIn = OpenRTM_aist.InPort("range_data", self._d_rangeData)
    self.addInPort("range_data",  self._rangeDataIn)

    return RTC.RTC_OK

  def onShutdown(self, ec_id):
    return RTC.RTC_OK

  def onDeactivated(self, ec_id):
    self.range_data = []
    self.start_point = 0
    self.end_point = 768
    return RTC.RTC_OK

  def onExecute(self, ec_id):
    if self._rangeDataIn.isNew():
      _rangeData = self._rangeDataIn.read()
      self.range_data = _rangeData.ranges
      self.start_point = _rangeData.config.minAngle
      self.end_point = _rangeData.config.maxAngle
      self.angular_res = _rangeData.config.angularRes
    time.sleep(0.01)
    return RTC.RTC_OK

  def get_range_data(self):
    return self.range_data

  def get_start_point(self):
    return self.start_point

  def get_end_point(self):
    return self.end_point

  def get_angular_res(self):
    return self.angular_res


class ToggleItem:
  def __init__(self):
    self.active = True
    return

  def __del__(self):
    self.delete()
    return

  def activate(self):
    self.active = True
    self.draw()
    return

  def deactivate(self):
    self.active = False
    self.delete()
    return

  def toggle(self):
    if self.active:
      self.deactivate()
    else:
      self.activate()
    return


class CanvasText(ToggleItem):
  def __init__(self, canvas, text, x, y):
    ToggleItem.__init__(self)
    self.canvas = canvas
    self.id = self.canvas.create_text(x, y, text=text)
    self.text = text
    self.x = x
    self.y = y
    self.draw_text(x, y, text)
    return

  def draw(self):
    if self.active == False: return
    self.delete()
    self.id = self.canvas.create_text(self.x, self.y, text=self.text)
    return

  def draw_text(self, x, y, text):
    self.x = x
    self.y = y
    self.text = text
    self.draw()
    return

  def delete(self):
    self.canvas.delete(self.id)
    return

class CanvasGrid(ToggleItem):
  def __init__(self, canvas, x0, y0, width, height, pitch, color, linewd):
    ToggleItem.__init__(self)
    self.canvas = canvas
    self.x0 = x0
    self.y0 = y0
    self.width = width
    self.height = height
    self.pitch = pitch
    self.color = color
    self.linewd = linewd
    self.idx = []
    self.idy = []

    self.draw()
    return

  def draw(self):
    if self.active == False: return
    self.delete()
    
    x_start = int(self.x0 % self.pitch)
    x_num   = int((self.width - x_start) / self.pitch) + 1
    for x in range(x_num):
      x0 = x_start + self.pitch * x
      id = self.canvas.create_line(x0, 0, x0, self.height,
                                   fill=self.color, width=self.linewd)
      self.idx.append(id)

    y_start = int(self.y0 % self.pitch)
    y_num   = int((self.height - y_start) / self.pitch) + 1
    for y in range(y_num):
      y0 = y_start + self.pitch * y
      id = self.canvas.create_line(0, y0, self.width, y0,
                                   fill=self.color, width=self.linewd)
      self.idy.append(id)

    for i in self.idx:
      self.canvas.tag_lower(i)
    for i in self.idy:
      self.canvas.tag_lower(i)

    return

  def delete(self):
    for i in self.idx:
      self.canvas.delete(i)
    for i in self.idy:
      self.canvas.delete(i)
    return

  def set_pitch(self, pitch):
    if pitch != 0:
      self.pitch = pitch

    self.draw()
    return


class CanvasAxis(ToggleItem):
  def __init__(self, canvas, width, height, color="#ffffff", linewd=1):
    ToggleItem.__init__(self)
    self.x0 = width/2
    self.y0 = height/2
    self.width = width
    self.height = height
    self.color = color
    self.linewd = linewd
    self.canvas = canvas
    self.id = [None] * 4
    self.draw()
    return

  def draw(self):
    if self.active == False: return
    self.delete()
    self.id[0] = self.canvas.create_line(0, self.height/2, 
                                         self.width, self.height/2,
                                         fill = self.color,
                                         width = self.linewd)
    self.id[1] = self.canvas.create_text(self.width - 10,
                                         self.height/2 + 10,
                                         text="x",
                                         fill = self.color,
                                         font="courier 12")
    self.id[2] = self.canvas.create_line(self.width/2, 0, 
                                         self.width/2, self.height,
                                         fill = self.color,
                                         width = self.linewd)
    self.id[3] = self.canvas.create_text(self.width/2 + 10,
                                         + 10, text="y",
                                         fill = self.color,
                                         font="courier 12")
        
    return

  def delete(self):
    for i in self.id:
      self.canvas.delete(i)
    return


class ScaledObject:
  def __init__(self, simulator):
    self.simulator = simulator
    self.tick = simulator.get_tick()
    self.canvas = simulator.get_canvas()
    self.trans = simulator.get_translation()
    return

  def translate(self, x, y, dx, dy, dth):
    return self.trans(x, y, dx, dy, dth)

  def get_tick(self):
    return self.simulator.get_tick()


#------------------------------------------------------------
# LRFrange range data drawing
#
#------------------------------------------------------------
class LRFrange(ScaledObject):
  def __init__(self, simulator,
               line_color="#ff0000", fill_color="#ff0000", linewd=1):
    ScaledObject.__init__(self, simulator)
    self.fill_color = fill_color
    self.line_color = line_color
    self.default_fill_color = fill_color
    self.default_line_color = line_color
    self.linewd = linewd
    self.rdata = []
    self.pre_data = []
    self.poly_id = None
    self.source = None

    # URG parameter
    self.beg_angle = -45
    self.end_angle = 225
    self.angle_per_step = 360.0 / 1024.0
    self.valid_beg_angle = 44 * 360.0 / 1024.0
    self.valid_end_angle = self.valid_beg_angle + 725 * self.angle_per_step
    self.offset_step = 0

    self.threshold = 0.0
    self.sfilter = 0.0
    self.tfilter = 0.0

    self.threshold_check = BooleanVar()
    self.threshold_check.set(True)
    self.tfilter_check = BooleanVar()
    self.tfilter_check.set(True)
    self.sfilter_check = BooleanVar()
    self.sfilter_check.set(True)

    self.threshold_var = DoubleVar()
    self.threshold_var.set(self.threshold)
    self.tfilter_var = DoubleVar()
    self.tfilter_var.set(self.tfilter)
    self.sfilter_var = DoubleVar()
    self.sfilter_var.set(self.sfilter)

    self.update()
    return

  def create_ctrl(self, frame):
    self.lrf_fill_check = StringVar()
    self.lrf_fill_check.set("on")
    self.lrf_line_check = StringVar()
    self.lrf_line_check.set("on")

    text = Label(frame, text="LRF range area", anchor=W, justify=LEFT)
    # "Line" check box
    line = Checkbutton(frame, text="Line",
                       onvalue="on", offvalue="off",
                       justify=LEFT, anchor=W,
                       variable=self.lrf_line_check,
                       command=self.line_toggle)
    # "Fill" check box
    fill = Checkbutton(frame, text="Fill",
                       onvalue="on", offvalue="off",
                       justify=LEFT, anchor=W,
                       variable=self.lrf_fill_check,
                       command=self.fill_toggle)
    # Threshold (check box/scale)
    thresh = Checkbutton(frame, text="Threshold",
                         onvalue=True, offvalue=False,
                         justify=LEFT, anchor=W,
                         variable=self.threshold_check)
    thresh_scale = Scale(frame, from_=0, to=100, resolution=0.1,
                         label="Threshold", command=self.on_threshold,
                         variable=self.threshold_var, orient=HORIZONTAL)
    # Time-Filter (check box/scale)
    tfilter = Checkbutton(frame, text="Filter(Time)",
                          onvalue=True, offvalue=False,
                          justify=LEFT, anchor=W,
                          variable=self.tfilter_check)
    tfilter_scale = Scale(frame, from_=0, to=1, resolution=0.01,
                          label="Filter", command=self.on_tfilter,
                          variable=self.tfilter_var, orient=HORIZONTAL)
    # Spacial-Filter (check box/scale)
    sfilter = Checkbutton(frame, text="Filter(Spacial)",
                          onvalue=True, offvalue=False,
                          justify=LEFT, anchor=W,
                          variable=self.sfilter_check)
    sfilter_scale = Scale(frame, from_=0, to=1, resolution=0.01,
                          label="Filter", command=self.on_sfilter,
                          variable=self.sfilter_var, orient=HORIZONTAL)

    for w in [text, line, fill, thresh, thresh_scale,
              tfilter, tfilter_scale, sfilter, sfilter_scale]:
      w.pack(side=TOP, anchor=W, fill=X) 
    return

  def on_threshold(self, var):
    self.threshold = self.threshold_var.get()
    return

  def on_sfilter(self, var):
    self.sfilter = self.sfilter_var.get()
    return

  def on_tfilter(self, var):
    self.tfilter = self.tfilter_var.get()
    return

  def line_toggle(self):
    if self.lrf_line_check.get() == "on":
      self.line_color = self.default_line_color
    else:
      self.line_color = ""
    return

  def fill_toggle(self):
    if self.lrf_fill_check.get() == "on":
      self.fill_color = self.default_fill_color
    else:
      self.fill_color = ""
    return

  def set_data_source(self, source):
    self.source = source
    return

  def set_value(self, data):
    self.rdata = data
    return

  def draw(self):
    self.delete()
    rpos = []
    rpos.append(self.translate(0, 0, 0, 0, 0))
    rpos.append(self.range_to_pos(self.rdata))
    self.poly_id = self.canvas.create_polygon(rpos,
                                              width = self.linewd,
                                              outline = self.line_color,
                                              fill = self.fill_color,
                                              smooth = 1,
                                              splinesteps = 5)
    return

  def range_to_pos(self, data):
    pos = []
    pre_d = 0
    
    tfilter = self.tfilter_check.get()
    sfilter = self.sfilter_check.get()
    thresh  = self.threshold_check.get()

    # Time-Filter
    if tfilter and len(data) == len(self.pre_data):
      for (n, d) in enumerate(data):
        k_t = self.tfilter
        data[n] = self.pre_data[n] * k_t + d * (1 - k_t)

    # Spacial Filter
    for (n, d) in enumerate(data):
      # Threshold
      if thresh and d < self.threshold:
        d = 10000 #pre_d

      if sfilter:
        k_s = self.sfilter
        d = pre_d * k_s + d * (1 - k_s)
      pre_d = d
            
      # n: step number
      # d: length data
      #deg = (n + self.offset_step) * self.angle_per_step + self.beg_angle
      #th = deg * math.pi / 180
      th = (n + self.offset_step) * self.angle_per_step + self.beg_angle
      x = d * math.cos(th)
      y = d * math.sin(th)
      pos.append(self.translate(x, y, 0, 0, 0))
    self.pre_data = data
    return pos

  def delete(self):
    if self.poly_id != None:
      self.canvas.delete(self.poly_id)
    return

  def update(self):
    if self.source != None:
      rdata = self.source.get_range_data()
      if len(rdata) != 0:
        self.rdata = rdata

      res = self.source.get_angular_res()
      if res:
        self.angle_per_step = res

      beg_angle = self.source.get_start_point()
      if beg_angle:
        self.beg_angle = beg_angle

      end_angle = self.source.get_end_point()
      if end_angle:
        self.end_angle = end_angle

    else:
      pass
    self.draw()



#------------------------------------------------------------
# TkLRFViewer main windows class
#
#------------------------------------------------------------
class TkLRFViewer(Frame):
  def __init__(self, master=None, width=480, height=480):
    Frame.__init__(self, master)

    # canvas properties
    self.width = width
    self.height = height
    # zero of canvas
    self.x0 = width/2
    self.y0 = height/2

    self.wd = 150

    self.robots = {}

    self.robot = None
    self.postext = None

    self.scale = 1.0
    self.scale_var = DoubleVar()
    self.scale_var.set(self.scale)

    self.grid_pitch = 50

    self.tick = 0.1
    self.default_tick = 0.1
    self.tickscale_var = DoubleVar()
    self.tickscale_var.set(self.tick)

    self.axis_check = StringVar()
    self.axis_check.set("on")
    self.grid_check = StringVar()
    self.grid_check.set("on")
    self.rnames = {}


    self.init()
    self.pack()


    self.after(20, self.on_update)
    return

  def init(self):
    self.canvas = Canvas(self, bg="#000000",
                         width = self.width, height = self.height)
    self.canvas.pack(side=LEFT)

    self.can_grid = CanvasGrid(self.canvas, self.x0, self.y0,
                               self.width, self.height, self.grid_pitch,
                               "#aaaaaa", 1)
    self.can_axis = CanvasAxis(self.canvas, self.width, self.height,
                               "#ffffff", 1)

    self.frame = Frame(self)
    self.frame.pack(side=LEFT)

    # Screen control
    self.scrctrl_frame = Frame(self.frame, width=self.wd, height=300,
                               relief=GROOVE, bd=2)
    self.scrctrl_frame.pack(side=TOP, fill=X)
    self.create_scale(self.scrctrl_frame)
    self.create_checkbutton(self.scrctrl_frame)


    self.lrfctrl_frame = Frame(self.frame, width=self.wd, height=300,
                               relief=GROOVE, bd=2)
    self.lrfctrl_frame.pack(side=TOP, fill=X)
    self.lrf = LRFrange(self)
    self.lrf.create_ctrl(self.lrfctrl_frame)

    return
      


  def on_update(self):
    self.lrf.update()
    self.after(20, self.on_update)
    return

  def get_tick(self):
    return self.tick
  
  def get_canvas(self):
    return self.canvas

  def get_translation(self):
    return self.real_to_canvas


  #------------------------------------------------------------
  # Scale control set
  def create_scale(self, frame):
    dummy = Frame(frame, width=self.wd)
    dummy.pack(side=TOP)
    sl = Scale(frame, from_=0, to=10, resolution=0.01,
               label="Scale Factor", command=self.on_scale,
               variable=self.scale_var, orient=HORIZONTAL)
    bt = Button(frame, text="Reset Scale", command=self.reset_scale)
    sl.pack(side=TOP, fill=X)
    bt.pack(side=TOP, fill=X)
    return

  def on_scale(self, val):
    v =  float(val)
    if v == 0.0:
      pitch = 0
    else:
      pitch = self.grid_pitch/v
      self.scale = v
    self.can_grid.set_pitch(pitch)
    return
        
  def reset_scale(self):
    self.scale_var.set(1.)
    pitch = self.grid_pitch/1.0
    self.scale = 1.0
    self.can_grid.set_pitch(pitch)
    return

  def on_tickchange(self, val):
    v =  self.tickscale_var.get()
    if v == 0.0:
      self.tick = 0
    else:
      self.tick = v
    return
        
  def reset_tickscale(self):
    self.tick = self.default_tick
    self.tickscale_var.set(self.default_tick)

  # end of Scale widget set
  #------------------------------------------------------------

  #------------------------------------------------------------
  # Canvas control set
  def create_checkbutton(self, frame):
    axis = Checkbutton(frame, text="Axis",
                       onvalue="on", offvalue="off",
                       justify=LEFT, anchor=W,
                       variable=self.axis_check,
                       command=self.can_axis.toggle)
    grid = Checkbutton(frame, text="Grid",
                       onvalue="on", offvalue="off",
                       justify=LEFT, anchor=W,
                       variable=self.grid_check,
                       command=self.can_grid.toggle)
    for w in [axis, grid]:
      w.pack(side=TOP, anchor=W, fill=X)

  def on_rname_toggle(self):
    for r in self.rnames.keys():
      self.rnames[r].toggle()
    return

  # end of Canvas control set
  #------------------------------------------------------------

  #------------------------------------------------------------
  # 
  def real_to_canvas(self, x, y, dx, dy, dt):
    # Simulator coordinate system -> display coordinate system
    # x, y: original position
    # dx, dy, dt: translation and rotation vector
    # translation and rotation
    x_tmp = (math.cos(dt) * x - math.sin(dt) * y + dx)/self.scale
    y_tmp = (math.sin(dt) * x + math.cos(dt) * y + dy)/self.scale
    # align to canvas coordinate system (origin is center and y+ is upward)
    xo =  x_tmp  + self.x0
    yo = -y_tmp + self.y0
    return xo, yo

import threading

class test_data_creator(threading.Thread):
  def __init__(self, lrf, step = 681):
    threading.Thread.__init__(self)
    import time
    self.lrf = lrf
    self.step = step
    self.flag = True
    return

  def stop(self):
    self.flag = False
    return

  def run(self):
    import random
    data = [0] * 681
    pre = 0
    while self.flag:
      for i in range(681):
        if i % 5 == 0:
          data[i] = pre * 0.9 + random.randint(0, 255) * 0.1
          pre = data[i]
        else:
          data[i] = pre
          pre = data[i]
      self.lrf.set_value(data)
      time.sleep(0.1)
    
    return

def main():
  m = TkLRFViewer(Tk())
  m.master.title("Laser Range Finder Viewer")

  mgr = OpenRTM_aist.Manager.init(sys.argv)
  mgr.activateManager()
  profile = OpenRTM_aist.Properties(defaults_str=lrfviewer_spec)
  mgr.registerFactory(profile, LRFViewer, OpenRTM_aist.Delete)
  mgr.runManager(True)
  lrf_rtc = mgr.createComponent("LRFViewer")
  m.lrf.set_data_source(lrf_rtc)
  m.mainloop()
  mgr.shutdown()

    
if  __name__ == '__main__': main()
