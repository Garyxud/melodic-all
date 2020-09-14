#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

# @brief 2D mobile robot on tk canvas
# @date $Date$
# @author Norkai Ando <n-ando@aist.go.jp>
#
# Copyright (C) 2007
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

#from Tkinter import *
from Tix import *
import time
import math

# Import RTM module
import RTC
import OpenRTM_aist
# This module's spesification
# <rtc-template block="module_spec">
tkmobilerobotsimulator_spec = ["implementation_id", "TkMobileRobotSimulator", 
                               "type_name",         "TkMobileRobotSimulator", 
                               "description",       "sample component for Python and Tkinter", 
                               "version",           "1.0", 
                               "vendor",            "Noriaki Ando, AIST", 
                               "category",          "example", 
                               "activity_type",     "DataFlowComponent", 
                               "max_instance",      "10", 
                               "language",          "Python", 
                               "lang_type",         "SCRIPT",
                               ""]
# </rtc-template>


class TkMobileRobotSimulator(OpenRTM_aist.DataFlowComponentBase):
  def __init__(self, manager):
    OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        
    self.pos = []
    self.vel = []
    return

  def onInitialize(self):
    self._d_vel = RTC.TimedFloatSeq(RTC.Time(0,0),[])
    self._velIn = OpenRTM_aist.InPort("vel", self._d_vel)
        
    self._d_pos = RTC.TimedFloatSeq(RTC.Time(0,0),[])
    self._posOut = OpenRTM_aist.OutPort("pos", self._d_pos)
        
    # Set InPort buffers
    self.addInPort("vel",self._velIn)
    self.addOutPort("pos",self._posOut)

    # Bind variables and configuration variable
    return RTC.RTC_OK

  def onShutdown(self, ec_id):
    return RTC.RTC_OK

  def onDeactivated(self, ec_id):
    self.pos = []
    self.vel = []
    return RTC.RTC_OK

  def onExecute(self, ec_id):
    if self._velIn.isNew():
      self.vel = self._velIn.read().data
    self._d_pos.data = self.pos
    self._posOut.write()
    time.sleep(0.01)
    return RTC.RTC_OK

  def get_velocity(self):
    return self.vel

  def set_position(self, pos):
    self.pos = pos


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

class RobotTitle(ToggleItem):
  def __init__(self, simulator, robot, name, r):
    ToggleItem.__init__(self)
    self.canvas = simulator.get_canvas()
    self.trans = simulator.get_translation()
    self.robot = robot
    self.name = name
    self.r = r
    self.id_circle = None
    self.id_line0 = None
    self.id_line1 = None
    self.id_name = None
    self.id_pos = None
    self.llength = 50
    return

  def draw(self):
    if self.active == False: return
    self.delete()
    rx, ry, rt = self.robot.get_pos()
        
    tmp_x0 = - self.r
    tmp_y0 = - self.r
    tmp_x1 =   self.r
    tmp_y1 =   self.r
    x0, y0 = self.trans(tmp_x0, tmp_y0, rx, ry, 0)
    x1, y1 = self.trans(tmp_x1, tmp_y1, rx, ry, 0)

    self.id_circle = self.canvas.create_oval(x0, y0, x1, y1,
                                             width=2,
                                             fill="", outline="#aaaaaa")
    r =  (x1 - x0)/2
    xo = x0 + r
    yo = y0 - r
    
    lx0 = xo + (r / math.sqrt(2))
    ly0 = yo - (r / math.sqrt(2))
    lx1 = lx0 + self.llength
    ly1 = ly0 - self.llength

    self.id_line0 = self.canvas.create_line(lx0, ly0, lx1, ly1,
                                            fill="#777777")
    self.id_line1 = self.canvas.create_line(lx1, ly1, lx1 + 120, ly1,
                                            fill="#777777")
    self.id_name = self.canvas.create_text(lx1+120, ly1-8,
                                           anchor=E, text=self.name)
    pos_text = '(%5.2f, %5.2f, %5.2f)' % (rx, ry, (rt*180/math.pi)%360)
    self.id_pos = self.canvas.create_text(lx1+120, ly1+8,
                                          anchor=E, text=pos_text)
    return


  def delete(self):
    if self.id_circle != None:
      self.canvas.delete(self.id_circle)
    if self.id_line0 != None:
      self.canvas.delete(self.id_line0)
    if self.id_line1 != None:
      self.canvas.delete(self.id_line1)
    if self.id_name != None:
      self.canvas.delete(self.id_name)
    if self.id_pos != None:
      self.canvas.delete(self.id_pos)
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
    self.pitch = pitch
    self.draw()
    return


class CanvasAxis(ToggleItem):
  def __init__(self, canvas, width, height):
    ToggleItem.__init__(self)
    self.x0 = width/2
    self.y0 = height/2
    self.width = width
    self.height = height
    self.canvas = canvas
    self.id = [None] * 4
    self.draw()
    return


  def draw(self):
    if self.active == False: return
    self.delete()
    self.id[0] = self.canvas.create_line(0, self.height/2, 
                                         self.width, self.height/2)
    self.id[1] = self.canvas.create_text(self.width - 10,
                                         self.height/2 + 10,
                                         text="x")
    self.id[2] = self.canvas.create_line(self.width/2, 0, 
                                         self.width/2, self.height)
    self.id[3] = self.canvas.create_text(self.width/2 + 10,
                                         + 10, text="y")
    
    return

  def delete(self):
    for i in self.id:
      self.canvas.delete(i)
    return



class SimulatedObject:
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


import tkSimpleDialog

class PropertyDialog:
  def __init__(self):
    # robot's profile
    self.name = ""
    self.type = ""
    self.description = ""
    self.vendor = ""
    # robot's parameter/input/output
    self.param = []
    self.input = []
    self.output = []
    # max length of label text
    self.label_len = 0
    # max length of unit text
    self.unit_len = 0

    self.apply_param  = None
    self.apply_input  = None
    self.reset_output = None
    return

  def set_profile(self, name, type, description, vendor):
    self.name = name
    self.type = type
    self.description = description
    self.vendor =vendor
    return

  def append_parameter(self, label, variable, unit):
    self.param.append({"label":label, "var":variable, "unit":unit})
    self.label_len = max(len(label), self.label_len)
    self.unit_len  = max(len(unit), self.unit_len)
    return

  def append_input(self, label, variable, unit):
    self.input.append({"label":label, "var":variable, "unit":unit})
    self.label_len = max(len(label), self.label_len)
    self.unit_len  = max(len(unit), self.unit_len)
    return

  def append_output(self, label, variable, unit):
    self.output.append({"label":label, "var":variable, "unit":unit})
    self.label_len = max(len(label), self.label_len)
    self.unit_len  = max(len(unit), self.unit_len)
    return

  def set_apply_param(self, func):
    self.apply_param = func
    return

  def set_apply_input(self, func):
    self.apply_input = func
    return

  def set_reset_output(self, func):
    self.reset_output = func
    return

  def pack(self):
    f = Toplevel()
    self.toplevel = f
    f.title(self.name)
    w0 = LabelFrame(f, label="Robot's Profile",
                    options="frame.anchor w frame.justify left")
    prof_frame = w0.subwidget('frame')
    self.profile_label(prof_frame)

    w1 = LabelFrame(f, label="Robot's Parameters")
    param_frame = w1.subwidget('frame')
    self.label_entries(param_frame, self.param)
    if self.apply_param != None:
      self.button(param_frame, "Apply", self.apply_param)

    w2 = LabelFrame(f, label="Robot's Input Values")
    input_frame = w2.subwidget('frame')
    self.label_entries(input_frame, self.input)
    if self.apply_input != None:
      self.button(input_frame, "Set", self.apply_input)

    w3 = LabelFrame(f, label="Robot's Output Values")
    output_frame = w3.subwidget('frame')
    self.label_entries(output_frame, self.output)
    if self.reset_output != None:
      self.button(output_frame, "Reset", self.reset_output)


    for w in [w0, w1, w2, w3]:
      w.pack(side=TOP, anchor=W, fill=X)
    self.button(f, "OK", self.on_ok)

    return

  def on_ok(self):
    self.toplevel.destroy()
    return


  def button(self, master, label, func):
    bt = Button(master, text=label, command=func, width=10,
                padx=3, pady=3)
    bt.pack(side=TOP, padx=5, pady=5)
    return


  def profile_label(self, master):
    t = ["Robot's name: ", "Robot's type: ", "Description: ", "Vendor: "]
    for i in range(len(t)):
      Label(master, text=t[i], anchor=W).grid(row=i, sticky=W,
                                              padx=3, pady=3)
    l = [self.name, self.type, self.description, self.vendor]
    for i in range(len(l)):
      Label(master, text=l[i], anchor=W).grid(row=i, column=1, sticky=W,
                                              padx=3, pady=3)
    return

        
  def label_entry(self, master, label0, var, label1):
    f = Frame(master)
    l0 = Label(f, text=label0, width=self.label_len, justify=LEFT, anchor=W)
    e = Entry(f, width=7, textvariable=var,
              justify=RIGHT, relief=GROOVE, bd=2)
    l1 = Label(f, text=label1, width=self.unit_len, justify=LEFT, anchor=W)
    for w in [l0, e, l1]:
      w.pack(side=LEFT, anchor=W, padx=3, pady=3)
    return f

  def label_entries(self, f, props):
    for p in props:
      self.label_entry(f, p["label"], p["var"], p["unit"]).pack(side=TOP)
    return



class DiffMobileModel:
  def __init__(self, radius, wheeld, pos = (0, 0, math.pi/2), dt=0.1):
    self.radius = radius
    self.wheeld = wheeld
    self.dt = dt
    self.pre_x = pos[0]
    self.pre_y = pos[1]
    self.pre_t = pos[2]
    self.pre_x_dot = 0
    self.pre_y_dot = 0
    self.pre_t_dot = 0
    return


  def set_wheel_radius(self, radius):
    # wheel radius [m]
    self.radius = radius
    return


  def set_wheel_distance(self, distance):
    # distance between wheels [m]
    self.wheeld = distance
    return

  
  def set_time_tick(self, tick):
    # time tick for simulation [sec]
    self.dt = tick
    return


  def set_pos(self, pos = (0, 0, math.pi/2)):
    # x:     pos[0] [m]
    # y:     pos[1] [m]
    # theta: pos[2] [rad]
    self.pre_x = pos[0]
    self.pre_y = pos[1]
    self.pre_t = pos[2]
    return


  def get_pos(self):
    return self.pre_x, self.pre_y, self.pre_t

  def control(self, w1, w2):
    # w1: [rad/s]
    # w2: [rad/s]
    x_dot = self.radius * (w1 + w2) * math.cos(self.pre_t)
    y_dot = self.radius * (w1 + w2) * math.sin(self.pre_t)
    t_dot = self.radius * (-w1 + w2) / self.wheeld

    x     = (self.dt * (self.pre_x_dot + x_dot) / 2) + self.pre_x
    y     = (self.dt * (self.pre_y_dot + y_dot) / 2) + self.pre_y
    theta = (self.dt * (self.pre_t_dot + t_dot) / 2) + self.pre_t

    self.pre_x = x
    self.pre_y = y
    self.pre_t = theta

    self.pre_x_dot = x_dot
    self.pre_y_dot = y_dot
    self.pre_t_dot = t_dot
    
    return x, y, theta



class DDMobileRobot(SimulatedObject):
  count = 0
  def __init__(self, simulator, radius=2, wheeld=20,
               pos = (0, 0, math.pi/2)):
    SimulatedObject.__init__(self, simulator)
    self.tick = self.get_tick()
    self.model = DiffMobileModel(radius, wheeld, pos, self.tick)
    self.fig = [[10, 0], [5, 10], [-10, 10], [-10, -10], [5, -10]]
    self.id = None
    self.wl = 0.0
    self.wr = 0.0
    self.name = "DDMobileRobot" + str(self.__class__.count)
    self.__class__.count += 1 
    self.comp = OpenRTM_aist.Manager.instance().createComponent("TkMobileRobotSimulator")

    # properties
    self.rentry = StringVar()
    self.rentry.set(radius)
    self.dentry = StringVar()
    self.dentry.set(wheeld)
    # input variables
    self.wlentry = StringVar()
    self.wrentry = StringVar()
    # output variables
    self.xentry = StringVar()
    self.yentry = StringVar()
    self.tentry = StringVar()
    return


  def __del__(self):
    try:
      self.comp.exit()
      del self.comp
    except:
      pass
    self.delete()
    return


  def get_name(self):
    return self.name


  def set_pos(self, (x, y, th)):
    self.model.set_pos((x, y, th))
    return


  def get_pos(self):
    return self.model.get_pos()


  def set_wheel_velocity(self, wl, wr):
    self.wl = wl
    self.wr = wr
    self.wlentry.set('%5.2f'%self.wl)
    self.wrentry.set('%5.2f'%self.wr)
    return


  def on_update(self):
    self.model.set_time_tick(self.get_tick())
    v = self.comp.get_velocity()
    if len(v) == 2:
      self.set_wheel_velocity(v[0], v[1])
    self.x, self.y, self.th = self.model.control(self.wl, self.wr)
    self.th_deg = (self.th * 180 / math.pi) % 360
    self.comp.set_position((self.x, self.y, self.th_deg))
    self.xentry.set('%5.2f'%self.x)
    self.yentry.set('%5.2f'%self.y)
    self.tentry.set('%5.2f'%self.th_deg)
    self.draw()
    return
        

  def draw(self):
    # converting actual coordinate system into display coordinate
    # system, and drawing figures
    robotfig = []
    for pos in self.fig:
      robotfig.append(self.translate(pos[0], pos[1],
                                       self.x, self.y, self.th))
    if self.id != None:
      self.canvas.delete(self.id)
    self.id = self.canvas.create_polygon(robotfig, 
                                         fill="#00aa00",
                                         outline="#eeeeee")
    return


  def delete(self):
    if self.id != None:
      self.canvas.delete(self.id)
    return


  def property_page(self):
    p = PropertyDialog()
    p.set_profile(self.name, "DDMobileRobot",
                  "Differential Drive Mobile Robot", "AIST")
    p.append_parameter("Wheel radius r: ", self.rentry, "[m]")
    p.append_parameter("Wheel distance d: ", self.dentry, "[m]")
    p.append_input("Angular velocity (LEFT)  wl: ", self.wlentry, "[rad/s]")
    p.append_input("Angular velocity (RIGHT) wr: ", self.wrentry, "[rad/s]")
    p.append_output("Robot position x : ", self.xentry, "[m]")
    p.append_output("Robot position y : ", self.yentry, "[m]")
    p.append_output("Robot position th: ", self.tentry, "[deg]")
    p.set_apply_param(self.on_apply_param)
    p.set_apply_input(self.on_apply_input)
    p.set_reset_output(self.on_reset_output)
    p.pack()
    return


  def on_reset_output(self):
    self.set_pos((0.0, 0.0, math.pi/2))
    return


  def on_apply_param(self):
    r = float(self.rentry.get())
    d = float(self.dentry.get())
    self.model.set_wheel_radius(r)
    self.model.set_wheel_distance(d)
    return


  def on_apply_input(self):
    self.wl = float(self.wlentry.get())
    self.wr = float(self.wrentry.get())
    return




class TkMobileRobot(Frame):
  def __init__(self, master=None, width=800, height=600):
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
    self.rname_check = StringVar()
    self.rname_check.set("on")
    self.rnames = {}

    self.robot_kind_var = StringVar()
    self.robot_factory = {"DDMobileRobot": DDMobileRobot}



    self.init()
    self.pack()


    self.after(20, self.on_update)
    return

  def init(self):
    self.canvas = Canvas(self, bg="#eeeeee",
                         width = self.width, height = self.height)
    self.canvas.pack(side=LEFT)

    self.can_grid = CanvasGrid(self.canvas, self.x0, self.y0,
                               self.width, self.height, self.grid_pitch,
                               "#aaaaaa", 1)
    self.can_axis = CanvasAxis(self.canvas, self.width, self.height)

    self.frame = Frame(self)
    self.frame.pack(side=LEFT)

    # Screen control
    self.scrctrl_frame = Frame(self.frame, width=self.wd, height=300,
                               relief=GROOVE, bd=2)
    self.scrctrl_frame.pack(side=TOP, fill=X)
    self.create_scale(self.scrctrl_frame)
    self.create_checkbutton(self.scrctrl_frame)

    # Robot manager
    self.robomgr_frame = Frame(self.frame, width=self.wd, height=300,
                               relief=GROOVE, bd=2)
    self.robomgr_frame.pack(side=TOP)
    self.create_robotcreator(self.robomgr_frame)
    self.create_robotlist(self.robomgr_frame)
    return


  def on_update(self):
    for o in self.robots.keys():
      self.robots[o].on_update()
    for r in self.rnames.keys():
      self.rnames[r].draw()
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

    sl = Scale(frame, from_=0.001, to=1, resolution=0.001,
               label="Time tick [s]", command=self.on_tickchange,
               variable=self.tickscale_var, orient=HORIZONTAL)
    bt = Button(frame, text="Reset Tick", command=self.reset_tickscale)
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
    return
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
    rname = Checkbutton(frame, text="Robots' name",
                        onvalue="on", offvalue="off",
                        justify=LEFT, anchor=W,
                        variable=self.rname_check,
                        command=self.on_rname_toggle)
    for w in [axis, grid, rname]:
      w.pack(side=TOP, anchor=W, fill=X)
    return


  def on_rname_toggle(self):
    for r in self.rnames.keys():
      self.rnames[r].toggle()
    return


  # end of Canvas control set
  #------------------------------------------------------------


  #------------------------------------------------------------
  # Robot creator control set
  def create_robotcreator(self, frame):
    lb = Label(frame, text="Robot Type", anchor=W, justify=LEFT)

    om = OptionMenu(frame, label="Type: ", variable=self.robot_kind_var)
    for opt in self.robot_factory.keys():
      om.add_command(opt, label=opt)
    self.robot_kind_var.set(self.robot_factory.keys()[0])

    creater = Button(frame, text="Create", command=self.create_robot)
    deleter = Button(frame, text="Delete", command=self.delete_robot)

    om.pack(side=TOP, fill=X)
    creater.pack(side=TOP, fill=X)
    deleter.pack(side=TOP, fill=X)
    return


  def create_robotlist(self, frame):
    f = Frame(frame, width=self.wd, height=200)
    f.pack(side=TOP, fill=BOTH, expand=1)
    ys = Scrollbar(f, orient = VERTICAL)
    ys.grid(row = 0, column=1, sticky = N+S)
    xs = Scrollbar(f, orient = HORIZONTAL)
    xs.grid(row = 1, column=0, sticky = E+W)

    self.rlistbox = Listbox(f,
                            xscrollcommand = xs.set,
                            yscrollcommand = ys.set,
                            selectmode = 'single',
                            setgrid = TRUE,
                            height=20)
    self.rlistbox.grid(row = 0, column = 0, sticky = N+S+E+W,)
    xs['command']=self.rlistbox.xview
    ys['command']=self.rlistbox.yview
    self.rlistbox.bind("<Double-Button-1>", self.on_clickrlistbox)
    return


  def on_clickrlistbox(self, event):
    index = self.rlistbox.curselection()
    if len(index) > 0:
      robot_name  = self.rlistbox.get(index[0])
      self.robots[robot_name].property_page()
    return


  def create_robot(self):
    kind = self.robot_kind_var.get()
    robot = self.robot_factory[kind](self)

    self.rlistbox.insert(END, robot.get_name())
    self.rnames[robot.get_name()] = RobotTitle(self,
                                               robot,
                                               robot.get_name(),
                                               20)
    self.robots[robot.get_name()] = robot
    return


  def delete_robot(self):
    index = self.rlistbox.curselection()
    if len(index) > 0:
      robot_name  = self.rlistbox.get(index[0])
      r = self.rnames.pop(robot_name)
      del(r)
      r = self.robots.pop(robot_name)
      del(r)
    self.rlistbox.delete(index)
    return

  # end of Robot creator control set
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


def TkMobileRobotSimulatorInit(manager):
  profile = OpenRTM_aist.Properties(defaults_str=tkmobilerobotsimulator_spec)
  manager.registerFactory(profile,
                          TkMobileRobotSimulator,
                          OpenRTM_aist.Delete)
  return


def main():
  m = TkMobileRobot(Tk())
  m.master.title("Tk Mobile Robot Simulator")
  mgr = OpenRTM_aist.Manager.init(sys.argv)
  mgr.activateManager()
  profile = OpenRTM_aist.Properties(defaults_str=tkmobilerobotsimulator_spec)
  mgr.registerFactory(profile, TkMobileRobotSimulator, OpenRTM_aist.Delete)
  mgr.runManager(True)
  m.mainloop()

if  __name__ == '__main__':
  main()
