#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

from Tkinter import *
import thread
import time
import math

class TkMotor(Frame):
	def __init__(self, num, radius, master=None):
		Frame.__init__(self, master)
		self.num = num
		self.r = radius
		self.init()
		self.pack()



	def init(self):
		self.space = 4
		canwidth = self.r * self.space * self.num
		canheight = self.r * self.space
		self.can = Canvas(self, bg="white",
						  width=canwidth, height=canheight)
		self.can.pack()
		self.circ = []
		self.line = []
		self.x = []
		self.y = []
		self.x1 = []
		self.y1 = []
		self.x2 = []
		self.y2 = []
		self.sx1 = []
		self.sy1 = []
		self.sx2 = []
		self.sy2 = []
		
		for i in xrange(self.num):
			self.x.append(self.r * self.space / 2 * (i+0.5) * 2)
			self.y.append(self.r * self.space / 2)
			self.x1.append(self.x[i] - self.r)
			self.y1.append(self.y[i] - self.r)
			self.x2.append(self.x[i] + self.r)
			self.y2.append(self.y[i] + self.r)
			self.sx1.append(self.x[i] - self.r * 0.1)
			self.sy1.append(self.y[i] - self.r * 0.1)
			self.sx2.append(self.x[i] + self.r * 0.1)
			self.sy2.append(self.y[i] + self.r * 0.1)
			self.circ.append(self.can.create_oval(self.x1[i], self.y1[i],
												  self.x2[i], self.y2[i],
												  outline="#aaaaaa",
												  fill="#dddddd"))
			self.circ.append(self.can.create_oval(self.sx1[i], self.sy1[i],
												  self.sx2[i], self.sy2[i],
												  outline="#000000",
												  fill="#000000"))
			self.line.append(self.can.create_line(self.x[i], self.y[i],
												  self.x[i],
												  self.y[i] - self.r,
												  fill="#700040", width=5))

		
	def set_angle(self, angle):
		if len(angle) != self.num: return
		i = 0
		for a in angle:
			a = a * math.pi / 180
			self.can.delete(self.line[i])
			self.line[i] = self.can.create_line(self.x[i], self.y[i],
								 self.x[i] + self.r * math.cos(a),
								 self.y[i] + self.r * math.sin(a),
								  fill="#700040", width=5)
			i = i + 1



def test ():
	m = TkMotor(6, 40, Toplevel())
	n = TkMotor(6, 40, Toplevel())
	thread.start_new_thread(m.mainloop, ())
	thread.start_new_thread(n.mainloop, ())
	cnt = 0
	while(1):
		angle = [cnt % 360, - cnt % 360, cnt % 360, - cnt % 360, cnt % 360, - cnt % 360]
		m.set_angle(angle)
		time.sleep(0.001)
		cnt = cnt + 1
		

if  __name__ == '__main__': test()
