#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  @file RtmFrame.py
#  @brief rtc-link main frame class
#  @date $Date: 2005-05-27 15:51:31 $
#  @author Noriaki Ando <n-ando@aist.go.jp>
# 
#  Copyright (C) 2003-2005
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id$
# 

import sys, os, time
import wx                  # This module uses the new wx namespace
import wx.html
import rtimages
from wxPython.wx import *
#import ScrolledWindow
#import OGL
import RtmSystemDraw
import wx.ogl  as  ogl
import os
import cPickle
#from wxPython.wx import *
#import wx

import threading

import RtmAbout

from RtmCompData import *

import RTM

#ID_NEW     = 100
#ID_OPEN    = 101
#ID_CONNECT = 102
#ID_EXIT    = 103
#ID_COPY    = 104
#ID_CUT     = 105
#ID_PASTE   = 106
#ID_HELP    = 107
#ID_ABOUT   = 108

ID_WINDOW_LEFT          = 5000
ID_WINDOW_CENTER_TOP    = 5001
ID_WINDOW_CENTER_BOTTOM = 5002
ID_WINDOW_RIGHT         = 5003

class MenuFactory:
	def __init__(self, parent, menulist):
		self.parent = parent
		self.menu = wx.Menu()
		for item_name, handler in menulist:
			if item_name == "---":
				self.menu.AppendSeparator()
			else:
				new_id = wx.NewId()
				parent.Bind(wx.EVT_MENU, handler, id=new_id)
				item = wx.MenuItem(self.menu, new_id, item_name)
				self.menu.AppendItem(item)
	def GetMenu(self):
		return self.menu

class RTComponentDropTarget(wx.DropTarget):
	def __init__(self, window):
		wx.DropTarget.__init__(self)
		self.window = window

		self.df = wx.CustomDataFormat("RTComponent")
#		print self.df, dir(self.df)
		print "RTComponentDropTarget", self.df.GetId(), self.df.GetType()
		self.data = wx.CustomDataObject(self.df)
		self.SetDataObject(self.data)
		
	def OnDragOver(self, x, y, d):
		print "OnDragOver"
		self.window.AppendText("OnDragOver\n")
		return wx.DragLink

	def OnDrop(self, x, y):
		print "OnDrop"
		print x, y
		return True

		
	def OnData(self, x, y, d):
		print "OnData"
		if self.GetData():
			data = self.data.GetData()
			cdata = cPickle.loads(data)
			self.window.AppendText(cdata + "\n")
#			self.window.AppendText(type(cdata[1]) + "\n")

#			return wx.DragNone
		#		url = self.data.GetURL()
		return d


class RtmFileMenu(MenuFactory):
	def __init__(self, parent):
		menu_list = [("&New System", self.OnNew),
					 ("&Open System", self.OnOpen),
					 ("&Save System", self.OnSave),
					 ("Save System &as", self.OnSaveAs),
					 ("---", None),
					 ("&Import Component", self.OnImport),
					 ("&Export Component", self.OnExport),
					 ("---", None),
					 ("Print Pre&view", self.OnPrintPreview),
					 ("&Print System", self.OnPrint),
					 ("---", None),
					 ("&Connect Naming Server", self.OnConnect),
					 ("---", None),
					 ("E&xit", self.OnExit)]
		MenuFactory.__init__(self, parent, menu_list)

	def OnNew(self, event):
		self.parent.OnNewSystemClick(None)
		print "OnNew"

	def OnOpen(self, event):
		self.parent.drawWin[self.parent.drawCurNum].loadXML()
		print "OnOpen"

	def OnSave(self, event):
		filename = "System%d.xml" % self.parent.drawCurNum
		self.parent.drawWin[self.parent.drawCurNum].saveXML(filename)
		print "OnSave"

	def OnSaveAs(self, event):
		self.parent.drawWin[self.parent.drawCurNum].saveAsXML()
		print "OnSaveAs"

	def OnImport(self, event):
		self.parent.OnFileOpen(event)
		print "OnImport"

	def OnExport(self, event):
		self.parent.OnFileOpen(event)
		print "OnExport"

	def OnPrint(self, event):
		dlg = wx.PrintDialog(self.parent, None)
		if dlg.ShowModal() == wx.ID_OK:
			data = dlg.GetPrintDialogData()
		print "OnPrint"

	def OnPrintPreview(self, event):
		self.parent.OnFileOpen(event)
		print "OnOpen"

	def OnConnect(self, event):
		self.parent.treepanel.OnConnectNSClick(None)
		print "OnConnect"

	def OnExit(self, event):
		print "OnExit"
		self.parent.treepanel.threadloop = 0
		self.parent.close_evt.wait()
		self.parent.Close(true)


class RtmEditMenu(MenuFactory):
	def __init__(self, parent):
		menu_list = [("&Copy", self.OnCopy),
					 ("C&ut", self.OnCut),
					 ("&Paste", self.OnPaste)]
		MenuFactory.__init__(self, parent, menu_list)

	def OnCopy(self, event):
		print "OnCopy"

	def OnCut(self, event):
		print "OnCut"

	def OnPaste(self, event):
		print "OnPaste"

class RtmDisplayMenu(MenuFactory):
	def __init__(self, parent):
		menu_list = [("Long Name &and Alias", self.OnAllDisp),
					 ("&Long Name", self.OnLongDisp),
					 ("&Alias", self.OnAliasDisp)]
		MenuFactory.__init__(self, parent, menu_list)

	def OnAllDisp(self, event):
		self.parent.kindDispMode = 'all'
		print "OnAllDisp"

	def OnLongDisp(self, event):
		self.parent.kindDispMode = 'long'
		print "OnLongDisp"

	def OnAliasDisp(self, event):
		self.parent.kindDispMode = 'alias'
		print "OnAliasDisp"


class RtmHelpMenu(MenuFactory):
	def __init__(self, parent):
		menu_list = [("&Help", self.OnHelp),
					 ("---", None),
					 ("&About", self.OnAbout)]
		MenuFactory.__init__(self, parent, menu_list)

	def OnHelp(self, event):
		print "OnHelp"

	def OnAbout(self, event):
		print "OnAbout"
		dlg = RtmAbout.RtdAboutBox(None)
		dlg.ShowModal()
		dlg.Destroy()





class Log:
	def WriteText(self, text):
		if text[-1:] == '\n':
			text = text[:-1]
#		wxLogMessage(text)
	write = WriteText
	
#class RtdLog(wx.PyLog):
#	def __init__(self, textCtrl, logTime=0):
#		wx.PyLog.__init__(self)
#		self.tc = textCtrl
#		self.logTime = logTime
#		
#	def DoLogString(self, message, timeStamp):
#		if self.logTime:
#			message = time.strftime("%X", time.localtime(timeStamp)) + \
#					  ": " + message
#		if self.tc:
#			self.tc.AppendText(message + '\n')



class RtdFrame(wxMDIParentFrame):
	def __del__(self):
#		for sys_no in self.drawWin.keys():
#			del self.drawWin[sys_no]

		ogl.OGLCleanUp()

	def __init__(self, parent, ID, title):
		wxMDIParentFrame.__init__(self, parent, -1, title, size = (800, 600),
						  style=wx.DEFAULT_FRAME_STYLE|wx.HSCROLL | wx.VSCROLL
								  ##wx.NO_FULL_REPAINT_ON_RESIZE
								  )
		#		wx.Frame.__init__(self, parent, ID, title,
		#						 wxDefaultPosition, wxSize(800, 600))
		self.winCount = 0
		self.drawWin = {}
		self.drawWin2 = {}
		self.drawWinID = {}
		self.drawCurNum = 0
		self.cwd = os.getcwd()
		self.curOverview = ""
		self.window = None
		self.log = Log()
		ogl.OGLInitialize()
		self.close_evt = threading.Event()
		self.kindDispMode = 'all'
		#------------------------------------------------------------
		# Basic window frame settings
		#------------------------------------------------------------
		# Set window icon image
		self.SetIcon(rtimages.getRT_iconIcon())

		# Status Bar
		self.status_bar = self.CreateStatusBar()

		# Tool Bar
		self.toolbar = self.CreateToolBar(wxTB_HORIZONTAL
									 | wxTB_DOCKABLE 
									 | wx.NO_BORDER
									 | wx.TB_FLAT
#									 | wx.TB_TEXT
									 )

		self.exitdoorID = wx.NewId()
		self.toolbar.AddSimpleTool(self.exitdoorID, rtimages.getExitDoorBitmap(),
								   "Exit", "Exit from rtc-link.")
		self.Bind(wx.EVT_TOOL, self.TimeToQuit, id=self.exitdoorID)

		self.toolbar.AddSeparator()

		self.connectID = wx.NewId()
		self.toolbar.AddSimpleTool(self.connectID,
								   rtimages.getConnect2Bitmap(),
								   "Connect",
								   "Connect to a Naming Server.")
#		self.toolbar.EnableTool(self.connectID, False)

		self.alldispID = wx.NewId()
		self.toolbar.AddSimpleTool(self.alldispID, rtimages.getMixNameBitmap(),
								   "Long Name and Alias", "Display long name and alias on the naming tree window.")
		self.Bind(wx.EVT_TOOL, self.OnAllDisp, id=self.alldispID)


		self.longdispID = wx.NewId()
		self.toolbar.AddSimpleTool(self.longdispID, rtimages.getLongNameBitmap(),
								   "Long Name", "Display only long name on the naming tree window.")
		self.Bind(wx.EVT_TOOL, self.OnLongnameDisp, id=self.longdispID)

		self.aliasdispID = wx.NewId()
		self.toolbar.AddSimpleTool(self.aliasdispID, rtimages.getShortNameBitmap(),
								   "Alias", "Display only alias on the naming tree window.")
		self.Bind(wx.EVT_TOOL, self.OnAliasDisp, id=self.aliasdispID)
		#		self.Bind(wx.EVT_TOOL, self.OnToolClick, id=40)
		#		self.Bind(wx.EVT_TOOL_RCLICKED, self.OnToolRClick, id=40)

		self.toolbar.AddSeparator()

		self.newsysID = wx.NewId()
		self.toolbar.AddSimpleTool(self.newsysID,
								   rtimages.getNewRTSystemBitmap(),
								   "New",
								   "Open.: Open a new system draw window.")
		self.Bind(wx.EVT_TOOL, self.OnNewSystemClick, id=self.newsysID)

#		cbID = wx.NewId()
#		self.toolbar.AddControl(
#			wx.ComboBox(
#			self.toolbar, cbID, "", choices=["", "This", "is a", "wxComboBox"],
#			size=(150,-1), style=wx.CB_DROPDOWN
#			))
		self.toolbar.Realize()

		# Menu bar
		self.SetMenuBar(self.CreateMenu())


		#------------------------------------------------------------
		# Menu event binding
		#------------------------------------------------------------
		#		EVT_MENU(self, ID_ABOUT, self.OnAbout)
		#		EVT_MENU(self, ID_EXIT, self.TimeToQuit)
		#		EVT_MENU(self, ID_OPEN, self.OnFileOpen)

		#------------------------------------------------------------
		# Dictionary 
		#------------------------------------------------------------
		self.myDict = RtmCompData(self)

		#------------------------------------------------------------
		# Naming Tree Window
		#------------------------------------------------------------
		self.nametreeW_ID = wxNewId()
		win = wxSashLayoutWindow(self, self.nametreeW_ID,
								 style=wx.NO_BORDER|wx.SW_3D)
		win.SetDefaultSize((200, 600))
		win.SetOrientation(wxLAYOUT_VERTICAL)
		win.SetAlignment(wxLAYOUT_LEFT)
		win.SetBackgroundColour(wxColour(200, 200, 200))
		win.SetSashVisible(wxSASH_RIGHT, True)

		# Tree
		tID = wxNewId()
		self.treeMap = {}

		import RtmTreeCtrl
		log = Log()
#		self.treepanel = RtmTreeCtrl.RtmTreeCtrlPanel(win, log)
		self.treepanel = RtmTreeCtrl.RtmTreeCtrlPanel(self,win, log)
		self.Bind(wx.EVT_TOOL,
				  self.treepanel.OnConnectNSClick,
				  id=self.connectID)
		self.treepanelW = win

		#------------------------------------------------------------
		# Profile Window
		#------------------------------------------------------------
		self.profileW_ID = wxNewId()
		win = wxSashLayoutWindow(self, self.profileW_ID,
								 style=wx.NO_BORDER|wx.SW_3D)
		win.SetDefaultSize((200, 600))
		win.SetOrientation(wxLAYOUT_VERTICAL)
		win.SetAlignment(wxLAYOUT_RIGHT)
		win.SetBackgroundColour(wxColour(200, 200, 200))
		win.SetSashVisible(wxSASH_LEFT, True)

		import RtmProfileList
		self.profilepanel = RtmProfileList.RtmProfileListPanel(win)
		self.profilepanelW = win

		#------------------------------------------------------------
		# Log Window
		#------------------------------------------------------------
		self.logW_ID = wxNewId()
		win =  wxSashLayoutWindow(self, self.logW_ID,
								  style=wx.NO_BORDER|wx.SW_3D)
		win.SetDefaultSize((400, 100))
		win.SetOrientation(wx.LAYOUT_HORIZONTAL)
		win.SetAlignment(wx.LAYOUT_BOTTOM)
		win.SetBackgroundColour(wx.Colour(200, 200, 200))
		win.SetSashVisible(wx.SASH_TOP, True)

		self.textWindow = wx.TextCtrl(win, -1, "",
									  style=wx.TE_MULTILINE|wx.SUNKEN_BORDER)
		self.textWindow.SetValue("A sub window")
		
		self.logW = win
		self.textWindow.SetDropTarget(RTComponentDropTarget(self.textWindow))
		
		self.bg_bmp = rtimages.getGridBGBitmap()
		self.GetClientWindow().Bind(
			wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground
			)

		self.Bind(wx.EVT_CLOSE, self.OnClose)
		self.Bind(
			wx.EVT_SASH_DRAGGED_RANGE, self.OnSashDrag,
			id=self.nametreeW_ID, 
			id2=self.logW_ID
			)
		self.Bind(wx.EVT_SIZE, self.OnSize)


	def OnAllDisp(self, event):
		self.kindDispMode = 'all'

	def OnLongnameDisp(self, event):
		self.kindDispMode = 'long'

	def OnAliasDisp(self, event):
		self.kindDispMode = 'alias'

	def OnNewSystemClick(self, event):
		self.winCount = self.winCount + 1
		new_id = wx.NewId()
		win = wxMDIChildFrame(self, new_id , "System: %d" % self.winCount)
		#		canvas = ScrolledWindow.MyCanvas(win)
		self.drawWin[self.winCount] = RtmSystemDraw.RtdSystemDraw(win, self.log, self)
		self.drawWin2[self.winCount] = win
		win.Show(True)
		win.SetIcon(rtimages.getRTIcon())
		self.drawWinID[self.winCount] = self.drawWin[self.winCount].GetId()
		self.drawCurNum = self.winCount

		os_check = sys.platform
		if os_check == 'win32':
			self.drawWin[self.winCount].Bind(wx.EVT_SET_FOCUS, self.OnChangeDraw)
		else:
			self.drawWin[self.winCount].Bind(wx.EVT_ENTER_WINDOW, self.OnChangeDraw)
		self.drawWin2[self.winCount].Bind(wx.EVT_CLOSE, self.OnChildClose)

	def OnChildClose(self, event):
		curID = event.GetId()
		for n in self.drawWinID.keys():
			if self.drawWinID[n] == curID:
				self.drawCurNum = n
				break
		del self.drawWin[self.drawCurNum]
		del self.drawWinID[self.drawCurNum]
		event.Skip()
		
	def OnChangeDraw(self, event):
		curID = event.GetId()
		for n in self.drawWinID.keys():
			if self.drawWinID[n] == curID:
				self.drawCurNum = n
				break

	def OnSize(self, event):
		wxLayoutAlgorithm().LayoutMDIFrame(self)


	def OnSashDrag(self, event):
		if event.GetDragStatus() == wxSASH_STATUS_OUT_OF_RANGE:
			return

		eID = event.GetId()
		if eID == self.nametreeW_ID:
			self.treepanelW.SetDefaultSize((event.GetDragRect().width, 200))

		elif eID == self.profileW_ID:
			self.profilepanelW.SetDefaultSize((event.GetDragRect().width, 200))

		elif eID == self.logW_ID:
			print "logW_ID", self.logW_ID
			self.logW.SetDefaultSize((1000, event.GetDragRect().height))

		wx.LayoutAlgorithm().LayoutMDIFrame(self)
		self.GetClientWindow().Refresh()


	def OnClose(self, event):
		if hasattr(self, "treepanel"):
			print 'OnClose:'
			self.treepanel.threadloop = 0
			self.close_evt.wait()
		event.Skip()
		
	def TimeToQuit(self, event):
		self.Close(true)
		

	def CreateMenu(self):
		menu_file = RtmFileMenu(self).GetMenu()
		menu_edit = RtmEditMenu(self).GetMenu()
		menu_disp = RtmDisplayMenu(self).GetMenu()
		menu_help = RtmHelpMenu(self).GetMenu()

		menuBar = wxMenuBar()
		menuBar.Append(menu_file, "&File");
		menuBar.Append(menu_edit, "&Edit");
		menuBar.Append(menu_disp, "&Display");
		menuBar.Append(menu_help, "&Help");

		# Disable to menu item
		id = menuBar.FindMenuItem("Edit","Copy")
		menuBar.FindItemById(id).Enable(false)
		id = menuBar.FindMenuItem("Edit","Cut")
		menuBar.FindItemById(id).Enable(false)
		id = menuBar.FindMenuItem("Edit","Paste")
		menuBar.FindItemById(id).Enable(false)

# assembly enable
#		id = menuBar.FindMenuItem("File","Open System")
#		menuBar.FindItemById(id).Enable(false)
#		id = menuBar.FindMenuItem("File","Save System")
#		menuBar.FindItemById(id).Enable(false)
#		id = menuBar.FindMenuItem("File","Save System as")
#		menuBar.FindItemById(id).Enable(false)
#
		id = menuBar.FindMenuItem("File","Import Component")
		menuBar.FindItemById(id).Enable(false)
		id = menuBar.FindMenuItem("File","Export Component")
		menuBar.FindItemById(id).Enable(false)

		id = menuBar.FindMenuItem("File","Print Preview")
		menuBar.FindItemById(id).Enable(false)
		id = menuBar.FindMenuItem("File","Print System")
		menuBar.FindItemById(id).Enable(false)

		return menuBar

	def OnFileOpen(self, event):
		dlg = wx.FileDialog(
			self, message="Choose a file", defaultDir=os.getcwd(), 
			defaultFile="", wildcard="*.*", style=wx.OPEN | wx.MULTIPLE | wx.CHANGE_DIR
			)
        
		if dlg.ShowModal() == wx.ID_OK:
			# This returns a Python list of files that were selected.
			self.paths = dlg.GetPaths()

			#		for path in paths:
			#            log.WriteText('           %s\n' % path)

			# Compare this with the debug above; did we change working dirs?

			# Destroy the dialog. Don't do this until you are done with it!
			# BAD things can happen otherwise!
		dlg.Destroy()



	def OnEraseBackground(self, evt):
		dc = evt.GetDC()

		if not dc:
			dc = wx.ClientDC(self.GetClientWindow())

			# tile the background bitmap
		sz = self.GetClientSize()
		w = self.bg_bmp.GetWidth()
		h = self.bg_bmp.GetHeight()
		x = 0

		while x < sz.width:
			y = 0

			while y < sz.height:
				dc.DrawBitmap(self.bg_bmp, (x, y))
				y = y + h

			x = x + w
