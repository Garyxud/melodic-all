#!/usr/bin/env python
#
#  @file RtmTreeCtrl.py
#  @brief rtc-link name tree management class
#  @date $Date: 2007-01-21 13:21:26 $
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

# Basic modules
import string
import time
import cPickle
import thread

# wxPython modules
import wx
import rtimages

# RTM modules
import RTM
import RtmNSHelper
from RtmCompData import *
from RtmDialog import *


#-------------------------------------------------------------------------------
class RtmPopup:
	"""ポップアップメニュ基底クラス"""
	def __init__(self, parent, menulist):
		self.menu = wx.Menu()
		self.parent = parent
		self.root_item = self.parent.tree.GetRootItem()
		for item_name, handler in menulist:
			if item_name == "---":
				self.menu.AppendSeparator()
			elif handler == None:
				sm = wx.Menu()
				new_id = wx.NewId()
				self.menu.AppendMenu(new_id, item_name, sm)
			else:
				new_id = wx.NewId()
				self.parent.Bind(wx.EVT_MENU, handler, id=new_id)
				item = wx.MenuItem(self.menu, new_id, item_name)
				self.menu.AppendItem(item)

	def GetMenu(self):
		"""メニューを取得する

		[引数]
		void

		[戻り値]
		self.menu -- メニューオブジェクト
		"""
		return self.menu

	def SetItem(self, item):
		"""カレントのツリーItemIDををセットする

		[引数]
		item -- ItemID

		[戻り値]
		void
		"""
		self.item = item

	def DeleteToCxt(self, event):
		test_id = self.parent.tree.GetItemText(self.item)
		fullpath = self.parent.makeFullPath(self.item)
		if self.item != self.root_item:
			item = self.parent.tree.GetItemParent(self.item)
			parent_path = self.parent.makeFullPath(item)
			if item != self.root_item:
				cxt_obj = self.parent.myDict.GetObjRefToFullpath(parent_path)
			else:
				cxt_obj = self.parent.nshelper.root_cxt
		else:
			cxt_obj = self.parent.nshelper.root_cxt
		bname = self.parent.myDict.GetBindingNameToFullpath(fullpath)
		self.parent.nshelper.DeleteToContext(cxt_obj,bname)

	def CreateComp(self,event):
		test_id = self.parent.tree.GetItemText(self.item)
		fullpath = self.parent.makeFullPath(self.item)
		mod_name = self.parent.myDict.GetCompName(fullpath)
		cate_name = self.parent.myDict.GetCateName(fullpath)
		if cate_name == '':
			print 'cate_cxt search error!!'
			return
		mgrpath = self.parent.searchManagerPath(self.item)
		if mgrpath == None:
			print 'RTCManager object-ref search error!!'
			return

		objref = self.parent.myDict.GetObjRefToFullpath(mgrpath)
		try:
			objref._narrow(RTM.RTCManager)

			(ret,ret_str) = objref.create_component(mod_name, cate_name)
		except:
			except_mess("create_component error!:")


#-------------------------------------------------------------------------------
class RtmCompPopup(RtmPopup):
	"""コンポーネントItemポップアップクラス"""
	def __init__(self, parent):
		menulist = [("Start", self.OnStart),
					("Stop", self.OnStop),
					("Reset", self.OnReset),
					("Exit", self.OnExit),
					("Kill", self.OnKill),
					("---", None),
					("Delete", self.OnDelete),
					("Profile", self.OnProperty)]
		self.parent = parent
		self.rootitem_id = self.parent.tree.GetRootItem()
		return RtmPopup.__init__(self, parent, menulist)

	def makeFullPath(self, cur_id):
		"""現在のItemIdからFullPathを生成
				
		[引数]
		cur_id -- 現在のItemID

		[戻り値]
		ret -- ツリーのFull Path
		"""
		ret = self.parent.tree.GetItemText(self.item)
#		num = ret.find('|')
#		if num != -1:
#			ret = ret[0:num]
		while (self.rootitem_id != cur_id):
			tmp_id = self.parent.tree.GetItemParent(cur_id)
#			if tmp_id == self.rootitem_id:
#				break
			name = self.parent.tree.GetItemText(tmp_id)
#			num = name.find('|')
#			if num != -1:
#				name = name[0:num]
			ret = name + '/' + ret
			cur_id = tmp_id
		return str(ret)

	def OnStart(self, event):
		"""コンポーネントをrtc_start()させる
		[引数]
		event -- Event, 使用していない

		[戻り値]
		void
		"""	
		try:
			fullpath = self.makeFullPath(self.item)
#			print "full-path:",fullpath
			ret = self.parent.myDict.TreeListCompStart(fullpath)
			if (ret != RTM.RTM_OK):
				print "RTM_ERR:"
		except:
			except_mess("except error!:")
			
		print "OnStart"
	
	def OnStop(self, event):
		"""コンポーネントをrtc_stop()させる
		[引数]
		event -- Event, 使用していない

		[戻り値]
		void
		"""	
		fullpath = self.makeFullPath(self.item)
#		print "full-path:",fullpath
		ret = self.parent.myDict.TreeListCompStop(fullpath)
		print "OnStop"
	
	def OnReset(self, event):
		"""コンポーネントをrtc_reset()させる
		[引数]
		event -- Event, 使用していない

		[戻り値]
		void
		"""	
		fullpath = self.makeFullPath(self.item)
#		print "full-path:",fullpath
		ret = self.parent.myDict.TreeListCompReset(fullpath)
		print "OnReset"
	
	def OnExit(self, event):
		"""コンポーネントをrtc_exit()させる
		[引数]
		event -- Event, 使用していない

		[戻り値]
		void
		"""	
		fullpath = self.makeFullPath(self.item)
#		print "full-path:",fullpath
		ret = self.parent.myDict.TreeListCompExit(fullpath)
		print "OnExit"
	
	def OnKill(self, event):
		"""コンポーネントをrtc_kill()させる
		[引数]
		event -- Event, 使用していない

		[戻り値]
		void
		"""	
		fullpath = self.makeFullPath(self.item)
#		print "full-path:",fullpath
		ret = self.parent.myDict.TreeListCompKill(fullpath)
		print "OnKill"
	
	def OnProperty(self, event):
		fullpath = self.makeFullPath(self.item)
		kind = self.parent.myDict.GetKindToFullpath(fullpath)
		if kind == "rtc":
			ref = self.parent.myDict.GetObjRefToFullpath(fullpath)
			try:
				self.parent.frame.profilepanel.RefreshProfile(ref.get_component_profile())
			except:
				except_mess("except error!:")
				pass
		print "OnProperty"

	def OnDelete(self, event):
		print 'OnDelete'
		self.DeleteToCxt(event)
	
#-------------------------------------------------------------------------------
class RtmModulePopup(RtmPopup):
	"""モジュールItemポップアップクラス"""
	def __init__(self, parent):
		menulist = [("Create", self.OnCreate),
					("---", None),
					("Delete", self.OnDelete)]
#					("Property", self.OnProperty)]
		return RtmPopup.__init__(self, parent, menulist)

	def OnCreate(self, event):
		print "OnCreate"
		self.CreateComp(event)
	
	def OnDelete(self, event):
		print "OnDelete"
		self.DeleteToCxt(event)
	
	def OnProperty(self, event):
		print "OnProperty"
	
#-------------------------------------------------------------------------------
class RtmUnknownPopup(RtmPopup):
	"""Unknown Itemポップアップクラス"""
	def __init__(self, parent):
		menulist = [("Delete", self.OnDelete)]
		return RtmPopup.__init__(self, parent, menulist)

	def OnDelete(self, event):
		print "OnDelete"
		self.DeleteToCxt(event)
	
#-------------------------------------------------------------------------------
class RtmManagerPopup(RtmPopup):
	"""マネージャItemポップアップクラス"""
	def __init__(self, parent):
		menulist = [("Create", None),
					("Load", self.OnLoad),
					("---", None),
					("Delete", self.OnDelete)]
#					("Property", self.OnProperty)]

		return RtmPopup.__init__(self, parent, menulist)

	def SetSubMenu(self):
		tmp_id = self.menu.FindItem("Create")

		item_list = self.menu.GetMenuItems()
		sub_menu = None
		for item in item_list:
			if item.GetId() == tmp_id:
				sub_menu = item.GetSubMenu()
				break

		child_items = sub_menu.GetMenuItems()
		for item in child_items:
			id = item.GetId()
			sub_menu.Remove(id)
			del item

		mgrpath = self.parent.searchManagerPath(self.item)
		if mgrpath == None:
			print 'RTCManager object-ref search error!!'
			return

		objref = self.parent.myDict.GetObjRefToFullpath(mgrpath)
		try:
			objref._narrow(RTM.RTCManager)

			fact_list = objref.factory_list()
		except:
			except_mess("component_factory_list error!:")

		for item_struct in fact_list:
			name = item_struct.name
			cate = item_struct.category
			item_name = cate + '/' + name
			new_id = wx.NewId()
			item = wx.MenuItem(sub_menu, new_id, item_name)
			sub_menu.AppendItem(item)
			self.parent.Bind(wx.EVT_MENU, self.OnCreateSub, id=new_id)

	def OnCreateSub(self, event):
		item_id= event.GetId()
		item_name = self.menu.GetLabel(item_id)
		num = item_name.find('/')
		cate = str(item_name[0:num])
		name = str(item_name[num+1:])
		mgrpath = self.parent.searchManagerPath(self.item)
		if mgrpath == None:
			print 'RTCManager object-ref search error!!'
			return
		objref = self.parent.myDict.GetObjRefToFullpath(mgrpath)
		try:
			objref._narrow(RTM.RTCManager)

			(ret,ret_str) = objref.create_component(name, cate)
		except:
			except_mess("create_component error!:")

	def OnCreate(self, event):
		print "OnCreate"
		mgrpath = self.parent.searchManagerPath(self.item)
		if mgrpath == None:
			print 'RTCManager object-ref search error!!'
			return
		cate_name = self.parent.searchCategoryName(self.item)
		if cate_name == '':
			print 'cate_cxt search error!!'
			return

		objref = self.parent.myDict.GetObjRefToFullpath(mgrpath)
		try:
			objref._narrow(RTM.RTCManager)

			item_id= event.GetId()
			item_name = self.menu.GetLabel(item_id)
			print "item_name:",item_name

			(ret,ret_str) = objref.create_component(item_name, cate_name)
		except:
			except_mess("create_component error!:")

	
	def OnLoad(self, event):
		print "OnLoad"
	
	def OnProperty(self, event):
		print "OnProperty"
	
	def OnDelete(self, event):
		print "OnDelete"
		self.DeleteToCxt(event)
	
#-------------------------------------------------------------------------------
class RtmNameServerPopup(RtmPopup):
	"""ネームサーバItemポップアップクラス"""
	def __init__(self, parent):
		menulist = [("Connect", self.OnConnect),
					("Refresh", self.OnRefresh)]
		return RtmPopup.__init__(self, parent, menulist)

	def OnConnect(self, event):
		"""ネームサーバ接続ダイアログを表示させる
		[引数]
		event -- Event, 使用していない

		[戻り値]
		void
		"""	
		self.parent.OnConnectNSClick(None)
	
	def OnRefresh(self, event):
		"""ネーミングツリーをリフレッシュする
		[引数]
		event -- Event, 使用していない
	
		[戻り値]
		void
		"""	
		dummy = 0
		self.parent.myDict.Mutex.lock(self.parent.Refresh,dummy)
		self.parent.myDict.Mutex.unlock()


#-------------------------------------------------------------------------------
class RtmTreeCtrl(wx.TreeCtrl):
	"""ネーミングツリークラス"""
	def __init__(self, parent, id, pos, size, style, log):
		"""初期化

		[引数]
		parent -- 親ウインドウ
		id     -- ウインドウID
		pos    -- 位置
		size   -- サイズ
		style  -- ウインドウスタイル
		# wx.TreeCtrl と同じ

		[戻り値]
		this
		"""
		
		wx.TreeCtrl.__init__(self, parent, id, pos, size, style)
		self.log = log

		isz = (16,16)
		il = wx.ImageList(isz[0], isz[1])

		self.treeimg_list = {}

		self.parent = parent
		
		rticon   = il.Add(rtimages.getRTLogoBitmap())
		computer = il.Add(rtimages.getComputerBitmap())
		manager  = il.Add(rtimages.getManagerBitmap())
		category = il.Add(rtimages.getCategoryBitmap())
		module   = il.Add(rtimages.getModuleBitmap())
		component= il.Add(rtimages.getComponentBitmap())
						  
		self.treeimg_list['unknown'] = (rticon,rticon)
		self.treeimg_list['ns'] = (rticon,rticon)
		self.treeimg_list['host_cxt'] = (computer, computer)
		self.treeimg_list['mgr_cxt'] = (manager, manager)
		self.treeimg_list['mgr'] = (category, category)
		self.treeimg_list['cate_cxt'] = (category, category)
		self.treeimg_list['mod_cxt'] = (module, module)
		self.treeimg_list['rtc'] = (component, component)

		print self.treeimg_list
		self.depth = 0

		self.SetImageList(il)
		self.il = il

		self.depth = 0
		self.root = self.AddRoot("NS:")
		self.SetPyData(self.root, None)
		self.SetItemImage(self.root, self.treeimg_list['ns'][0],
						  wx.TreeItemIcon_Normal)
		self.SetItemImage(self.root, self.treeimg_list['ns'][0],
						  wx.TreeItemIcon_Expanded)
		self.Expand(self.root)
		rootitem_id = self.GetRootItem()
		self.SelectItem(rootitem_id)

		self.kindDispMode = self.parent.frame.kindDispMode

	def OnCompareItems(self, item1, item2):
		"""itemの比較

		[引数]
		item1 -- item1
		item2 -- item2

		[戻り値]
		-1 -- t1 < t2
		0  -- t1 = t2
		1  -- t1 > t2
		"""		
		t1 = self.GetItemText(item1)
		t2 = self.GetItemText(item2)
		self.log.WriteText('compare: ' + t1 + ' <> ' + t2 + '\n')
		if t1 < t2: return -1
		if t1 == t2: return 0
		return 1

	def PrintAll(self):
		"""全Itemの表示

		[引数]
		void

		[戻り値]
		void
		"""
		rootitem_id = self.GetRootItem()
		self.__ListupRecursive__(rootitem_id)
	
	def checkDispKind(self,kind):
		ret = 1
		kindCheck = 0
		mode = self.kindDispMode
		# 'all', 'alias','long'

		if 'cxt_' == kind[0:4] or '' == kind or kind == None:
			kindCheck = 'alias'
		else:
			kindCheck = 'long'
		
		if mode == kindCheck or mode == 'all' or kind == 'rtc':
			ret = 1
		else:
			ret = 0

		return ret

	def SetItemByDict(self, dict):
		"""ディクショナリを与えてItemをセット

		[引数]
		dict -- Item のディクショナリ

		[戻り値]
		void
		"""
		rootitem_id = self.GetRootItem()
		item_list = self.GetChildrenItemNameList(rootitem_id)
		self.depth = 0
		if dict != None:
			self.__SetItemRecursive__(rootitem_id, dict)
		else:
			self.DeleteChildren(rootitem_id)
			self.SelectItem(rootitem_id)

	def __SetItemRecursive__(self, item_id, dict):
		"""Item をディクショナリから再帰的にセットする

		[引数]
		item_id -- 現在のレベルのItemのID
		dict    -- Item の残りのディクショナリ

		[戻り値]
		void
		"""
		self.depth += 1
		check_disp = 0
		self.kindDispMode = self.parent.frame.kindDispMode
		# dict => {"name":(data, child_dict)}
		# items => {"name":(id, cookie)}
		items = self.GetChildrenItemNameList(item_id)

		# Append new item or new item data
		for dict_name in dict.keys():

			kind = dict[dict_name][0]['kind']
			check_disp = self.checkDispKind(kind)
			# If given item name of dictionary is already exist,
			# just replace with new ItemData.
			if dict_name in items.keys():
				if check_disp == 0:
					pass
				else:
					self.SetPyData(items[dict_name][0], dict[dict_name][0])
					new_item_id = items[dict_name][0]
			else:
				if check_disp == 1:
					new_item_id = self.AppendItem(item_id, dict_name)
					self.SetPyData(new_item_id, dict[dict_name][0])
					print dict_name + ":", self.depth
					if kind not in self.treeimg_list.keys():
						kind = 'unknown'
					try:
						self.SetItemImage(new_item_id, self.treeimg_list[kind][0],
									  wx.TreeItemIcon_Normal)
						self.SetItemImage(new_item_id, self.treeimg_list[kind][0],
									  wx.TreeItemIcon_Expanded)
					except:
						except_mess("except error!:")


			# Recursive call to children.
			if dict[dict_name][1] != None and check_disp == 1:
				self.__SetItemRecursive__(new_item_id, dict[dict_name][1])

		# Delete current item if is not in given dictionary
		for item_name in items.keys():
			if item_name in dict.keys():
				kind = dict[item_name][0]['kind']
				check_disp = self.checkDispKind(kind)
				if check_disp != 1:
					self.DeleteChildren(items[item_name][0])
					self.Delete(items[item_name][0])
			else:	# 辞書にないケース
				self.DeleteChildren(items[item_name][0])
				self.Delete(items[item_name][0])
		self.depth -= 1

	def GetChildrenItemIdList(self, parent_id):
		"""現在のItemID以下の子供のIDリストを取得する

		[引数]
		paren_id -- 現在のレベルのItemID

		[戻り値]
		items -- 子供のItemのリスト
		"""
		items = []
		if self.ItemHasChildren(parent_id):
			id, cookie = self.GetFirstChild(parent_id)
			items.append((id, cookie))

			for i in range(self.GetChildrenCount(parent_id, False) - 1):
				id, cookie = self.GetNextChild(parent_id, cookie)
				items.append((id, cookie))
		return items
		
	def GetChildrenItemNameList(self, parent_id):
		"""現在のItemID以下の子供のItemTextリストを取得する

		[引数]
		paren_id -- 現在のレベルのItemID

		[戻り値]
		items -- 子供のItemTextのリスト
		"""
		item_names = {}
		if self.ItemHasChildren(parent_id):
			id, cookie = self.GetFirstChild(parent_id)
			item_names[self.GetItemText(id)] = (id, cookie)

			for i in range(self.GetChildrenCount(parent_id, False) - 1):
				id, cookie = self.GetNextChild(parent_id, cookie)
				item_names[self.GetItemText(id)] = (id, cookie)
		return item_names
		
	def __ListupRecursive__(self, item_id):
		"""現在のItemID以下の子供のItemTextを再帰的に表示する

		[引数]
		item_id -- 現在のレベルのItemID

		[戻り値]
		void
		"""
		for id, cookie in self.GetChildrenItemIdList(item_id):
			if self.ItemHasChildren(id):
				print self.GetItemText(id)
				self.__ListupRecursive__(id)
			else:
				print self.GetItemText(id)


#-------------------------------------------------------------------------------
class RtmTreeCtrlPanel(wx.Panel):
	"""ネーミングツリーコントロールのパネルクラス"""
	def __init__(self, frame, parent, log):
		"""クラス初期化

		[引数]
		frame -- 親フレーム
		parent -- 親ウインドウ

		[戻り値]
		this
		"""		
		self.parent = parent
		self.myDict = frame.myDict
		self.threadloop = 1
		self.frame = frame
		# Use the WANTS_CHARS style so the panel doesn't eat the Return key.
		wx.Panel.__init__(self, parent, -1, style=wx.WANTS_CHARS)
		self.Bind(wx.EVT_SIZE, self.OnSize)

		self.log = log
		tID = wx.NewId()

		self.tree = RtmTreeCtrl(self, tID, wx.DefaultPosition, wx.DefaultSize,
								wx.TR_HAS_BUTTONS
								#| wx.TR_HAS_BUTTONS
								#| wx.TR_TWIST_BUTTONS 
								#| wx.TR_EDIT_LABELS
								#| wx.TR_MULTIPLE
								#| wx.TR_HIDE_ROOT
								, self.log)
		self.tree.Expand(self.tree.GetRootItem())

		self.NSHistory = RtmNSHistory()

		self.name_server = self.NSHistory.GetNSName()
		self.name_server_pre = self.NSHistory.GetNSName()

		self.nshelper = RtmNSHelper.RtmNSHelper()
		self.nshelper.Connect(self.name_server)
		self.tree.SetItemText(self.tree.root, "NS:"+self.name_server)


		self.Bind(wx.EVT_TREE_ITEM_EXPANDED, self.OnItemExpanded, self.tree)
		self.Bind(wx.EVT_TREE_ITEM_COLLAPSED, self.OnItemCollapsed, self.tree)
#		self.Bind(wx.EVT_TREE_SEL_CHANGED, self.OnSelChanged, self.tree)
		self.Bind(wx.EVT_TREE_SEL_CHANGING, self.OnSelChanging, self.tree)
#		self.Bind(wx.EVT_TREE_BEGIN_LABEL_EDIT, self.OnBeginEdit, self.tree)
#		self.Bind(wx.EVT_TREE_END_LABEL_EDIT, self.OnEndEdit, self.tree)
		self.Bind(wx.EVT_TREE_ITEM_ACTIVATED, self.OnActivate, self.tree)
		self.Bind(wx.EVT_RIGHT_UP, self.OnRightUp)
		self.tree.Bind(wx.EVT_TREE_BEGIN_DRAG, self.OnBeginDrag)
		self.tree.Bind(wx.EVT_TREE_END_DRAG, self.OnEndDrag)
		
		self.tree.Bind(wx.EVT_LEFT_DCLICK, self.OnLeftDClick)
		self.tree.Bind(wx.EVT_RIGHT_DOWN, self.OnRightClick)
		self.tree.Bind(wx.EVT_RIGHT_UP, self.OnRightUp)
#		self.tree.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)
		self.tree.Bind(wx.EVT_TREE_SEL_CHANGED, self.OnSelChanged)

		# タイマでリフレッシュするとmutexが必要なのでやめた
		#		self.Bind(wx.EVT_TIMER, self.OnRefresh)
		#		self.t1 = wx.Timer(self)
		#		self.t1.Start(2000)
		#		self.mutex = mutex.mutex()
		thread.start_new_thread(self.OnRefresh,())		


		self.comp_popup = RtmCompPopup(self)
		self.module_popup = RtmModulePopup(self)
		self.manager_popup = RtmManagerPopup(self)
		self.names_popup = RtmNameServerPopup(self)
		self.unknown_popup = RtmUnknownPopup(self)


	def makeDict(self):
		"""ツリー表示のためのディクショナリを生成

		[引数]
		void

		[戻り値]
		void
		"""		
		rootitem_id = self.tree.GetRootItem()
		fullpath = self.makeFullPath(rootitem_id)
		self.myDict.SetObjRefToFullpath(fullpath,None)
		self.__makeDictRecursive__(rootitem_id)

	def __makeDictRecursive__(self, item_id):
		"""ツリー表示のためのディクショナリを再帰的に生成
		[引数]
		item_id -- 現在のレベルのItemID

		[戻り値]
		void
		"""		
		for id, cookie in self.tree.GetChildrenItemIdList(item_id):
			if self.tree.ItemHasChildren(id):
				data = self.tree.GetItemData(id)
				comp = data.GetData()
				fullpath = self.makeFullPath(id)
				self.myDict.SetObjRefToFullpath(fullpath,comp)
				self.__makeDictRecursive__(id)
			else:
				data = self.tree.GetItemData(id)
				comp = data.GetData()
				fullpath = self.makeFullPath(id)
				self.myDict.SetObjRefToFullpath(fullpath,comp)

	def SetNameServer(self, name_server):
		"""ネームサーバのアドレス:ポートをセット

				[引数]
		name_server -- ネームサーバのアドレス:ポート

				[戻り値]
		void
		"""		
		self.name_server = namer_server

	def GetNameServer(self):
		"""ネームサーバのアドレス:ポートを取得
		
		[引数]
		void

		[戻り値]
		name_server -- ネームサーバのアドレス:ポート
		"""		
		return self.name_server

	def OnConnectNSClick(self, event):
		"""ネームサービスホスト:ポート設定ダイアログを表示

		[引数]
		event -- Event, 使用していない

		[戻り値]
		void
		"""		
		win = ConnectDialog(self, -1, "Naming Server", self.NSHistory.GetNSNames(),size=(400, 200),
					 #style = wxCAPTION | wxSYSTEM_MENU | wxTHICK_FRAME
					 style = wx.DEFAULT_DIALOG_STYLE
					 )
		win.SetNameServer(self.name_server)
		win.CenterOnScreen()
		val = win.ShowModal()
	
		if val == wx.ID_OK:
			self.name_server = win.GetNameServer().encode()
			self.tree.SetItemText(self.tree.root, "NS:"+self.name_server)
			self.tree.Expand(self.tree.root)
			self.NSHistory.SetNSName(self.name_server)

		win.Destroy()

	def OnRefresh(self):
		"""ネーミングツリーをリフレッシュ
		ループするので別スレッドから呼ぶべき

		[引数]
		void
		
		[戻り値]
		void
		"""	
		#		if not self.mutex.testandset():
		#			return
		#		thread.start_new_thread(self.__OnRefresh__, ())
		self.frame.close_evt.clear()
		while (self.threadloop):
 			dummy = 0
			self.myDict.Mutex.lock(self.Refresh,dummy)
			self.myDict.Mutex.unlock()
			time.sleep(1.0)
		self.frame.close_evt.set()

	def Refresh(self,dummy):
		"""ネーミングツリーをリフレッシュ

		[引数]
		void

		[戻り値]
		void
		"""	
		if self.name_server_pre != self.name_server or self.nshelper.root_cxt == None:
			self.nshelper.Connect(self.name_server)
			self.name_server_pre = self.name_server

		self.tree.SetItemByDict(self.nshelper.GetNSDict())
		self.tree.Update()
		self.makeDict()

	def OnRightClick(self, event):
#		print "OnRightClick"
		pass
#		pt = event.GetPosition();
#		item, flags = self.tree.HitTest(pt)
#		self.tree.SelectItem(item)

	def OnRightUp(self, event):
		"""ツリー右クリック時コンテキストメニュー表示

		[引数]
		event -- イベント

		[戻り値]
		void
		"""	
		try:
			pt = event.GetPosition();
			item, flags = self.tree.HitTest(pt)

			# 空白部分をクリック->無視
			if self.tree.GetItemText(item) == "":
				return
			cur_item = item
			
			fullpath = self.makeFullPath(cur_item)
			kind = self.myDict.GetKindToFullpath(fullpath)

			if (kind == 'ns'):
				self.names_popup.SetItem(item)
				self.PopupMenu(self.names_popup.GetMenu(),
							   event.GetPosition())
			elif (kind == 'mgr' or kind == 'mgr_cxt'):
				self.manager_popup.SetItem(item)
				self.manager_popup.SetSubMenu()
				self.PopupMenu(self.manager_popup.GetMenu(),
							   event.GetPosition())
			elif (kind == 'mod_cxt'):
				self.module_popup.SetItem(item)
				self.PopupMenu(self.module_popup.GetMenu(),
							   event.GetPosition())
			elif (kind == 'rtc'):
				self.comp_popup.SetItem(item)
				self.PopupMenu(self.comp_popup.GetMenu(),
							   event.GetPosition())
			else:	# (kind == 'unknown'): & cate_cxt
				self.unknown_popup.SetItem(item)
				self.PopupMenu(self.unknown_popup.GetMenu(),
							   event.GetPosition())
		except:
			except_mess("except error!:")
			pass
			
	def OnBeginEdit(self, event):
		self.log.WriteText("OnBeginEdit\n")
		# show how to prevent edit...
		if self.tree.GetItemText(event.GetItem()) == "The Root Item":
			wx.Bell()
			self.log.WriteText("You can't edit this one...\n")

			# Lets just see what's visible of its children
			cookie = 0
			root = event.GetItem()
			(child, cookie) = self.tree.GetFirstChild(root)

			while child.IsOk():
				self.log.WriteText("Child [%s] visible = %d" %
								   (self.tree.GetItemText(child),
									self.tree.IsVisible(child)))
				(child, cookie) = self.tree.GetNextChild(root, cookie)

			event.Veto()

	def OnEndEdit(self, event):
		self.log.WriteText("OnEndEdit\n")
		# show how to reject edit, we'll not allow any digits
		for x in event.GetLabel():
			if x in string.digits:
				self.log.WriteText("You can't enter digits...\n")
				event.Veto()
				return

	def OnLeftDClick(self, event):
		pt = event.GetPosition();
		item, flags = self.tree.HitTest(pt)
		self.log.WriteText("OnLeftDClick: %s\n" % self.tree.GetItemText(item))
		rootitem_id = self.tree.GetRootItem()
		parent = self.tree.GetItemParent(item)
		try:
			self.tree.SortChildren(parent)
		except:
			except_mess("invalid tree item!:")
		event.Skip()

	def OnSize(self, event):
		w,h = self.GetClientSizeTuple()
		self.tree.SetDimensions(0, 0, w, h)

	def OnItemExpanded(self, event):
		item = event.GetItem()
		self.log.WriteText("OnItemExpanded: %s\n" % self.tree.GetItemText(item))

	def OnItemCollapsed(self, event):
		item = event.GetItem()
		self.log.WriteText("OnItemCollapsed: %s\n" % self.tree.GetItemText(item))

	def OnSelChanged(self, event):
		self.item = event.GetItem()
		self.log.WriteText("OnSelChanged: %s\n" % self.tree.GetItemText(self.item))
		if wx.Platform == '__WXMSW__':
			self.log.WriteText("BoundingRect: %s\n" %
							   self.tree.GetBoundingRect(self.item, True))

	def OnSelChanging(self, event):
		self.item = event.GetItem()

	def OnActivate(self, event):
		pass

	def makeFullPath(self, item):
		"""現在のItemIdからFullPathを生成

		[引数]
		cur_id -- 現在のItemID

		[戻り値]
		ret -- ツリーのFull Path
		"""	
		cur_id = item
		ret = self.tree.GetItemText(item)
		if ret == '':
			print "item name is None!!: error!!"
			return
#		num = ret.find('|')
#		if num != -1:
#			ret = ret[0:num]
		rootitem_id = self.tree.GetRootItem()
		while (rootitem_id != cur_id):
			tmp_id = self.tree.GetItemParent(cur_id)
#			if tmp_id == rootitem_id:
#				break
			name = self.tree.GetItemText(tmp_id)
#			num = name.find('|')
#			if num != -1:
#				name = name[0:num]
			ret = name + '/' + ret
			cur_id = tmp_id
		return str(ret)

	def searchCategoryName(self, item):
		"""現在のItemIdからCategoryの名称を検索
		manager にぶら下がっているcategory名を検索

		[引数]
		cur_id -- 現在のItemID

		[戻り値]
		ret -- ツリーのFull Path
		"""	
		cur_id = item
		ret = ''
		cur_name = ''
		kind = ''
		items = []
		parent_id = item
		if self.ItemHasChildren(parent_id):
			cur_id, cookie = self.GetFirstChild(parent_id)

			for i in range(self.GetChildrenCount(parent_id, False) - 1):
				cur_name = self.tree.GetItemText(cur_id)
				num = cur_name.find('|')
				if num != -1:
					kind = cur_name[num+1:]
				else:
	#				print 'nothing kind!!! error!?'
					kind = ''
					break
				if kind == 'cate_cxt':
					ret = cur_name[0:num-1]
					break
				cur_id, cookie = self.GetNextChild(cur_id, cookie)

		return ret


	def searchManagerPath(self, item):
		"""現在のItemIdからManagerのFullPathを作成

		[引数]
		cur_id -- 現在のItemID

		[戻り値]
		ret -- ツリーのFull Path
		"""	
		cur_id = item
		ret = ''
		cur_name = ''
		kind = ''
		rootitem_id = self.tree.GetRootItem()
		# search manager_cxt(kind:mgr_cxt)
		while (rootitem_id != cur_id):
			cur_name = self.tree.GetItemText(cur_id)
			num = cur_name.find('|')
			if num != -1:
				kind = cur_name[num+1:]
			else:
#				print 'nothing kind!!! error!?'
				kind = ''
				break
			if kind == 'mgr_cxt':
				break
			tmp_id = self.tree.GetItemParent(cur_id)
			cur_id = tmp_id

		# search mgr kind
		tmp_id = cur_id
		cur_id = None
		if kind == 'mgr_cxt':
			kind = ''
			for id, cookie in self.tree.GetChildrenItemIdList(tmp_id):
				cur_name = self.tree.GetItemText(id)
				num = cur_name.find('|')
				if num != -1:
					kind = cur_name[num+1:]
				else:
#					print 'nothing kind!!! error!?'
					kind = ''
					break
				if kind == 'mgr':
					cur_id = id
					break

		if cur_id == None:
			return None

		ret = self.tree.GetItemText(cur_id)

		while (rootitem_id != cur_id):
			tmp_id = self.tree.GetItemParent(cur_id)
#			if tmp_id == rootitem_id:
#				break
			name = self.tree.GetItemText(tmp_id)
			ret = name + '/' + ret
			cur_id = tmp_id

		return ret

	def OnBeginDrag(self, event):
		"""DnDスタート

		[引数]
		event -- Event, 使用していない

		[戻り値]
		void
		"""	
		# Change selection when drag is starting
		item = event.GetItem()
		self.tree.SelectItem(item)
		fullpath = self.makeFullPath(item)

		kind = self.myDict.GetKindToFullpath(fullpath)

#		itemPt = event.GetPoint()
#		(checkID, checkFlag) = self.tree.HitTest(itemPt)
#		if checkFlag == wx.TREE_HITTEST_NOWHERE:
#			print "check ok!!: item now here.",itemPt,checkFlag

		if (kind != 'rtc'):
			return

		data = wx.DataObjectComposite()

		dragText =  fullpath
		textDataObj = wx.TextDataObject(dragText)
		data.Add(textDataObj)

		drop_source = wx.DropSource(self.tree)
		drop_source.SetData(data)
		result = drop_source.DoDragDrop()

		if result == wx.DragError:
			print "DragError"
		elif result == wx.DragNone:
			print "DragNone"
		elif result == wx.DragCopy:
			print "DragCopy: Sucseed"
		elif result == wx.DragMove:
			print "DragCopy: Sucseed"
		elif result == wx.DragCancel:
			print "DragCancel"
		else:
			print result

	def OnEndDrag(self, event):
		"""DnDエンド

		[引数]
		event -- Event, 使用していない

		[戻り値]
		void
		"""	
		item = event.GetItem()
		self.tree.SelectItem(item)
		print self.tree.GetItemText(item)

	def OnLeftUp(self, event):
		"""マウスの左ボタンクリック

		[引数]
		event -- Event, 使用していない

		[戻り値]
		void
		"""	
		item = self.tree.GetSelection()
		name = self.tree.GetItemText(item)
		fullpath = self.makeFullPath(item)
		kind = self.myDict.GetKindToFullpath(fullpath)
		if kind == "rtc":
			ref = self.myDict.GetObjRefToFullpath(fullpath)
			try:
				self.frame.profilepanel.RefreshProfile(ref.get_component_profile())
			except:
				except_mess("except error!:")
				pass

	def OnSelChanged(self, event):
		"""セレクトされた

		[引数]
		event -- Event, 使用していない

		[戻り値]
		void
		"""	
		item = event.GetItem()
		name = self.tree.GetItemText(item)
		if name == '':
#			print "OnSelChanged name is Non!"
			return
		fullpath = self.makeFullPath(item)
		kind = self.myDict.GetKindToFullpath(fullpath)
		if kind == "rtc":
			ref = self.myDict.GetObjRefToFullpath(fullpath)
			try:
				self.frame.profilepanel.RefreshProfile(ref.get_component_profile())
			except:
				except_mess("except error!:")
				pass


#-------------------------------------------------------------------------------
class ConnectDialog(wx.Dialog):
	""" ネーミングサービス接続ダイアログクラス"""
	def __init__(
		self, parent, ID, title, text_list,size=wx.DefaultSize, pos=wx.DefaultPosition, 
		style=wx.DEFAULT_DIALOG_STYLE
		):
		"""クラス初期化

		[引数]
		parent -- 親ウインドウ
		ID     -- ウインドウID
		title  -- ダイアログウインドウタイトル
		size   -- サイズ
		pos    -- 位置
		style  -- ダイアログウインドウスタイル

		[戻り値]
		this
		"""	
		pre = wx.PreDialog()
		pre.SetExtraStyle(wx.DIALOG_EX_CONTEXTHELP)
		pre.Create(parent, ID, title, pos, size, style)

		self.this = pre.this

		sizer = wx.BoxSizer(wx.VERTICAL)
		box = wx.BoxSizer(wx.HORIZONTAL)

		label = wx.StaticText(self, -1, "NameServer:port ")
		label.SetHelpText("Name server and port number")
		box.Add(label, 0, wx.ALIGN_CENTRE|wx.ALL, 5)

		self.hosttxt = wx.ComboBox(self, -1, "", (90,50),(160,-1), text_list,wx.CB_DROPDOWN)
		self.hosttxt.SetHelpText("Name server and port number")
		box.Add(self.hosttxt, 1, wx.ALIGN_CENTRE|wx.ALL, 5)

		sizer.AddSizer(box, 0, wx.GROW|wx.ALIGN_CENTER_VERTICAL|wx.ALL, 5)

		line = wx.StaticLine(self, -1, size=(20,-1), style=wx.LI_HORIZONTAL)
		sizer.Add(line, 0, wx.GROW|wx.ALIGN_CENTER_VERTICAL|wx.RIGHT|wx.TOP, 5)

		box = wx.BoxSizer(wx.HORIZONTAL)

		if wx.Platform != "__WXMSW__":
			btn = wx.ContextHelpButton(self)
			box.Add(btn, 0, wx.ALIGN_CENTER|wx.ALL, 5)

		btn = wx.Button(self, wx.ID_OK, " OK ")
		btn.SetDefault()
		btn.SetHelpText("The OK button completes the dialog")
		box.Add(btn, 0, wx.ALIGN_CENTER|wx.ALL, 5)

		btn = wx.Button(self, wx.ID_CANCEL, " Cancel ")
		btn.SetHelpText("The Cancel button cnacels the dialog. (Cool, huh?)")
		box.Add(btn, 0, wx.ALIGN_CENTER|wx.ALL, 5)

		sizer.Add(box, 0,
				  wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL|wx.ALL,
				  5)

		self.SetSizer(sizer)
		self.SetAutoLayout(True)
		sizer.Fit(self)
		
	def SetNameServer(self, name_server):
		"""ダイアログウインドウに予め表示するネーミングサーバ名

		[引数]
		name_server -- ネーミングサーバ名

		[戻り値]
		void
		"""	
		self.hosttxt.SetValue(name_server)

	def GetNameServer(self):
		"""ダイアログウインドウに入力されたネーミングサーバ名を取得する

		[引数]
		void

		[戻り値]
		name_server -- ネーミングサーバ名
"""
		return self.hosttxt.GetValue()
#---------------------------------------------------------------------------
class RtmNSHistory:
	""" ネーミングサービス接続ヒストリークラス"""
	def __init__(self):
		"""クラス初期化

		[引数]
		なし

		[戻り値]
		なし
		"""
		self.err = 0
		self.history = []

		self.filename = 'rtclink_nshist.dat'
		if os.name == 'posix':
			self.rootdir = os.getenv('HOME','.')
			self.filename = '.rtclink_nshist'
		elif os.name == 'nt':
			self.win_drive = os.getenv('HOMEDRIVE','C:')
			self.win_root = os.getenv('HOMEPATH','\\')
			self.rootdir = os.path.join(self.win_drive, self.win_root)
		else:
			self.rootdir = '.'

		self.hist_path = os.path.join(self.rootdir,self.filename)
		try:
			self.file = open(self.hist_path, 'rw')
		except:
			print "No sach file:",self.hist_path
			print "create to history-file:",self.hist_path
			try:
				self.file = open(self.hist_path, 'w')
				self.file.write('localhost\n')
				self.file.close()
				self.file = open(self.hist_path, 'rw')
			except:
				except_mess("except error!:")
				self.history.append('localhost')
				self.err = 1
				return

		tmp_hist = self.file.readlines()
		for hist in tmp_hist:
			hist = hist[0:-1]
			self.history.append(hist)
		self.file.close()

	def GetNSName(self):
		return self.history[0]

	def GetNSNames(self):
		return self.history

	def SetNSName(self,newName):
		if newName in self.history:
			indx = self.history.index(newName)
			del self.history[indx]
		self.history.insert(0,newName)
		n = 10
		if len(self.history) > 10:
			for n in range(10,len(self.history)):
				del self.history[n]
		self.file = open(self.hist_path, 'w')
		for hist in self.history:
			self.file.write(hist)
			self.file.write('\n')
		self.file.close()

#---------------------------------------------------------------------------

def runTest(frame, nb, log):
	win = RtmTreeCtrlPanel(nb, log)
	return win

#---------------------------------------------------------------------------




if __name__ == '__main__':
	import sys,os
	import run
	run.main(['', os.path.basename(sys.argv[0])] + sys.argv[1:])

