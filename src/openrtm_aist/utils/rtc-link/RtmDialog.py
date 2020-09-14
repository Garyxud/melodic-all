#!/usr/bin/env python
# -*- coding: utf-8 -*- 
#
#  @file RtmDialog.py
#  @brief rtc-link dialog class
#  @date $Date: 2005-05-16 10:11:01 $
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

# RtmDialog.py               Created on: 2005/01/20
#                            Author    : Tsuyoshi Tanabe


"""
    wxPython を用いたダイアログ・モジュール
"""

import  wx
import  sys

#---------------------------------------------------------------------------
# Create and set a help provider.  Normally you would do this in
# the app's OnInit as it must be done before any SetHelpText calls.
provider = wx.SimpleHelpProvider()
wx.HelpProvider_Set(provider)

#---------------------------------------------------------------------------

class RtmDialog(wx.Dialog):
    """ダイアログ表示用クラス"""
    def __init__(
            self, parent, ID, message, size=wx.DefaultSize, pos=wx.DefaultPosition
            ):
        """初期化。ダイアログの作成を行う。

        [引数]
        parent  -- 親Window
        ID      -- window ID
        message -- 表示するメッセージ
        size    -- dialog のサイズ
        pos     -- dialog の位置

        [戻り値]
        void
        """

        style=wx.DEFAULT_DIALOG_STYLE
        title="Ask Dialog"

        # Instead of calling wx.Dialog.__init__ we precreate the dialog
        # so we can set an extra style that must be set before
        # creation, and then we create the GUI dialog using the Create
        # method.

        pre = wx.PreDialog()
#        pre.SetExtraStyle(wx.DIALOG_EX_CONTEXTHELP)
        pre.Create(parent, ID, title, pos, size, style)

        # This next step is the most important, it turns this Python
        # object into the real wrapper of the dialog (instead of pre)
        # as far as the wxPython extension is concerned.
        self.this = pre.this

        # Now continue with the normal construction of the dialog
        # contents
        sizer = wx.BoxSizer(wx.VERTICAL)

        label = wx.StaticText(self, -1, message)
        sizer.Add(label, 0, wx.ALIGN_CENTRE|wx.ALL, 5)

        line = wx.StaticLine(self, -1, size=(20,-1), style=wx.LI_HORIZONTAL)
        sizer.Add(line, 0, wx.GROW|wx.ALIGN_CENTER_VERTICAL|wx.RIGHT|wx.TOP, 5)

        box = wx.BoxSizer(wx.HORIZONTAL)

        btn = wx.Button(self, wx.ID_OK, " OK ")
        btn.SetDefault()
        box.Add(btn, 0, wx.ALIGN_CENTRE|wx.ALL, 5)

        btn = wx.Button(self, wx.ID_CANCEL, " Cancel ")
        box.Add(btn, 0, wx.ALIGN_CENTRE|wx.ALL, 5)

        sizer.Add(box, 0, wx.ALIGN_CENTER_HORIZONTAL|wx.ALL, 5)

        self.SetSizer(sizer)
        self.SetAutoLayout(True)
        sizer.Fit(self)

#---------------------------------------------------------------------------

def except_mess(mess):
    print "traceback code:",sys.exc_traceback.tb_frame.f_code
    print "except line:",sys.exc_traceback.tb_lineno
    print mess, sys.exc_type, sys.exc_value
