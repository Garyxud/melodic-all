#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  @file RtmAbout.py
#  @brief rtc-link version declaration dialog class
#  @date $Date: 2005-05-12 09:06:19 $
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

import sys

import wx                  # This module uses the new wx namespace
import wx.html
import wx.lib.wxpTag

#---------------------------------------------------------------------------

class RtdAboutBox(wx.Dialog):
    text = '''
<html>
<body bgcolor="#eeeeee">
<center><table bgcolor="#ffffff" width="100%%" cellspacing="0"
cellpadding="0" border="1">
<tr>
    <td align="center">
    <h1>RTCLink %s</h1>
    Running on wxPython %s<br>
    </td>
</tr>
</table>

<p><b>RTCLink</b> is a RTComponent based system design tool that
component connection, assembly, etc on GUI.</p>

<p><b>RTCLink</b> is brought to you by <br>
<b>Task-Intelligence Research Group,</b><br>
<b>Intelligent System Research Institute,</b><br>
<b>National Institute of Advanced Industrial Science and Techonology (AIST) Japan,</b><br>
Copyright (c) 2004.</p>

<p>
<!--font size="-1">Please see <i>license.txt</i> for licensing information.</font-->
</p>

<p><wxp module="wx" class="Button">
    <param name="label" value="Okay">
    <param name="id"    value="ID_OK">
</wxp></p>
</center>
</body>
</html>
'''
    def __init__(self, parent):
        wx.Dialog.__init__(self, parent, -1, 'About the wxPython demo',)
        html = wx.html.HtmlWindow(self, -1, size=(420, -1))
        py_version = sys.version.split()[0]
        html.SetPage(self.text % ("0.1", wx.VERSION_STRING))
        btn = html.FindWindowById(wx.ID_OK)
        ##btn.SetDefault()
        ir = html.GetInternalRepresentation()
        html.SetSize( (ir.GetWidth()+25, ir.GetHeight()+25) )
        self.SetClientSize(html.GetSize())
        self.CentreOnParent(wx.BOTH)

#---------------------------------------------------------------------------



if __name__ == '__main__':
    app = wx.PySimpleApp()
    dlg = RtdAboutBox(None)
    dlg.ShowModal()
    dlg.Destroy()
    app.MainLoop()

