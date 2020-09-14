#/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  @file RtmProfileList.py
#  @brief rtc-link component profile list display class
#  @date $Date: 2007-01-21 13:21:20 $
#  @author Noriaki Ando <n-ando@aist.go.jp>
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

import wx
import rtimages

#----------------------------------------------------------------------
class RtmProfileListCtrl(wx.ListCtrl):
    def __init__(self, parent, ID, pos=wx.DefaultPosition,
                 size=wx.DefaultSize, style=0):
        wx.ListCtrl.__init__(self, parent, ID, pos, size, style)

        
class RtmProfileListPanel(wx.Panel):
    def __init__(self, parent):
        self.datamap = [
            ("Instance ID", ""),
            ("Impl. ID", ""),
            ("Maker", ""),
            ("Description", ""),
            ("Version", ""),
            ("Category", ""),
            ("Component Type" , ""),
            ("Activity Type", ""),
            ("Max Instance", ""),
            ("Language", ""),
            ("Lang. Type", ""),
            ("InPort Attr.", ""),
            ("    Name", ""),
            ("    Type", ""),
            ("OutPort Attr.", ""),
            ("    Name", ""),
            ("    Type", "")]
        
        wx.Panel.__init__(self, parent, -1, style=wx.WANTS_CHARS)

        self.il = wx.ImageList(16, 16)
        self.comp_bmp = self.il.Add(rtimages.getComponentBitmap())
        self.inp_bmp = self.il.Add(rtimages.getInPortBitmap())
        self.outp_bmp = self.il.Add(rtimages.getOutPortBitmap())
        self.blank_bmp = self.il.Add(rtimages.getBlankBitmap())

        tID = wx.NewId()
        self.list = RtmProfileListCtrl(self, tID, wx.DefaultPosition,
                                       (1000, 1000), #wx.DefaultSize,
                                       wx.LC_REPORT | wx.SUNKEN_BORDER
                                       | wx.LC_VRULES | wx.LC_HRULES)
        
        self.list.SetImageList(self.il, wx.IMAGE_LIST_SMALL)
        info = wx.ListItem()
        info.m_mask = wx.LIST_MASK_TEXT | wx.LIST_MASK_IMAGE | wx.LIST_MASK_WIDTH | wx.LIST_MASK_FORMAT
        info.m_text = "Attribute"
        info.m_image = -1
        info.m_width = 120
        info.m_format = wx.LIST_FORMAT_LEFT
        self.list.InsertColumnInfo(0, info)
        
        info.m_width = 100
        info.m_text = "Value"
        self.list.InsertColumnInfo(1, info)
        self.PopulateList()        

        self.Bind(wx.EVT_SIZE, self.OnSize)

    def PopulateList(self):

        x = 0
        for data in self.datamap:
            attr, val = data
            if attr == "Instance ID":
                self.list.InsertImageStringItem(x, attr, self.comp_bmp)
            elif attr== "InPort Attr.":
                self.list.InsertImageStringItem(x, attr, self.inp_bmp)
            elif attr== "OutPort Attr.":
                self.list.InsertImageStringItem(x, attr, self.outp_bmp)
            else:
                self.list.InsertImageStringItem(x, attr, self.blank_bmp)
            self.list.SetStringItem(x, 1, val)

            if attr == "InPort Attr.":
                print "InPort Attr."
                self.list.SetItemBackgroundColour(x,
                                                  wx.Color(0xdd, 0xdd, 0xdd))
            elif attr == "OutPort Attr.":
                print "OutPort Attr."
                self.list.SetItemBackgroundColour(x,
                                                  wx.Color(0xdd, 0xdd, 0xdd))
            x += 1

        # show how to select an item
        self.list.SetItemState(5, wx.LIST_STATE_SELECTED,
                               wx.LIST_STATE_SELECTED)

        self.currentItem = 0

    def RefreshProfile(self, profile):

        self.datamap = [
            ("Instance ID", profile.instance_name),
            ("Impl. ID",    profile.type_name),
            ("Maker",       profile.vendor),
            ("Description", profile.description),
            ("Version",     profile.version),
            ("Category",    profile.category)
#            ("Comp. Type",  profile.component_type.__str__()),
#            ("Act. Type",   profile.activity_type.__str__()),
#            ("Max Inst.",   profile.max_instance.__str__()),
#            ("Language",    profile.language),
#            ("Lang. Type",  profile.language_type.__str__())
            ]

#        print profile.component_type
#        print dir(profile.component_type)

        item_len = len(self.datamap)
#        in_prof = profile.inport_profile_list
#        for x in in_prof:
#            self.datamap.append(("InPort Attr.", "Value"))
#            self.datamap.append(("    Name", x.name))
#            self.datamap.append(("    Type", x.port_type.name()))
#        out_prof = profile.outport_profile_list
#        for x in out_prof:
#            self.datamap.append(("OutPort Attr.", "Value"))
#            self.datamap.append(("    Name", x.name))
#            self.datamap.append(("    Type", x.port_type.name()))
        self.list.DeleteAllItems()            
        self.PopulateList()

    def OnSize(self, event):
        w,h = self.GetClientSizeTuple()
        self.list.SetDimensions(0, 0, w, h)

#----------------------------------------------------------------------

def runTest(frame, nb, log):
    win = RtmProfileListPanel(frame)
    return win

#----------------------------------------------------------------------


overview = """\
This example demonstrates the ListCtrl's Virtual List features. A Virtual list
can contain any number of cells, but data is not loaded into the control itself.
It is loaded on demand via virtual methods <code>OnGetItemText(), OnGetItemImage()</code>,
and <code>OnGetItemAttr()</code>. This greatly reduces the amount of memory required
without limiting what can be done with the list control itself.
"""



if __name__ == '__main__':
    import sys,os
    import run
    run.main(['', os.path.basename(sys.argv[0])] + sys.argv[1:])

