#!/usr/bin/env python
# -*- coding: utf-8 -*- 
#
#  @file RtmParser.py
#  @brief rtc-link XML connection list file parser
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

import sys,os
import string
from omniORB import CORBA
from xml.dom.minidom import parse
from xml.dom.minidom import getDOMImplementation

import RtmDtdValidator

#
# 
#
class RtmParser :
#  def serializeDOM(self, target, node) :

# XMLファイルの出力: DOMツリー → XML
  def dump2file(self, outfilename=None) :
    if outfilename:
      self.outfilename = outfilename
    if self.outfilename :
      print "outfilename:",self.outfilename
      self.outfile = open(self.outfilename, "w")
      self.outfile.write(self.cur_node.toxml())
      self.outfile.write("\n")
      self.outfile.close()
    else :
      none
    return 1

  def dump2cout(self) :
    print(self.cur_node.toxml())
    return 1

# root node の取得　？
  def get_root(self) :
    return self.the_root_node

  def get_cur_node(self) :
    return self.cur_node

  def set_cur_node(self, nd) :
    self.cur_node = nd
    return 1

  def reset_node(self) :
    self.cur_node = self.the_root_node
    return 1

  def getChildNodes(self) :
    self.node_list = self.cur_node.childNodes
    return self.node_list

  def getTaggedNodes(self, tg) :
    self.node_list = self.cur_node.getElementsByTagName(tg)
    return self.node_list

  def getText(self, nodes=None) :
    if nodes == None:
      nodes = self.cur_node.childNodes
    rc = ''
    for node in nodes:
      if node.nodeType == node.TEXT_NODE:
        rc = rc + node.data
    return rc

  def replaceText(self, txt) :
    tmp_node = self.dom.createTextNode(txt)
    self.cur_node.childNodes = [tmp_node]
    return 1

  def getTaggedText(self, tg) :
    nodes = self.cur_node.getElementsByTagName(tg)[0]
#    tmp_root = nodes[0]
#    nodes = tmp_root.childNodes
    rc = self.getText(nodes.childNodes)
    return rc
  
# child element の更新(replace)
  def replaceTaggedText(self, tg, txt) :
    nodes = self.cur_node.getElementsByTagName(tg)
    tmp_root = nodes[0]
    tmp_node = self.dom.createTextNode(txt)
    tmp_root.childNodes = [tmp_node]
    return 1

# child node の削除(remove)
  def removeChildNode(self, nd):
    self.cur_node.removeChild(nd)

# element,text を子ノードとして追加
  def appendChildTaggedText(self, tg, txt):
    tmp_elem = self.dom.createElement(tg)
    tmp_txt = self.dom.createTextNode(txt)
    tmp_elem.appendChild(tmp_txt)
    self.cur_node.appendChild(tmp_elem)
    return tmp_elem
    
# elementを子ノードとして追加
  def appendChildTagged(self, tg):
    tmp_elem = self.dom.createElement(tg)
    self.cur_node.appendChild(tmp_elem)
    return tmp_elem

#
# for C access
#
  def set_tmplist(self, lst) :
    self.tmp_node_list = lst
    return 1

  def elt_tmp(self, idx) :
    return self.tmp_node_list[idx];
#
# for C access (end)
#

  def getComponentNode(self, comp_key):
    # search for compornent node list
    comp_list = self.the_root_node.getElementsByTagName("rt_component")
    tmp = ''
    tmp1 = ''
    tmp2 = ''
    tmp3 = ''
    tmp4 = ''
    tmp5 = ''
    ret = None
    for comp_node in comp_list:

      tmp1 = comp_node.getElementsByTagName("host_name")[0]
      tmp1 = self.getText(tmp.childNodes)
      tmp2 = comp_node.getElementsByTagName("manager_name")[0]
      tmp2 = self.getText(tmp.childNodes)
      tmp3 = comp_node.getElementsByTagName("category")[0]
      tmp3 = self.getText(tmp.childNodes)
      tmp4 = comp_node.getElementsByTagName("module_name")[0]
      tmp4 = self.getText(tmp.childNodes)
      tmp5 = comp_node.getElementsByTagName("component_name")[0]
      tmp5 = self.getText(tmp.childNodes)

      tmp = tmp1 + "/" + tmp2 + "/" + tmp3 + "/" + tmp4 + "/" + tmp5

      if tmp == comp_key:
        break

    if tmp1 == '' or tmp2 == '' or tmp3 == '' or tmp4 == '' or tmp5 == '':
#       tmp1 == '.*' or tmp2 == '.*' or tmp3 == '.*' or tmp4 == '.*' or tmp5 == '.*' :
      print "Component Name is not an"
      print self.infilename
    else:
      self.cur_node = comp_node
      ret = self.cur_node
    return ret

  def getComponentNames(self):
    # search for compornent node list
    comp_list = self.the_root_node.getElementsByTagName("rt_component")
    ret = []
    save_node = self.get_cur_node()
    for comp_node in comp_list:
      self.set_cur_node(comp_node)
      tmp = self.getTaggedText("component_name")
      ret.append(tmp)
    self.set_cur_node(save_node)
    return ret

  def getComponentKeys(self):
    # search for compornent node list
    comp_list = self.the_root_node.getElementsByTagName("rt_component")
    ret = []
    save_node = self.get_cur_node()
    for comp_node in comp_list:
      self.set_cur_node(comp_node)
      tmp = self.getTaggedText("component_name")

      tmp1 = self.getTaggedText("host_name")
      tmp2 = self.getTaggedText("manager_name")
      tmp3 = self.getTaggedText("category")
      tmp4 = self.getTaggedText("module_name")
      tmp5 = self.getTaggedText("component_name")
      nam = tmp1 + "/" + tmp2 + "/" + tmp3 + "/" + tmp4 + "/" + tmp5

      ret.append(nam)
    self.set_cur_node(save_node)
    return ret


# XMLファイルの読込み: XML → DOMツリー → ディクショナリ
  def readXML(self, in_name):

    self.infilename = in_name

    self.dom = parse(self.infilename)
    self.the_root_node = self.dom
    self.reset_node()

    dict = {}
    dict['rtc'] = {}
    dict['line'] = []
    rtc_list = self.getTaggedNodes('rtc_list')[0]
    self.set_cur_node(rtc_list)
    comp_list = self.getChildNodes()
    for comp_node in comp_list:
      self.set_cur_node(comp_node)
      name = self.getTaggedText('name')
      name_elem =  self.getTaggedNodes('name')[0]
      x = string.atof(name_elem.getAttribute('x'))
      y = string.atof(name_elem.getAttribute('y'))
      rot = name_elem.getAttribute('rot')
#      print "Comp name:",name," x=",x," y=",y
      dict['rtc'][name] = {}
      dict['rtc'][name]['x'] = x
      dict['rtc'][name]['y'] = y
      dict['rtc'][name]['rot'] = rot

    self.reset_node()
    line_list = self.getTaggedNodes('line_list')[0]
    self.set_cur_node(line_list)
    lines = self.getChildNodes()
    for line_node in lines:
      self.set_cur_node(line_node)
      pos = self.getTaggedText('pos')
      inp_comp = self.getTaggedText('inp-comp')
      inp_name = self.getTaggedText('inp-name')
      outp_comp = self.getTaggedText('outp-comp')
      outp_name = self.getTaggedText('outp-name')
      tmp = {}
      tmp['pos'] = pos
      tmp['in-comp'] = inp_comp
      tmp['in-name'] = inp_name
      tmp['out-comp'] = outp_comp
      tmp['out-name'] = outp_name
      dict['line'].append(tmp)

    return dict


  def writeXML(self, filename, dict):
    #XML document create
    impl = getDOMImplementation()
    doc = impl.createDocument(None, "rt_system", None)
    top_elem = doc.documentElement
#    LF = doc.createTextNode('\n')
#    top_elem.appendChild(LF)

    #XML node append
    #component block
    comp_list = dict['rtc'].keys()
    elemRtcList = doc.createElement("rtc_list")
    top_elem.appendChild(elemRtcList)
    for comp_name in comp_list:
      elem = doc.createElement("rt_component")
      elemRtcList.appendChild(elem)
      elemName = doc.createElement("name")
      elem.appendChild(elemName)
      child = doc.createTextNode(comp_name)
      elemName.appendChild(child)
      elemName.setAttribute('x','%f'%dict['rtc'][comp_name]['x'])
      elemName.setAttribute('y','%f'%dict['rtc'][comp_name]['y'])
      elemName.setAttribute('rot',dict['rtc'][comp_name]['rot'])

    elemLineList = doc.createElement("line_list")
    top_elem.appendChild(elemLineList)
    line_list = dict['line']
    for line in line_list:
      elem = doc.createElement("line")
      elemLineList.appendChild(elem)
      str = ''
      for pos in line['pos']:
        str += '%f,%f,'%(pos[0],pos[1])
#      str[-1] = ''
      elemPos = doc.createElement("pos")
      elem.appendChild(elemPos)
      child = doc.createTextNode(str)
      elemPos.appendChild(child)

      elemInComp = doc.createElement("inp-comp")
      elemInName = doc.createElement("inp-name")
      elem.appendChild(elemInComp)
      elem.appendChild(elemInName)
      child = doc.createTextNode(line['in-comp'])
      elemInComp.appendChild(child)
      child = doc.createTextNode(line['in-name'])
      elemInName.appendChild(child)

      elemOutComp = doc.createElement("outp-comp")
      elemOutName = doc.createElement("outp-name")
      elem.appendChild(elemOutComp)
      elem.appendChild(elemOutName)
      child = doc.createTextNode(line['out-comp'])
      elemOutComp.appendChild(child)
      child = doc.createTextNode(line['out-name'])
      elemOutName.appendChild(child)


    #dump to file
    outfile = open(filename, "w")
    doc.writexml(outfile)
    outfile.close()

    doc.unlink()

  def __init__(self) :
    self.infilename = None
    self.outfilename = None
#----------------------------------------------------------------------
