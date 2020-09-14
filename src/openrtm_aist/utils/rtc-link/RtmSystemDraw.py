#!/usr/bin/env python
# -*- coding: utf-8 -*- 
#
#  @file RtmSystemDraw.py
#  @brief rtc-link component block diagram system draw management class
#  @date $Date: 2005-05-27 15:51:31 $
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
# RtmSystemDraw.py                    Created on: 2004/09/13
#                            Author    : Tsuyoshi Tanabe

"""
    wxPython, OGL を用いたコンポーネント図形表示画面
"""


import time
import  wx
import wx.lib.colourdb
import  wx.ogl  as  ogl
import RtmLineUtil as lu
import string
import sys,os
import copy
import RtmParser
from RtmDialog import *

import RTM


##wx.Trap()

# ソース内で使用するデファイン
# 色名称は、wx の colourdb パッケージ内のものを使用
SELECTED_COLOR = "LIGHT BLUE"
UNLOADED_COLOR = "black"
INACTIVE_COLOR = "blue"
ACTIVE_COLOR = "green"
ERROR_COLOR = "red"
TEXT_COLOR = "red"
#BACK_COLOR = "LightYellow"
BACK_COLOR = "WHITE"
MARK_COLOR = "red"
OUTLINE_COLOR = "red"
VIRTUAL_COLOR = "WHITE"

TRUE = 1
FALSE = 0
DUP_CONNECT = -1 # Duplication connection

BOX_WIDTH = 50
POLYGON_SIZE = 12

USE_BUFFERED_DC = 1

# バッググランド上でのコンテキストメニュー用文字列
strDEL_SELECT = "Delete Selected Item"
strREFRESH    = "Refresh"
strOPEN       = "Open System"
strSAVE       = "Save System"
strSAVE_AS    = "Save System As"
strDEL_SYS    = "Current System is Deleted when OPEN.\nDelete It?"

# コンポーネント上でのコンテキストメニュー用文字列
strSTART   = "Start"
strSTOP    = "Stop"
strRESET   = "Reset"
strEXIT    = "Exit"
strKILL    = "Kill"
strDELITEM = "Delete Item"

# アセンブリ・読み込み後のコンテキストメニュー
strASM_CONNECT = "Connect"
strASM_DELETE  = "Delete"

# 再接続処理の確認ダイアログ用文字列
strASKMESSAGE = "Old Connection Information was found.\nDelete it and reconnect?"
# 訳： 古い接続情報がコンポーネント上に見つかりました。古い接続は削除されます。再接続しますか？

# Save File As ダイアログのタイトル 
strSAVE_AS_TITLE = "Save file as ..."
# Open FIle ダイアログのタイトル 
strOPEN_TITLE = "Open a file"

#----------------------------------------------------------------------
class MyTextDropTarget(wx.TextDropTarget):
    """ドラッグ＆ドロップ：コンポーネントのツリー画面からテキストデータを受け取るクラス"""
    def __init__(self, parent, log):
        """クラスの初期化（TextDropTargetの作成）

        [引数]
        parent -- 親クラス
        log    -- ログ出力用クラス（wx.LogMessageのラッパー）

        [戻り値]
        void
        """
        wx.TextDropTarget.__init__(self)
        self.parent = parent
        self.log = log

    def OnDropText(self, x, y, text):
        """ドロップ機能のイベントハンドラ
        別ウィンドウからのドロップ操作で文字列を受け取る

        [引数]
        x      -- マウスカーソルのｘ座標
        y      -- マウスカーソルのｙ座標
        text   -- ドロップされた文字列（現時点はコンポーネント名）

        [戻り値]
        void
        """
        self.log.WriteText("(%d, %d)\n%s\n" % (x, y, text))
        canvas = self.parent.diagram.GetCanvas()
        dc = wx.ClientDC(self.parent)
        canvas.PrepareDC(dc)
        self.parent.PrepareDC(dc)

        # NameService Treeからドロップされた名称を登録
        if text not in self.parent.rtc_dict.keys():

#            print "drag text:",text , " kind:",check
            ref = self.parent.frame.myDict.GetObjRefToFullpath(text)
            kind = self.parent.frame.myDict.GetKindToFullpath(text)
            if ref != None and kind == 'rtc':
                self.parent.rtc_dict[text] = GRtc(self.parent, text, x, y)
                self.parent.rtc_list.append(text)
                self.parent.rtc_dict[text].refresh()
                self.parent.remakeLines()
            else:
                print "error: Drag Item does not obj-ref!"

        self.parent.Redraw(dc)

    def OnDragOver(self, x, y, d):
        """ドラッグ通知？イベントハンドラ
        マウスカーソルがターゲット上に来たら呼び出される

        [引数]
        x      -- マウスカーソルのｘ座標
        y      -- マウスカーソルのｙ座標
        d      -- SHIFT or CONTROL 押下時のフラグ

        [戻り値]
        wxDragResult -- ドラッグの状態を（システムに？）通知する
        """
        return wx.DragCopy

#----------------------------------------------------------------------
def getBufferedDC(canvas):
    """メモリDC（BufferedDC）を設定、取得する関数

    [引数]
    canvas      -- キャンバス

    [戻り値]
    dc         -- BufferedDC
    """
    cdc = wx.ClientDC(canvas)
    canvas.PrepareDC(cdc)
    bufSize = wx.Size(1000, 1000)
    dc = wx.BufferedDC(cdc, bufSize)
    canvas.PrepareDC(dc)
    dc.SetBackground(wx.Brush(canvas.GetBackgroundColour()))
    dc.Clear()
    return dc
#----------------------------------------------------------------------
def setBodyColor(shape, colorFlag):
    """コンポーネントの状態で図形の色を変更する関数

    [引数]
    shape      -- 図形(Shape)オブジェクト
    colorFlag  -- コンポーネントの状態を示す文字列
                  select, unloaded, inactive, active, error, virtual

    [戻り値]
    void
    """
    if colorFlag == 'select':
        shape.SetBrush(wx.Brush(wx.NamedColor(SELECTED_COLOR)))
    elif colorFlag == 'unloaded':
        shape.SetBrush(wx.Brush(wx.NamedColor(UNLOADED_COLOR)))
    elif colorFlag == 'inactive':
        shape.SetBrush(wx.Brush(wx.NamedColor(INACTIVE_COLOR)))
    elif colorFlag == 'active':
        shape.SetBrush(wx.Brush(wx.NamedColor(ACTIVE_COLOR)))
    elif colorFlag == 'error':
        shape.SetBrush(wx.Brush(wx.NamedColor(ERROR_COLOR)))
    elif colorFlag == 'virtual':
        shape.SetBrush(wx.Brush(wx.NamedColor(VIRTUAL_COLOR)))
    else:
        shape.SetBrush(wx.Brush(colorFlag))

    shape.Flash()

#----------------------------------------------------------------------
class makeCompositeShape(ogl.CompositeShape):
    """CompositeShapeのラッパークラス"""
    def __init__(self, parent):
        """クラスの初期化（CompositeShapeの作成）

        [引数]
        parent -- 親クラスを指定

        [戻り値]
        void
        """
        ogl.CompositeShape.__init__(self)
        self.parent = parent

#----------------------------------------------------------------------
class makeLineShape(ogl.LineShape):
    """２点間の線を描画するラッパークラス"""
    def __init__(self, parent, canvas ):
        """クラスの初期化（LineShapeの作成）

        [引数]
        parent -- 親クラスを指定
        canvas -- ShapeCanvasオブジェクトを指定。

        [戻り値]
        void
        """
        ogl.LineShape.__init__(self)
        self.parent = parent
        self.SetCanvas(canvas)
        self.SetPen(wx.Pen(wx.BLUE, 1))
        setBodyColor(self, 'inactive')
        self.MakeLineControlPoints(2)
        diagram = canvas.GetDiagram()
        diagram.AddShape(self)

    def setPoints(self, startX,startY, endX, endY):
        """開始、終了座標を指定し線を作成する

        [引数]
        startX -- 線を描画開始するｘ座標
        startY -- 線を描画開始するｙ座標
        endX   -- 線を描画終了するｘ座標
        endY   -- 線を描画終了するｙ座標

        [戻り値]
        void
        """
        self.SetEnds(startX, startY, endX, endY)

#----------------------------------------------------------------------
class makeRectangle(ogl.RectangleShape):
    """四角形を描画するラッパークラス"""
    def __init__(self, parent, width, height):
        """クラスの初期化（Rectangle作成）

        [引数]
        parent -- 親クラスを指定
        width  -- 四角形の幅
        height -- 四角形の高さ

        [戻り値]
        void
        """
        ogl.RectangleShape.__init__(self,width, height)
        self.parent = parent
        # lastx,lasty は、イベント（移動、拡大等）後の座標を格納、移動量や拡大比率で使用
        self.lastx = 0
        self.lasty = 0

#----------------------------------------------------------------------
class makeInportPolygon(ogl.PolygonShape):
    """インポート図形(polygon)描画用クラス"""
    def __init__(self, parent, points):
        """クラスの初期化(PolygonShapeの作成）

        [引数]
        parent -- 親クラスを指定
        points -- タプルで連続した(x,ｙ)座標(wxPoints型）を指定

        [戻り値]
        void
        """
        ogl.PolygonShape.__init__(self)
        self.parent = parent

        self.Create(points)
        self.CalculatePolygonCentre()

    def updateInportPolygon(self, points):
        """インポート図形(polygon)の再描画（座標再指定）

        [引数]
        points -- タプルで連続した(x,ｙ)座標(wxPoints型）を指定

        [戻り値]
        void
        """
        self.Create(points)
        self.UpdateOriginalPoints()
        self.CalculatePolygonCentre()

#----------------------------------------------------------------------
class makeOutportPolygon(ogl.PolygonShape):
    """アウトポート図形(polygon)描画用クラス"""
    def __init__(self, parent, points):
        """クラスの初期化(PolygonShapeの作成）

        [引数]
        parent -- 親クラスを指定
        points -- タプルで連続した(x,ｙ)座標(wxPoints型）を指定

        [戻り値]
        void
        """
        ogl.PolygonShape.__init__(self)
        self.parent = parent

        self.Create(points)
        self.CalculatePolygonCentre()

    def updateOutportPolygon(self, points):
        """アウトポート図形(polygon)の再描画（座標再指定）

        [引数]
        points -- タプルで連続した(x,ｙ)座標(wxPoints型）を指定

        [戻り値]
        void
        """
        self.Create(points)
        self.UpdateOriginalPoints()
        self.CalculatePolygonCentre()

#----------------------------------------------------------------------
class makeRectOval(ogl.EllipseShape):
    """楕円図形を生成するクラス"""
    def __init__(self, parent, pos_x, pos_y, width, height):
        """クラスの初期化(EllipseShapeの作成）

        [引数]
        parent -- 親クラスを指定
        pos_x  -- 描画するｘ座標
        pos_y  -- 描画するｙ座標
        width  -- 楕円の幅
        height -- 楕円の高さ

        [戻り値]
        void
        """
        ogl.EllipseShape.__init__(self, width, height)
        self.parent = parent
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.lastx = pos_x
        self.lasty = pos_y
        self.SetX(pos_x)
        self.SetY(pos_y)
        self.SetPen(wx.Pen(wx.BLACK, 1))
        self.SetBrush(wx.Brush('red'))

#----------------------------------------------------------------------
class makeTextShape(ogl.TextShape):
    """テキストを生成するクラス"""
    def __init__(self, parent, width, height):
        """クラスの初期化（TextShapeの作成）

        [引数]
        parent -- 親クラスを指定する
        width  -- テキスト描画エリアの幅
        height -- テキスト描画エリアの高さ

        [戻り値]
        void
        """
        ogl.TextShape.__init__(self,width, height)
        self.parent = parent

#----------------------------------------------------------------------
class makeToolTip(ogl.Shape):
    """ツールチップ（バルーンヘルプ）図形を生成するクラス"""
    def __init__(self,parent,pt,dc):
        """クラスの初期化（ツールチップの作成）

        [引数]
        parent -- 親クラスを指定する
        pt     -- ツールチップを表示する座標（x,y）のタプルで指定
        dc     -- 描画するデバイス・コンテキストを指定

        [戻り値]
        void
        """
        self.parent = parent
        self.body = None
        self.x_size = 0
        self.y_size = 0
        self.color = BACK_COLOR

        # 表示文字列の設定
        if parent.tag == 'in':
            string1 = parent.inport['name']
            string2 = parent.inport['port_type']
        else:
            string1 = parent.outport['name']
            string2 = parent.outport['port_type']
        atr = '%s\n%s'%(string1, string2)

        tmp = max(len(string1), len(string2))
        # カレントのフォントサイズ取得：Widthは平均値なので注意
        charW = dc.GetCharWidth()
        charH = dc.GetCharHeight()
        self.x_size = charW * tmp 
        self.y_size = charH * 2.5
        # Rectangleに文字列を表示させツールチップの代用とする
        self.body = makeRectangle(self, self.x_size, self.y_size)
        self.body.AddText(atr)
#        self.body.SetFormatMode(ogl.FORMAT_NONE,0)
        self.body.FormatText(dc,atr,0)
        self.body.SetDraggable(False, False)
        # 表示位置は、四角形の左下の頂点をマウスカーソルの位置に設定
        self.body.SetX(pt[0]+self.x_size/2)
        self.body.SetY(pt[1]-self.y_size/2)
        self.body.SetPen(wx.Pen(wx.RED, 1))
        self.body.SetBrush(wx.Brush(wx.NamedColor(BACK_COLOR)))

    def removeWidget(self,dc):
        """ツールチップ図形をキャンバス、DC上から削除

        [引数]
        dc     -- 描画されているデバイス・コンテキストを指定

        [戻り値]
        void
        """
        canvas = self.body.GetCanvas()
        self.body.Erase(dc)
        self.body.RemoveFromCanvas(canvas)

#----------------------------------------------------------------------
class GRectOval(ogl.Shape):
    """線の移動用の円を生成するクラス"""
    def __init__(self,parent,tag,pos_x, pos_y):
        """クラスの初期化（円を作成）

        [引数]
        parent -- 親クラスを指定する
        tag    -- 識別子（連番,線との関係を表すフラグ）を指定
                  ※連番は、線を格納する配列の添え字と連係している
        pos_x  -- 表示するｘ座標を指定
        pos_y  -- 表示するｙ座標を指定

        [戻り値]
        void
        """
        self.parent = parent
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.tag = tag
        self.createWidget()

    def createWidget(self):
        """円を生成

        [引数]
        なし

        [戻り値]
        void
        """
        self.body = makeRectOval(self, self.pos_x, self.pos_y, 8, 8)

    def removeWidget(self, dc):
        """円をキャンバス、DC上から削除

        [引数]
        dc     -- 描画するデバイス・コンテキストを指定

        [戻り値]
        void
        """
        canvas = self.body.GetCanvas()
        self.body.Erase(dc)
        self.body.RemoveFromCanvas(canvas)

    def dmove(self, dc, d_x, d_y):
        """円及び線の移動

        [引数]
        dc     -- 描画するデバイス・コンテキストを指定
        d_x    -- ｘ座標の相対移動量 (endPoint.x - startPoint.x の値）
        d_y    -- ｙ座標の相対移動量 (endPoint.y - startPoint.y の値）

        [戻り値]
        void
        """
        canvas = self.body.GetCanvas()

        line = self.parent
        oval_tag = self.getTag()
        oval_id = oval_tag[0]

        # 移動前の図形をDC上から削除
        self.body.Erase(dc)

        # 移動後の座標算出
        if oval_tag[1] == 'oval_width_pos':
            self.pos_x = self.body.GetX() + d_x
            self.pos_y = self.body.GetY()
        else:
            self.pos_x = self.body.GetX()
            self.pos_y = self.body.GetY() + d_y

        # 新しい座標で線の再作成
        line.lines[oval_id].Move(dc, self.pos_x, self.pos_y)
        x1,y1,x2,y2 = line.lines[oval_id].GetEnds()
        line.changeCoordT(oval_id, (x1, y1), (x2, y2) )

        self.body.Move(dc, self.pos_x, self.pos_y)

        # 円の移動（座標の再計算）
        for x in range(1,len(line.oval_dict)+1):
            tag = line.oval_dict[x].getTag()
            if oval_id != tag[0]:
                line_pos_0 = line.coordT[x]
                line_pos_1 = line.coordT[x+1]

                if tag[1] == 'oval_width_pos':
                    hight = line_pos_0[1] - line_pos_1[1]
                    pos_y = line_pos_1[1] + (hight/2)
                    pos_x = line_pos_0[0]
                else:
                    width = line_pos_0[0] - line_pos_1[0]
                    pos_x = line_pos_1[0] + (width/2)
                    pos_y = line_pos_1[1]

                line.oval_dict[x].body.Move(dc, pos_x, pos_y)

    def getTag(self):
        """タグの取得

        [引数]
        なし

        [戻り値]
        tag    -- 識別子（連番,線との関係を表すフラグ）を返却
                  ※連番は、線を格納する配列の添え字と連係している
        """
        return self.tag

#----------------------------------------------------------------------
class GRtcLine(ogl.Shape):
    """線を生成するクラス"""
    def __init__(self, canvas, parent):
        """クラスの初期化

        [引数]
        canvas -- 描画するキャンバスを指定
        parent -- 親クラスを指定する

        [戻り値]
        void
        """
        self.canvas = canvas
        self.parent = parent
        self.startx = 0
        self.starty = 0
        self.endx = 0
        self.endy = 0
        self.coordT = None
        self.g_inp = None
        self.g_outp = None
        self.idx = 'L' + `canvas.line_idx`
        self.curOvalObj = None    # Value is setup GWorld.leftDown()
        self.oval_dict  = {}
        self.tag = 'line'
        self.lines = []
        self.subscription_type = RTM.OPS_NEW
        self.profile = None

    def refresh(self):
        """リフレッシュ処理
        線及び移動用の円を非選択状態にする

        [引数]
        なし

        [戻り値]
        void
        """
        canvas = self.body.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        self.unselected(dc)

    def remove(self, dc, canvas):
        """線および移動用の円をキャンバス、DC上から削除する

        [引数]
        dc     -- 描画されているデバイス・コンテキストを指定
        canvas -- 描画されているキャンバスを指定

        [戻り値]
        void
        """
        for x in range(len(self.lines)):
            self.lines[x].Unlink()
            self.lines[x].Erase(dc)
            self.lines[x].DeleteControlPoints()
            self.lines[x].RemoveFromCanvas(canvas)

        for x in range(len(self.oval_dict)):
            self.oval_dict[x+1].removeWidget(dc)

    def removeWidget(self, dc):
        """線の削除
        関連するInport/Outportの情報(色、unsubscribe)の更新処理を呼び出す

        [引数]
        dc     -- 描画するデバイス・コンテキストを指定

        [戻り値]
        void
        """
        if self.g_inp:
            if len(self.g_inp.line_idx) == 1:
                setBodyColor(self.g_inp.body, 'inactive')
        if self.g_outp:
            if len(self.g_outp.line_idx) == 1:
                setBodyColor(self.g_outp.body, 'inactive')
        canvas = self.lines[0].GetCanvas()
        if self.g_outp != None:
            self.g_outp.disconnect(self.idx)
            self.g_outp = None
        if self.g_inp != None:
            self.g_inp.disconnect(self.idx)
            self.g_inp = None
        self.remove(dc, canvas)


    def createWidget(self):
        """線の生成

        [引数]
        なし

        [戻り値]
        void
        """
        num = len(self.coordT)
        if num < 2:
            return

        if num == 2: # ２点間の線
            self.lines.append(makeLineShape(self, self.canvas))
            self.lines[0].setPoints(self.startx, self.starty, self.endx, self.endy)
        else:        # 折線（頂点が２つ以上）
            for cnt in range(num-1):
            
                self.lines.append(makeLineShape(self, self.canvas))
                self.lines[cnt].setPoints(self.coordT[cnt][0], self.coordT[cnt][1], self.coordT[cnt+1][0], self.coordT[cnt+1][1])


        # イベント割付
        for x in range(len(self.lines)):
            setBodyColor(self.lines[x], 'inactive')
            evthandler2 = MyEvtHandlerLine()
            evthandler2.SetShape(self.lines[x])
            evthandler2.SetPreviousHandler(self.lines[x].GetEventHandler())
            self.lines[x].SetEventHandler(evthandler2)

    def setPoints(self, startX,startY, endX, endY):
        """線の座標設定

        [引数]
        startX -- 描画開始位置のｘ座標
        startY -- 描画開始位置のｙ座標
        endtX  -- 描画終了位置のｘ座標
        endtY  -- 描画終了位置のｙ座標

        [戻り値]
        void
        """

        lineUtil = lu.LineUtil(self, self.g_inp, self.g_outp, startX, startY, endX, endY)
        self.coordT = lineUtil.drawLine()

        self.startx = startX
        self.starty = startY
        self.endx = endX
        self.endy = endY

        self.createWidget()


    def setStartPoint(self, dc, movex,movey):
        """線の開始点を再設定（開始点の移動）

        [引数]
        dc     -- 描画するデバイス・コンテキストを指定
        movex  -- 開始点ｘ座標の相対移動量
        movey  -- 開始点ｙ座標の相対移動量

        [戻り値]
        void
        """
        canvas = self.lines[0].GetCanvas()
        # 図形をキャンバス、DC上から削除
        for x in range(len(self.lines)):
            self.lines[x].Erase(dc)
            self.lines[x].RemoveFromCanvas(canvas)
        for x in range(len(self.oval_dict)):
            self.oval_dict[x+1].removeWidget(dc)

        # 座標を再設定し線を生成
        self.lines = []
        self.startx = self.startx + movex
        self.starty = self.starty + movey
        self.setPoints(self.startx, self.starty, self.endx, self.endy)
        for x in range(len(self.lines)):
            self.lines[x].Show(True)

    def setEndPoint(self, dc, shape, movex,movey):
        """線の終了点を再設定（終了点の移動）

        [引数]
        dc     -- 描画するデバイス・コンテキストを指定
        movex  -- 終了点ｘ座標の相対移動量
        movey  -- 終了点ｙ座標の相対移動量

        [戻り値]
        void
        """
        canvas = self.lines[0].GetCanvas()
        # 図形をキャンバス、DC上から削除
        for x in range(len(self.lines)):
            self.lines[x].Erase(dc)
            self.lines[x].RemoveFromCanvas(canvas)
        for x in range(len(self.oval_dict)):
            self.oval_dict[x+1].removeWidget(dc)

        # 座標を再設定し線を生成
        self.lines = []
        self.endx = self.endx + movex
        self.endy = self.endy + movey
        self.setPoints(self.startx, self.starty, self.endx, self.endy)
        for x in range(len(self.lines)):
            self.lines[x].Show(True)

    def selected(self):
        """線の選択処理（色の変更）

        [引数]
        なし

        [戻り値]
        void
        """
        for x in range(len(self.lines)):
            self.lines[x].SetBrush(wx.Brush(wx.NamedColor(SELECTED_COLOR)))
            self.lines[x].SetPen(wx.Pen(SELECTED_COLOR, 1))
            self.lines[x].Flash()

    def unselected(self,dc):
        """線の選択解除処理（色の変更、移動用の円を削除）

        [引数]
        なし

        [戻り値]
        void
        """
        for x in range(len(self.lines)):
            self.lines[x].SetPen(wx.Pen(INACTIVE_COLOR, 1))
            self.lines[x].SetBrush(wx.Brush(wx.NamedColor(INACTIVE_COLOR)))
            self.lines[x].Flash()
        for x in range(len(self.oval_dict)):
            self.oval_dict[x+1].removeWidget(dc)

    def dmove(self, dc, movex, movey):
        """移動処理のダミールーチン

        [引数]
        dc    -- DCを指定
        movex -- 移動時の相対距離
        movey -- 移動時の相対距離

        [戻り値]
        void
        """
        pass

    def setLine2port(self, canvas, dc ):
        """線の生成
        線を引く２つのポートをあらかじめ指定し本メソッドを呼び出す。
        canvas.lineTo, canvas.lineFrom にポートを設定しておく。

        [引数]
        canvas -- 線を描画するキャンバスを指定
        dc     -- 線を描画するDCを指定

        [戻り値]
        void
        """
        # オブジェクトリファレンスの確認
        ref = canvas.lineTo.parent.ns_dict.GetObjRefToFullpath(canvas.lineTo.parent.fullpath)
        if not ref:
            return
        if canvas.lineFrom.parent.tag == 'in':
            self.g_inp = canvas.lineFrom.parent
            self.g_outp = canvas.lineTo.parent
        else:
            self.g_inp = canvas.lineTo.parent
            self.g_outp = canvas.lineFrom.parent
        # 線の生成とイベントの割付
        self.setPoints(self.g_inp.body.GetX(), self.g_inp.body.GetY(), self.g_outp.body.GetX(), self.g_outp.body.GetY())
        for x in range(len(self.lines)):
            evthandler2 = MyEvtHandlerLine()
            evthandler2.SetShape(self.lines[x])
            evthandler2.SetPreviousHandler(self.lines[x].GetEventHandler())
            self.lines[x].SetEventHandler(evthandler2)

        # キャンバスへラインの登録及びラインカウンターのアップ
        canvas.line[self.idx] = self
        self.g_inp.connect(self.idx)
        canvas.line_idx = canvas.line_idx + 1
        for x in range(len(self.lines)):
            self.lines[x].Show(True)
        # Inport/Outport の色の変更
        self.g_inp.body.Move(dc, self.g_inp.body.GetX(), self.g_inp.body.GetY())
        self.g_outp.body.Move(dc, self.g_outp.body.GetX(), self.g_outp.body.GetY())
        setBodyColor(self.g_inp.body, 'active')
        setBodyColor(self.g_outp.body, 'active')

    def changeCoordT(self, id, new_p1, new_p2):
        """線移動時に移動した線の座標を再設定

        [引数]
        id     -- 座標を再設定する開始インデックス（添え字）
        new_p1 -- 新しい座標（x,y）の開始点をタプルで指定
        new_p2 -- 新しい座標（x,y）の終了点をタプルで指定

        [戻り値]
        void
        """
        self.coordT[id] = new_p1
        self.coordT[id+1] = new_p2

        num = len(self.coordT)
        for cnt in range(num-1):
            self.lines[cnt].setPoints(self.coordT[cnt][0], self.coordT[cnt][1], self.coordT[cnt+1][0], self.coordT[cnt+1][1])

    def childMove(self, dc, pos_new):
        """線の移動

        [引数]
        dc      -- 描画するデバイス・コンテキストを指定
        pos_new -- 移動後の座標リスト

        [戻り値]
        void
        """
#        canvas = self.body.GetCanvas()

        max_num = len(pos_new)
        if max_num <= 2:
            return

        self.coordT = pos_new
        for cnt in range(max_num-1):
            self.lines[cnt].setPoints(self.coordT[cnt][0], self.coordT[cnt][1], self.coordT[cnt+1][0], self.coordT[cnt+1][1])

#----------------------------------------------------------------------
class GRtcIn(ogl.Shape):
    """インポート図形を作成するクラス"""
    def __init__(self, parent, ns_dict, fullpath, inp, pos_x, pos_y):
        """クラスの初期化（インポート図形の作成）

        [引数]
        parent     -- 親クラスを指定する
        ns_data -- コンポーネントのディクショナリー
        inp        -- インポートのディクショナリー(in_list[n])
        pos_x      -- インポート図形のｘ座標
        pos_y      -- インポート図形のｙ座標

        [戻り値]
        void
        """
        ogl.Shape.__init__(self)
        self.parent = parent
        self.ns_dict = ns_dict
        self.fullpath = fullpath
        self.inport = inp
        self.x = pos_x
        self.y = pos_y
        self.x_size = POLYGON_SIZE
        self.y_size = POLYGON_SIZE
        self.line = []
        self.line_idx = []         # Lxx line index
        self.position = 'Left'     # current position on compornent widget Left/Right/Top/Bottom
        self.textwin = 'non'
        self.tag = 'in'
        self.points = []
        self.createWidget()

    def getConfig(self, name) :
        """x,y座標もしくはpositionを取得する

        [引数]
        name -- 取得したい値のフラグを指定する
                フラグ：'x', 'y', 'position'

        [戻り値]
        void
        """
        if name == 'x' :
            return self.x      
        elif name == 'y' :
            return self.y  
        elif name == 'position' :
            return self.position  
        else :    
            return None

    def removeWidget(self, dc, rot=0):
        """インポート図形をキャンバス、DC上から削除する

        [引数]
        dc     -- 描画するデバイス・コンテキストを指定
        rot    -- 線の削除可否を指定する。（回転処理等で使用）
                  0:線を削除  /  1:線を削除しない

        [戻り値]
        void
        """
        canvas = self.body.GetCanvas()
        self.body.Erase(dc)
        self.body.RemoveFromCanvas(canvas)

        # line
        if rot == 0:
            line_list = copy.deepcopy(self.line_idx)
            for idx in line_list:
                canvas.line[idx].removeWidget(dc)
                if idx in canvas.line.keys():
                    del canvas.line[idx]
            self.line_idx = []

    def createWidget(self):
        """インポート図形を生成する

        [引数]
        なし

        [戻り値]
        void
        """
        self.color = INACTIVE_COLOR
        # 座標の設定
        self.dcoords()
        # 図形の生成
        self.body = makeInportPolygon(self, self.points)
        # イベントや図形の割付
        self.parent.parent.MyAddShape(
            self.body, self.x+POLYGON_SIZE/2-1, self.y+POLYGON_SIZE/2-1, wx.Pen(OUTLINE_COLOR, 1), wx.Brush(self.color, wx.SOLID), "" , 1)

    def dmove(self, dc, movex, movey):
        """インポート図形の移動
        インポートに割り付けられている線も同時に移動させる

        [引数]
        dc     -- 描画するデバイス・コンテキストを指定
        movex  -- ｘ座標の相対移動量を指定
        movey  -- ｙ座標の相対移動量を指定

        [戻り値]
        void
        """
        canvas = self.body.GetCanvas()
        canvas.PrepareDC(dc)

        self.body.Erase(dc)
        self.x = self.body.GetX() + movex
        self.y = self.body.GetY() + movey
        self.body.Move(dc, self.x, self.y)

        # line
        for line_index in self.line_idx:
            canvas.line[line_index].setStartPoint(dc, movex, movey)

    def delLineIdx(self,idx):
        """インポートに割り付けた線のインデックスを削除する

        [引数]
        idx   -- 線のインデックス

        [戻り値]
        void
        """
        if idx in self.line_idx:
            tmp = self.line_idx.index(idx)
            del self.line_idx[tmp]

    def selected(self):
        """インポートを選択状態にする

        [引数]
        なし

        [戻り値]
        void
        """
        self.body.SetBrush(wx.Brush(wx.NamedColor(SELECTED_COLOR)))
        self.body.Flash()

    def unselected(self,dc):
        """インポートを非選択状態にする

        [引数]
        なし

        [戻り値]
        void
        """
        self.body.SetBrush(wx.Brush(wx.NamedColor(self.color)))
        self.body.Flash()

    def updatePolygonSize(self, x, y, ratioW, ratioH):
        """インポート図形のサイズ変更

        [引数]
        x      -- サイズ変更後のｘ座標を指定
        y      -- サイズ変更後のｙ座標を指定
        ratioW -- サイズ変更を行うWidthのサイズ比率
        ratioH -- サイズ変更を行うHeightのサイズ比率

        [戻り値]
        void
        """
        movex = x - self.body.GetX()
        movey = y - self.body.GetY()
        self.x = x
        self.y = y
        # 図形回転後にwidth,heightを入れ替える
        if self.parent.xy_swap == 1:
            self.y_size, self.x_size = self.body.GetBoundingBoxMin()
            self.parent.py_size, self.parent.px_size = self.parent.px_size, self.parent.py_size
        else:
            self.x_size, self.y_size = self.body.GetBoundingBoxMin()

        # ポリゴンサイズの再計算
        self.parent.px_size = self.x_size
        self.parent.py_size = self.y_size
        canvas = self.body.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        brush = self.body.GetBrush()
        # 図形をキャンバス、DC上から削除する
        self.body.Erase(dc)
        self.body.RemoveFromCanvas(canvas)
        # 座標の再設定
        self.dcoords()
        self.body.updateInportPolygon(self.points)
        # 新しい図形にイベントを割付
        self.parent.parent.MyAddShape(
            self.body, self.x , self.y,
            wx.Pen(OUTLINE_COLOR, 1), brush, "" , 1)

        # 線を再設定（インポートの位置に合わせ移動）
        for line_index in self.line_idx:
            canvas.line[line_index].setStartPoint(dc, movex, movey)

    def connect(self, line_idx):
        """コネクト処理(線のインデックスを格納）

        [引数]
        line_idx   -- インポートに接続する線のインデックス

        [戻り値]
        成否フラグ -- 0:エラー(オブジェクトリファレンス無し） / 1:成功
        """
        ref = self.ns_dict.GetObjRefToFullpath(self.fullpath)
        if not ref:
            return 0
        else:
            self.line_idx.append(line_idx)
            return 1

    def disconnect(self, line_idx):
        """ディスコネクト処理(線のインデックスを削除）

        [引数]
        line_idx   -- インポートに接続している線のインデックス

        [戻り値]
        成否フラグ -- 0:エラー / 1:成功
        """
        if line_idx in self.line_idx:
            self.delLineIdx(line_idx)
            return 1
        else:
            return 0

    def dcoords(self):
        """インポート図形の座標設定
        現在のpositionを見て図形の向きを決定する
        各頂点は(x,y)のタプル形式

        [引数]
        なし

        [戻り値]
        void
        """
        if self.position == 'Left':
            self.points = [ (self.x + self.x_size/2-1, self.y + self.y_size/2),
                            (self.x,  self.y),
                            (self.x + self.x_size ,    self.y),
                            (self.x + self.x_size ,    self.y + self.y_size ),
                            (self.x ,    self.y + self.y_size ),
                            (self.x + self.x_size/2-1,    self.y + self.y_size/2)
                          ]
        elif self.position == 'Right' :
            self.points = [ (self.x+self.x_size/2+1, self.y+self.y_size/2),
                            (self.x+self.x_size, self.y+self.y_size),
                            (self.x, self.y+self.y_size),
                            (self.x, self.y),
                            (self.x+self.x_size, self.y),
                            (self.x+self.x_size/2+1, self.y+self.y_size/2)
                          ]
        elif self.position == 'Top' :
            self.points = [ (self.x+self.x_size/2, self.y+self.y_size/2-1),
                            (self.x+self.x_size, self.y),
                            (self.x+self.x_size, self.y+self.y_size),
                            (self.x, self.y+self.y_size),
                            (self.x, self.y),
                            (self.x+self.x_size/2, self.y+self.y_size/2-1)
                          ]
        elif self.position == 'Bottom' :    
            self.points = [ (self.x+self.x_size/2, self.y+self.y_size/2+1),
                            (self.x, self.y+self.y_size),
                            (self.x, self.y),
                            (self.x+self.x_size, self.y),
                            (self.x+self.x_size, self.y+self.y_size),
                            (self.x+self.x_size/2, self.y+self.y_size/2+1)
                          ]

#----------------------------------------------------------------------
class GRtcOut(ogl.Shape):
    """アウトポート図形を作成するクラス"""
    def __init__(self, parent, ns_dict, fullpath, outp, pos_x, pos_y) :
        """クラスの初期化（アウトポート図形の作成）

        [引数]
        parent     -- 親クラスを指定する
        ns_data -- コンポーネントのディクショナリー
        inp        -- インポートのディクショナリー(in_list[n])
        pos_x      -- インポート図形のｘ座標
        pos_y      -- インポート図形のｙ座標

        [戻り値]
        void
        """
        ogl.Shape.__init__(self)
        self.parent = parent
        self.ns_dict = ns_dict
        self.fullpath = fullpath
        self.outport = outp
        self.x = pos_x
        self.y = pos_y
        self.x_size = POLYGON_SIZE
        self.y_size = POLYGON_SIZE
        self.line = []
        self.line_idx = []         # Lxx line index
        self.position = 'Right'     # current position on compornent widget Left/Right/Top/Bottom
        self.textwin = 'non'
        self.isInactive = 0
        self.tag = 'out'
        self.uuid = {}
        self.subscription_type = RTM.OPS_NEW
        self.createWidget()

    def refresh(self):
        """リフレッシュ処理
        現在の接続状況（Inportのオブジェクトリファレンスが存在するか？）を
        チェックし、接続状態を継続もしくはunsubscribeを実行する

        [引数]
        なし

        [戻り値]
        void
        """
        canvas = self.body.GetCanvas()
        if canvas.viewMode == True:
            return

        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        for idx in self.line_idx:
            # for line 
            try:
                # g_inp update
                # state get ??
                for inp in self.parent.in_list :
                    if inp['name'] == canvas.line[idx].g_inp.inport['name']:
                        canvas.line[idx].g_inp.inport = inp
                        break

                ref = canvas.line[idx].g_inp.inport['ref']
                ref = ref._narrow(RTM.InPort)
            except :
                except_mess('inport object-ref failure:%s\n'%inp['name'])
                setBodyColor(canvas.line[idx].g_inp.body, 'inactive')
                try :  
                    ref = self.outport['ref']
                    ref = ref._narrow(RTM.OutPort)
                    rslt = ref.unsubscribe(self.uuid[idx])
                    print "refresh:unsubscribe:",rslt
                    if rslt != 0:
                        print 'unsubscribe failure: rslt=',rslt
                except :
                    except_mess('unsubscribe failure:')

    def getConfig(self, name) :
        """x,y座標もしくはpositionを取得する

        [引数]
        name -- 取得したい値のフラグを指定する
                フラグ：'x', 'y', 'position'

        [戻り値]
        void
        """
        if name == 'x' :
            return self.x      
        elif name == 'y' :
            return self.y  
        elif name == 'position' :
            return self.position  
        else :    
            return None

    def removeWidget(self, dc, rot=0):
        """アウトポート図形を削除する

        [引数]
        dc     -- 描画するデバイス・コンテキストを指定
        rot    -- 線の削除可否を指定する。（回転処理等で使用）
                  0:線を削除  /  1:線を削除しない

        [戻り値]
        void
        """
        canvas = self.body.GetCanvas()
        self.body.Erase(dc)
        self.body.RemoveFromCanvas(canvas)

        # line
        if rot == 0:
            line_list = copy.deepcopy(self.line_idx)
            for idx in line_list:
                canvas.line[idx].removeWidget(dc)
                if idx in canvas.line.keys():
                    del canvas.line[idx]
            self.line_idx = []

    def createWidget(self):
        """アウトポート図形を作成する

        [引数]
        なし

        [戻り値]
        void
        """
        self.color = INACTIVE_COLOR
        # 座標の設定
        self.dcoords()
        # 図形の生成
        self.body = makeOutportPolygon(self, self.points)
        # イベントや図形の割付
        self.parent.parent.MyAddShape(
            self.body, self.x+POLYGON_SIZE/2-1, self.y+POLYGON_SIZE/2-1, wx.Pen(OUTLINE_COLOR, 1), wx.Brush(self.color, wx.SOLID), "",1)

    def dmove(self, dc, movex, movey):
        """アウトポート図形の移動
        アウトポートに割り付けられている線も同時に移動させる

        [引数]
        dc     -- 描画するデバイス・コンテキストを指定
        movex  -- ｘ座標の相対移動量を指定
        movey  -- ｙ座標の相対移動量を指定

        [戻り値]
        void
        """
        canvas = self.body.GetCanvas()
        canvas.PrepareDC(dc)

        self.x = self.body.GetX() + movex
        self.y = self.body.GetY() + movey
        self.body.Erase(dc)
        self.body.Move(dc, self.x, self.y)

        # line
        num = 0
        for line_index in self.line_idx:
            canvas.line[line_index].setEndPoint(dc, self.body,movex, movey)
            num = num+1

    def delLineIdx(self, idx):
        """アウトポートに割り付けた線のインデックスを削除する

        [引数]
        idx    -- 線のインデックス

        [戻り値]
        void
        """
        if idx in self.line_idx:
            tmp = self.line_idx.index(idx)
            del self.line_idx[tmp]

    def selected(self):
        """アウトポートを選択状態にする

        [引数]
        なし

        [戻り値]
        void
        """
        self.body.SetBrush(wx.Brush(wx.NamedColor(SELECTED_COLOR)))
        self.body.Flash()

    def unselected(self,dc):
        """アウトポートを非選択状態にする

        [引数]
        なし

        [戻り値]
        void
        """
        self.body.SetBrush(wx.Brush(wx.NamedColor(self.color)))
        self.body.Flash()

    def updatePolygonSize(self, x, y, ratioW, ratioH):
        """アウトポート図形のサイズ変更

        [引数]
        x      -- サイズ変更後のｘ座標を指定
        y      -- サイズ変更後のｙ座標を指定
        ratioW -- 変更を行うWidthのサイズ比率
        ratioH -- 変更を行うHeightのサイズ比率

        [戻り値]
        void
        """
        movex =  x - self.body.GetX()
        movey =  y - self.body.GetY()
        self.x = x
        self.y = y
        # 図形回転後にwidth,heightを入れ替える
        if self.parent.xy_swap == 1:
            self.y_size, self.x_size = self.body.GetBoundingBoxMin()
            self.parent.py_size, self.parent.px_size = self.parent.px_size, self.parent.py_size
        else:
            self.x_size, self.y_size = self.body.GetBoundingBoxMin()
        # ポリゴンサイズの再計算
        self.parent.px_size = self.x_size
        self.parent.py_size = self.y_size
        canvas = self.body.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        brush = self.body.GetBrush()
        # 図形をキャンバス、DC上から削除する
        self.body.Erase(dc)
        self.body.RemoveFromCanvas(canvas)
        # 座標の再設定
        self.dcoords()
        self.body.updateOutportPolygon(self.points)
        # 新しい図形にイベントを割付
        self.parent.parent.MyAddShape(
            self.body, self.x , self.y,
            wx.Pen(OUTLINE_COLOR, 1), brush, "" , 1)

        # 線を再設定（アウトポートの位置に合わせ移動）
        for line_index in self.line_idx:
            canvas.line[line_index].setEndPoint(dc, self.body,movex, movey)

    def connect2(self, line_idx, subscription_type):
        """コネクト処理(subscribeを発行）

        [引数]
        line_idx   -- インポートに接続する線のインデックス
        subscription_type -- サブスクリプション・タイプを指定（現在未使用）

        [戻り値]
        成否フラグ -- 0:エラー(オブジェクトリファレンス無し,subscribe失敗） / 1:成功
        """
        canvas = self.body.GetCanvas()

        # get outport-object-ref
        ref = self.outport['ref']
        if ref == None :
            return 0  
        try:
            ref = ref._narrow(RTM.OutPort)
        except:
            except_mess('outport obj-ref failure:')
            return 0

        # get inport-object-ref
        inp_ref = canvas.line[line_idx].g_inp.inport['ref']
        try:
            inp_ref = inp_ref._narrow(RTM.InPort)
        except:
            except_mess('inport obj-ref failure:')
            return 0

        # get subscription-list
        subscription_list = []
        try:
            subscription_list = ref._get_subscriptions()
            if subscription_list == None:
                print "get subscriptions failure: return value is None."
                return 0
        except:
            except_mess('get subscriptions failure:')
            return 0

        connect_num = self.checkConnect(inp_ref, subscription_list)

        if canvas.viewMode == False:
            if connect_num == -1:
                try:
                    canvas.line[line_idx].subscription_type = subscription_type
                    canvas.line[line_idx].profile = RTM.SubscriptionProfile(subscription_type,"",None,None,False,[])
                    canvas.line[line_idx].profile.out_port = ref
                    canvas.line[line_idx].profile.in_port = inp_ref
                    rslt, canvas.line[line_idx].profile = ref.subscribe(canvas.line[line_idx].profile)
                    self.uuid[line_idx] = canvas.line[line_idx].profile.id

                    if rslt != 0:
                        print "subscribe failure!"
                except:
                    except_mess('subscribe failure:')
                print "connect2 subscribe :",self.uuid[line_idx]
            else:
                rslt = 0
                # get uuid
                self.uuid[line_idx] = subscription_list[connect_num].id
        return 1

    def connect(self, line_idx, subscription_type):
        """コネクト処理(線のインデックスを格納、subscribeを発行）

        [引数]
        line_idx   -- インポートに接続する線のインデックス
        subscription_type -- サブスクリプション・タイプを指定（現在未使用）

        [戻り値]
        成否フラグ -- 0:エラー(オブジェクトリファレンス無し,subscribe失敗） / 1:成功
        """
        canvas = self.body.GetCanvas()
        n = 0
        for n in range(2):    # for retry
            try:
                ref = self.outport['ref']
                if ref == None :
                    return 0  
                ref = ref._narrow(RTM.OutPort)
                break
            except:
                except_mess('outport obj-ref failure:')
                self.parent.refresh()
        if n == 2:
            print "error retry"
            return 0


        inp_ref = canvas.line[line_idx].g_inp.inport['ref']
        try:
            inp_ref = inp_ref._narrow(RTM.InPort)
        except:
            except_mess('inport obj-ref failure:')
            return 0

        # get subscription-list
        subscription_list = []
        try:
            subscription_list = ref._get_subscriptions()
            if subscription_list == None:
                print "get subscriptions failure: return value is None."
                return 0
        except:
            except_mess('get subscriptions failure:')
            return 0

        canvas.line[line_idx].subscription_type = subscription_type
        canvas.line[line_idx].profile = RTM.SubscriptionProfile(subscription_type,"",None,None,False,[])


        connect_num = self.checkConnect(inp_ref, subscription_list)
#assembly dummy
#        connect_num = -1
#assembly dummy

        rslt = 0
        if canvas.viewMode == False:
            if connect_num == -1:
                try:
                    canvas.line[line_idx].profile.out_port = ref
                    canvas.line[line_idx].profile.in_port = inp_ref
                    (rslt, canvas.line[line_idx].profile) = ref.subscribe(canvas.line[line_idx].profile)
                    if rslt != 0:
                        print "subscribe failuer! :rslt=",rslt
                    self.uuid[line_idx] = canvas.line[line_idx].profile.id
                    print "connect subscribe :",self.uuid[line_idx]
                except:
                    err_mess =  'subscribe failure! :'
                    except_mess(err_mess)
            else:
                rslt = 0
                # get uuid
                self.uuid[line_idx] = subscription_list[connect_num].id

        if rslt :
            print "subsrcibe-rslt:",rslt
            return 0
        else :
            self.line_idx.append(line_idx)
            self.isInactive = self.isInactive + 1
            return 1

    def disconnect(self, line_idx):
        """ディスコネクト処理(線のインデックスを削除、unsubscribeを発行）

        [引数]
        line_idx   -- アウトポートに接続している線のインデックス

        [戻り値]
        成否フラグ -- 0:エラー / 1:成功
        """
        if self.isInactive == 1 :
            setBodyColor(self.body, 'inactive')

        canvas = self.body.GetCanvas()

        n = 0
        for n in range(2):    # for retry
            ref = self.outport['ref']
            try:
                ref = ref._narrow(RTM.OutPort)

                inp_obj = canvas.line[line_idx].g_inp.inport['ref']

                # get subscription-list
                subscription_list = []
                subscription_list = ref._get_subscriptions()
                if subscription_list == None:
                    print "get subscriptions failure: return value is None."
                    return 0

                connect_num = self.checkConnect(inp_obj, subscription_list)
#assembly dummy
#                connect_num = 0
#assembly dummy
                break
            except:
                err_mess =  'outport disconnect failure:'
                except_mess(err_mess)
                connect_num = -1
                self.parent.refresh()

        if n == 2: # bad connect
            return 0

        if ref != None and canvas.viewMode == False and connect_num != -1:
            try :
                print "unsubscribe :",self.uuid[line_idx]
                rslt = ref.unsubscribe(self.uuid[line_idx])
                if rslt != 0:
                    print 'unsubscribe failure: rslt=',rslt
            except :
                err_mess = 'unsubscribe failure:'
                except_mess(err_mess)

        self.isInactive = self.isInactive - 1

        if line_idx in self.line_idx:
            self.delLineIdx(line_idx)
            return 1
        else :
            return 0

    def remakeLines(self):
        """再接続処理
        オブジェクト上だけに存在する接続情報(subscribe)を検索し線を引く

        [引数]
        なし

        [戻り値]
        void
        """
# assembly dummy process
#        return
# assembly dummy process

        canvas = self.body.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)

        ref = self.outport['ref']
        try:
            ref = ref._narrow(RTM.OutPort)
        except:
            err_mess =  'outport obj-ref failure:'
            except_mess(err_mess)
            return

        # get subscription-list
        subscription_list = []
        subscr_list_tmp = []
        try:
            subscription_list = ref._get_subscriptions()
            subscr_list_tmp = copy.deepcopy(subscription_list)
            if subscription_list == None:
                print "get subscriptions failure: return value is None."
                return
        except:
            except_mess('get subscriptions failure:')
            return

        for line_idx in self.line_idx:
            line = canvas.line[line_idx]
            (ret2,subscr_list_tmp) = self.checkConnect2(line,subscr_list_tmp)

        rtc_list = self.parent.parent.rtc_list
        rtc_dict = self.parent.parent.rtc_dict
        ret_name = []
        ret_obj = []
        ret_ref = []

        for subscr in subscr_list_tmp:
            inp_ref = subscr.in_port
            for fullname in rtc_list:
                in_list = rtc_dict[fullname].in_list
                in_dict = rtc_dict[fullname].in_dict

                for inp in in_list:
                    if inp['name'] in in_dict.keys():
                        ref = in_dict[inp['name']].inport['ref']
                        if inp_ref._is_equivalent(ref):
                            print "_is_equivalent is OK!!!"
                            ret_name.append( inp['name'] )
                            ret_obj.append( in_dict[inp['name']] )
                            ret_ref.append(inp_ref)

        for num in range(len(ret_name)):
            canvas.lineFrom = self.body
            canvas.lineTo = ret_obj[num].body
            line = GRtcLine(canvas,self)
            line.setLine2port(canvas, dc)

            self.line_idx.append(line.idx)
            self.isInactive = self.isInactive + 1
            connect_num = self.checkConnect(ret_ref[num], subscription_list)
            # get uuid
            self.uuid[line.idx] = subscription_list[connect_num].id

        canvas.lineFrom = None
        canvas.lineTo = None
        canvas.Redraw(dc)


    def checkOtherConnect(self):
        """古い接続情報があるかチェックする
        画面上の線以外の接続がオブジェクト上にあるかチェックする

        [引数]
        なし

        [戻り値]
        ret  ---  True:ある / False:ない
        """
        ret = False
        canvas = self.body.GetCanvas()
        ref = self.outport['ref']
        try:
            ref = ref._narrow(RTM.OutPort)
        except:
            err_mess = 'outport obj-ref failure:'
            except_mess(err_mess)
            return ret

        # get subscription-list
        subscription_list = []
        try:
            subscription_list = ref._get_subscriptions()
            if subscription_list == None:
                print "get subscriptions failure: return value is None."
                return ret
        except:
            except_mess('get subscriptions failure:')
            return ret

        for line_idx in self.line_idx:
            line = canvas.line[line_idx]
            (ret2,subscription_list) = self.checkConnect2(line,subscription_list)
        if len(subscription_list) > 0:
            ret = True
        return ret

    def checkConnect(self, inp_obj, subscr_list):
        """接続チェック
        指定した接続先（inport）のリファレンスがあるかチェックする

        [引数]
        inp_obj  ---  インポートのオブジェクト・リファレンス
        ref_list ---  インポートのリファレンス・リスト

        [戻り値]
        ret_num --- subScription_list の添え字/ない場合は-1
        """
        ret = False
        ret_num = 0
        for subscr in subscr_list:
            ref_inp = subscr.in_port
            if ref_inp._is_equivalent(inp_obj):
                print "checkConnect: _is_equivalent is OK!!!"
                ret = True
                break
            ret_num = ret_num + 1
        if ret == False:
            ret_num = -1
        return ret_num

    def checkConnect2(self, line, subscr_list):
        """接続チェック
        チェック対象の接続があった場合は、リスト上から削除して返却する
        古い情報があるか調べる為に呼ばれる

        [引数]
        line      ---  線のオブジェクト
        ref_list  ---  接続先（inport）のオブジェクトリファレンス・リスト

        [戻り値]
        (ret, ref_list) --- ret 0:ない / 1:ある , ref_list: 残りのリファレンスリスト
        """

        inp_obj = line.g_inp.inport['ref']

        cnt = 0
        ret = 0
        for subscr in subscr_list:
            ref_inp = subscr.in_port
            if ref_inp._is_equivalent(inp_obj):
#                print "checkConnect2: _is_equivalent is OK!!!"
                ret = 1
                break
            cnt = cnt + 1
        if ret == 1:
            del subscr_list[cnt]

        return (ret, subscr_list)

    def disconnectToObjref(self,subscr_list):
        """コンポーネント上の接続情報（subscribe）を削除する

        [引数]
        inp_list  ---  接続先（subscriptionProfile）のリスト

        [戻り値]
        void
        """
#assembly: for debug:
#        print "disconnectToObjref : it's dummy for debug!"
#        return      # for debug
#for debug:

        canvas = self.body.GetCanvas()
        ref = self.outport['ref']
        ref = ref._narrow(RTM.OutPort)

        for subscr in subscr_list:
            inp = subscr.in_port
#            print "test:",dir(inp)
            #get uuid
            connect_num = self.checkConnect(inp, subscr_list)
            uuid = subscr_list[connect_num].id

            if ref != None and canvas.viewMode == False:
                try :
                    print "unsubscribe2 :",uuid
                    rslt = ref.unsubscribe(uuid)
                    if rslt != 0:
                        print 'unsubscribe2 failure: rslt=',rslt
                except :
                    err_mess = 'unsubscribe failure:'
                    except_mess(err_mess)


    def reConnectLine(self):
        """再接続処理

        [引数]
        なし

        [戻り値]
        void
        """
#assembly dummy
#        return
#assembly dummy

        canvas = self.body.GetCanvas()
        ref = self.outport['ref']
        try:
            ref = ref._narrow(RTM.OutPort)
        except:
            err_mess = 'outport obj-ref failure:'
            except_mess(err_mess)
            return

        # get subscription-list
        subscription_list = []
        subscr_list_tmp = []
        try:
            subscription_list = ref._get_subscriptions()
            subscr_list_tmp = copy.deepcopy(subscription_list)
            if subscription_list == None:
                print "get subscriptions failure: return value is None."
                return
        except:
            except_mess('get subscriptions failure:')
            return

        for line_idx in self.line_idx:
            line = canvas.line[line_idx]
            (ret,subscr_list_tmp) = self.checkConnect2(line,subscr_list_tmp)
            if ret == 0:
                self.connect2(line_idx,line.subscription_type)
            else:
                # get uuid
                inp_ref = canvas.line[line_idx].g_inp.inport['ref']
                connect_num = self.checkConnect(inp_ref, subscription_list)
                self.uuid[line_idx] = subscription_list[connect_num].id

		# 再接続処理から漏れたsubscribeの検出：大抵はnaming-service上のゴミ？
        for line_idx in self.line_idx:
            line = canvas.line[line_idx]
            (ret,subscr_list_tmp) = self.checkConnect2(line,subscr_list_tmp)
            if ret == 0:
                self.connect2(line_idx,line.subscription_type)
        if len(subscr_list_tmp) > 0:
#            print "reconnect "
            self.disconnectToObjref(subscr_list_tmp)

    def dcoords(self):
        """アウトポート図形の座標設定
        現在のpositionを見て図形の向きを決定する
        各頂点は(x,y)のタプル形式

        [引数]
        なし

        [戻り値]
        void
        """
        if self.position == 'Left':
            self.points = [ (self.x, self.y+self.y_size/2),
                            ( self.x+self.x_size/2, self.y),
                            ( self.x+self.x_size, self.y),
                            ( self.x+self.x_size, self.y+self.y_size),
                            ( self.x+self.x_size/2, self.y+self.y_size),
                            ( self.x, self.y+self.y_size/2) ]
        elif self.position == 'Right' :
            self.points = [ ( self.x+self.x_size, self.y+self.y_size/2 ),
                            ( self.x+self.x_size/2, self.y+self.y_size ),
                            ( self.x, self.y+self.y_size ),
                            ( self.x, self.y ),
                            ( self.x+self.x_size/2, self.y ),
                            ( self.x+self.x_size, self.y+self.y_size/2) ]
        elif self.position == 'Top' :
            self.points = [ ( self.x+self.x_size/2, self.y ),
                            ( self.x+self.x_size, self.y+self.y_size/2 ),
                            ( self.x+self.x_size, self.y+self.y_size ),
                            ( self.x, self.y+self.y_size ),
                            ( self.x, self.y+self.y_size/2 ),
                            ( self.x+self.x_size/2, self.y ) ]
        elif self.position == 'Bottom' :
            self.points = [ ( self.x+self.x_size/2, self.y+self.y_size ),
                            ( self.x, self.y+self.y_size/2 ),
                            ( self.x, self.y ),
                            ( self.x+self.x_size, self.y ),
                            ( self.x+self.x_size, self.y+self.y_size/2 ),
                            ( self.x+self.x_size/2, self.y+self.y_size ) ]

#----------------------------------------------------------------------
class GRtc(ogl.Shape):
    """コンポーネント図形の本体を作成するクラス"""
    def __init__(self, parent, fullpath, pos_x, pos_y):
        """クラスの初期化（コンポーネント図形の作成）

        [引数]
        parent     -- 親クラスを指定する
        fullpath   -- コンポーネントのロングネーム
        pos_x      -- コンポーネント図形のｘ座標
        pos_y      -- コンポーネント図形のｙ座標

        [戻り値]
        void
        """
        ogl.Shape.__init__(self)
        self.parent = parent
        self.fullpath = fullpath
        self.ns_dict = self.parent.frame.myDict
#        print "check cur_dict:",cur_dict[0]
        self.name = self.ns_dict.GetCompName(fullpath)
        self.in_list = self.ns_dict.GetInPortToRef(fullpath)
        self.out_list = self.ns_dict.GetOutPortToRef(fullpath)

        self.x = pos_x
        self.y = pos_y
        self.color = INACTIVE_COLOR
        self.state = 'inactive'
        self.x_size = BOX_WIDTH
        self.y_size = BOX_WIDTH
        self.ratioW = 1.0
        self.ratioH = 1.0
        self.rotTogle = 0
        self.revTogle = 1
        self.lastRot = 'LR'   # LR(Left/Right) or TB(Top/Bottom)
        self.xy_swap = 0
        self.mark = None
        self.tag = 'body'
        self.text = None
        self.lastBBoxWidth = 0
        self.lastBBoxHeight = 0
        self.text_x = 0
        self.text_y = 0
        self.px_size = POLYGON_SIZE
        self.py_size = POLYGON_SIZE
        tmp = max(len(self.in_list), len(self.out_list))
        self.minWidth = self.x_size
        self.minHeight = 2 * POLYGON_SIZE * tmp
#        self.blink = blinkTimer()
        self.createWidget(0)

    def remakeLines(self):
# assembly dummy process
#        return
# assembly dummy process
        for outp in self.out_list :
            if outp['name'] in self.out_dict.keys():
                self.out_dict[outp['name']].remakeLines()

    def checkOtherConnect(self):
        """古い接続情報（画面上に表示されていないsubscribe情報）をチェックする

        [引数]
        なし

        [戻り値]
        ret   ---  True:古い情報あり / False:古い情報なし
        """
        ret = False
        for outp in self.out_list :
            if outp['name'] in self.out_dict.keys():
                ret = self.out_dict[outp['name']].checkOtherConnect()
                if ret == True:
                    break
        return ret

    def reConnectLine(self):
        """再接続処理
        コンポーネントのアウトポートの再接続処理を呼び出す

        [引数]
        なし

        [戻り値]
        void
        """
        for outp in self.out_list :
            if outp['name'] in self.out_dict.keys():
                self.out_dict[outp['name']].reConnectLine()

    def portToFlash(self):
        """ポート(Shape)のFlash（再描画？）を呼び出す
        コンポーネントの絵の下にポートの絵がもぐり込むケースの時に呼び出している。（回避策）
        他に良い処理があれば、随時そちらに変更する

        [引数]
        なし

        [戻り値]
        void
        """
        for inp in self.in_list :
            if inp['name'] in self.in_dict.keys():
                self.in_dict[inp['name']].body.Flash()
        for outp in self.out_list :
            if outp['name'] in self.out_dict.keys():
                self.out_dict[outp['name']].body.Flash()

    def checkCompState(self):
        """コンポーネントのステータスをチェックする

        [引数]
        なし

        [戻り値]
        void
        """
        state = 'inactive'

        canvas = self.body.GetCanvas()
        tmp = self.ns_dict.GetCompState(self.fullpath)
        if tmp == RTM.RTComponent.RTC_STARTING or tmp == RTM.RTComponent.RTC_ACTIVE:
            state = 'active'
        elif tmp == RTM.RTComponent.RTC_READY or tmp == RTM.RTComponent.RTC_STOPPING:
            state = 'inactive'
        elif tmp >= RTM.RTComponent.RTC_ABORTING :
            state = 'error'
        else:   # unknown , born?, initializing
            state = 'unloaded'
        if canvas.viewMode == True and state != 'unloaded':
            state = 'virtual'

        self.state = state

    def ref_start(self):
        """コンポーネントにstart命令を発行

        [引数]
        なし

        [戻り値]
        void
        """
        try:
            ref = self.ns_dict.GetObjRefToFullpath(self.fullpath)
            ref = ref._narrow(RTM.RTCBase)
            ref.rtc_start()
        except :
            err_mess = 'rtc_start error:%s\n'%self.fullpath
            except_mess(err_mess)
        self.checkCompState()
#        setBodyColor(self.baseBox, self.state)
#        setBodyColor(self.baseBox, 'active')
        self.ns_dict.setCompBodyColor(self.fullpath, 'active')
        self.state = 'active'
        self.portToFlash()

    def ref_stop(self):
        """コンポーネントにstop命令を発行

        [引数]
        なし

        [戻り値]
        void
        """
        try:
            ref = self.ns_dict.GetObjRefToFullpath(self.fullpath)
            ref = ref._narrow(RTM.RTCBase)
            ref.rtc_stop()
        except :
            err_mess = 'rtc_stop error:%s\n'%self.fullpath
            except_mess(err_mess)
        self.checkCompState()
#        setBodyColor(self.baseBox, self.state)
#        setBodyColor(self.baseBox, 'inactive')
        self.ns_dict.setCompBodyColor(self.fullpath, 'inactive')
        self.state = 'inactive'
        self.portToFlash()

    def ref_reset(self):
        """コンポーネントにreset命令を発行

        [引数]
        なし

        [戻り値]
        void
        """
        try:
            ref = self.ns_dict.GetObjRefToFullpath(self.fullpath)
            ref = ref._narrow(RTM.RTCBase)
            ref.rtc_reset()
        except :
            err_mess = 'rtc_reset error:%s\n'%self.fullpath
            except_mess(err_mess)
#        self.checkCompState()
#        setBodyColor(self.baseBox, self.state)
        self.ns_dict.setCompBodyColor(self.fullpath, self.state)
        self.portToFlash()

    def ref_kill(self):
        """コンポーネントにkill命令を発行

        [引数]
        なし

        [戻り値]
        void
        """
        try:
            ref = self.ns_dict.GetObjRefToFullpath(self.fullpath)
            ref = ref._narrow(RTM.RTCBase)
            ref.rtc_kill()
        except :
            err_mess = 'rtc_kill error:%s\n'%self.fullpath
            except_mess(err_mess)
        self.checkCompState()
#        setBodyColor(self.baseBox, self.state)
#        setBodyColor(self.baseBox, 'inactive')
        self.ns_dict.setCompBodyColor(self.fullpath, 'unloaded')
        self.state = 'inactive'
        self.portToFlash()

    def ref_exit(self):
        """コンポーネントにexit命令を発行

        [引数]
        なし

        [戻り値]
        void
        """
        try:
            ref = self.ns_dict.GetObjRefToFullpath(self.fullpath)
            ref = ref._narrow(RTM.RTCBase)
            ref.rtc_exit()
        except :
            err_mess = 'rtc_exit error:%s\n'%self.fullpath
            except_mess(err_mess)
        self.checkCompState()
#        setBodyColor(self.baseBox, self.state)
#        setBodyColor(self.baseBox, 'unloaded')
        self.ns_dict.setCompBodyColor(self.fullpath, 'unloaded')
        self.state = 'unloaded'
        self.portToFlash()

    def changeBodyColor(self,state):
        """ステータスによりコンポーネントの色を設定する

        [引数]
        state  ---  コンポーネントの状態を指定する
                    'active','inactive','error',unloaded','virtual'

        [戻り値]
        void
        """
        if state == 'unloaded':
            self.state = 'unloaded'
            self.color = UNLOADED_COLOR
        elif state == 'active' :
            self.state = 'active'
            self.color = ACTIVE_COLOR
        elif state == 'inactive':
            self.state = 'inactive'
            self.color = INACTIVE_COLOR
        elif state == 'error' :
            self.state = 'error'
            self.color = ERROR_COLOR
#        canvas = self.body.GetCanvas()
        canvas = self.parent.diagram.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        if canvas.viewMode == True and self.state != 'unloaded':
            self.state = 'virtual'
            self.color = VIRTUAL_COLOR
        setBodyColor(self.baseBox, self.state)
        self.portToFlash()
        canvas.Redraw(dc)

    def refresh_outp(self):
        """アウトポートのrefresh

        [引数]
        なし

        [戻り値]
        void
        """
        for outp in self.out_list :
            if outp['name'] in self.out_dict.keys():
                self.out_dict[outp['name']].refresh()

    def refresh(self):
        """リフレッシュ処理
        コンポーネントのstateフラグで現在の状態（active,error,inactive等）を設定

        [引数]
        なし

        [戻り値]
        void
        """
        old_state = self.state
        canvas = self.body.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        try :  
            ref = self.ns_dict.GetObjRefToFullpath(self.fullpath)
            ref = ref._narrow(RTM.RTCBase)
            tmp_port = ref._get_rtc_state()
            tmp_port = tmp_port._narrow(RTM.OutPort)
            tmp = tmp_port.get()
            tmp = tmp.value()
            tmp = tmp.data
            print "refresh state:",tmp
        except :
            except_mess("except error:")
            ref = None

        if not ref:
            self.state = 'unloaded'
            self.color = UNLOADED_COLOR
        else:
            self.name = self.ns_dict.GetCompName(self.fullpath)
            self.in_list = self.ns_dict.GetInPortToRef(self.fullpath)
            self.out_list = self.ns_dict.GetOutPortToRef(self.fullpath)

            for outp in self.out_list :
                if outp['name'] in self.out_dict.keys():
                    self.out_dict[outp['name']].outport = outp
            for inp in self.in_list :
                if inp['name'] in self.in_dict.keys():
                    self.in_dict[inp['name']].inport = inp


            if tmp == RTM.RTComponent.RTC_STARTING or tmp == RTM.RTComponent.RTC_ACTIVE:
                self.state = 'active'
                self.color = ACTIVE_COLOR
            elif tmp == RTM.RTComponent.RTC_STOPPING or tmp == RTM.RTComponent.RTC_READY:
                self.state = 'inactive'
                self.color = INACTIVE_COLOR
            elif tmp >= RTM.RTComponent.RTC_ABORTING :
                self.state = 'error'
                self.color = ERROR_COLOR
            else :
                self.state = 'unloaded'
                self.color = UNLOADED_COLOR

#            if old_state == 'unloaded' and self.state != 'unloaded':
            if len(self.out_dict.keys()) != len(self.out_list):
                self.removeWidget(dc,0)

                old_rot = self.rotTogle
                old_rev = self.revTogle
                old_lastrot = self.lastRot
                self.rotTogle = 0
                self.revTogle = 1
                self.lastRot = 'LR'   # LR(Left/Right) or TB(Top/Bottom)
                self.x_size = BOX_WIDTH
                self.x = self.x - self.x_size/2
                self.y = self.y - self.y_size/2
                self.ratioW = 1.0
                self.ratioH = 1.0
                self.createWidget(0)
                if old_lastrot == 'LR':
                    if old_rev == 0:
                        self.reversesBody()
                else:
                    if old_rot == 1:
                        self.rotatesBody()
                    else:
                        self.rotatesBody()
                        self.rotatesBody()

#        setBodyColor(self.baseBox, self.state)
        self.ns_dict.setCompBodyColor(self.fullpath, self.state)
        self.portToFlash()
        canvas.Redraw(dc)

    def removeWidget(self, dc, rot=0):
        """コンポーネント図形を削除する
        インポート図形、アウトポート図形、関連する線も削除する

        [引数]
        dc     -- 描画するデバイス・コンテキストを指定
        rot    -- 線の削除可否を指定する。（回転処理等で使用）
                  0:線を削除  /  1:線を削除しない

        [戻り値]
        void
        """
        # 本体図形をキャンバス、DC上から削除する
        canvas = self.body.GetCanvas()
        self.body.Erase(dc)
        self.body.RemoveFromCanvas(canvas)

        # ビットマップの削除
        if self.bmp:
            self.bmp.Erase(dc)
            self.bmp.RemoveFromCanvas(canvas)

        # コンポーネント名称の削除
        self.text.Erase(dc)
        self.text.RemoveFromCanvas(canvas)

        # インポート／アウトポートの削除
        for inp in self.in_list:
            if inp['name'] in self.in_dict.keys():
                self.in_dict[inp['name']].removeWidget(dc,rot)
        for outp in self.out_list:
            if outp['name'] in self.out_dict.keys():
                self.out_dict[outp['name']].removeWidget(dc,rot)


    def createWidget(self, rot):
        """コンポーネント図形の作成

        [引数]
        rot     -- 図形の回転処理を行うフラグ
                   0:回転なし（座標を計算で求める） 1:回転あり(既存の座標を使用）

        [戻り値]
        void
        """
        if rot == 0:
            tmp = max(len(self.in_list), len(self.out_list))
            if tmp == 0:
                tmp = 1
            self.y_size = 2 * POLYGON_SIZE * self.ratioH * tmp
            pos_x = self.x + (BOX_WIDTH * self.ratioW)/2
            pos_y = self.y + self.y_size/2
        else:
            pos_x = self.x
            pos_y = self.y

        # コンポーネント名称(TextShape)の作成
        canvas = self.parent.diagram.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        cnt = len(self.name)
        charW = dc.GetCharWidth()
        charH = dc.GetCharHeight()
        tmpW = charW * (cnt*1.2)
        tmpH = charH * 1.4
        self.text_x = pos_x + (self.x_size/2)
        self.text_y = self.y + self.y_size + POLYGON_SIZE

        self.text = makeTextShape(self,tmpW, tmpH)
        self.text.AddText(self.name)
        self.parent.MyAddText(self.text, self.text_x, self.text_y,wx.BLACK_PEN)
###
        # ビットマップの作成
        self.bmp = ogl.BitmapShape()
#        if self.rtc.icon_path != None or self.rtc.icon_path != "":
#        if self.rtc.icon_path != "":
#            bitmap = wx.Bitmap(self.rtc.icon_path)
#        else:
#            bitmap = wx.NullBitmap
#            self.bmp.SetSize(10,10,False)
        bitmap = wx.NullBitmap
        self.bmp.SetSize(10,10,False)
#
        self.bmp.SetBitmap(bitmap)
        self.bmp.parent = self
        self.parent.MyAddBmp( self.bmp, pos_x, pos_y, wx.BLACK_PEN )

        # 本体の四角形を作成
        self.baseBox = makeRectangle(self, self.x_size, self.y_size)
        self.parent.MyAddShape(self.baseBox, pos_x, pos_y, wx.BLACK_PEN, wx.Brush(self.color, wx.SOLID), "" ,0)
        # 前述のパーツ郡をCompositeShapeとして、親子構造にする
        self.body = makeCompositeShape(self)
        self.body.AddChild(self.baseBox)
        self.body.AddChild(self.bmp)
        self.body.AddChild(self.text)
#
        self.constraint = ogl.OGLConstraint(ogl.gyCONSTRAINT_ALIGNED_TOP, self.baseBox, [self.bmp])
        self.body.AddConstraint(self.constraint)
        self.constraint2 = ogl.OGLConstraint(ogl.gyCONSTRAINT_CENTRED_HORIZONTALLY, self.bmp, [self.text])
        self.body.AddConstraint(self.constraint2)
#        self.constraint = ogl.OGLConstraint(ogl.gyCONSTRAINT_CENTRED_HORIZONTALLY, self.baseBox, [self.text])
#        self.body.AddConstraint(self.constraint)
        self.body.Recompute()
        self.body.CalculateSize()
###
        self.parent.MyAddShape(
            self.body, pos_x, pos_y, wx.BLACK_PEN, wx.Brush(self.color, wx.SOLID), "" ,0)
        # 比率計算用の最後の座標および拡大縮小用の最後のサイズ
        self.baseBox.lastx = self.body.GetX()
        self.baseBox.lasty = self.body.GetY()
        self.lastBBoxWidth, self.lastBBoxHeight = self.baseBox.GetBoundingBoxMin()

        if rot == 0:
            # インポートの作成（回転処理時は行わない）
            self.in_dict = {}
            port_x = self.x - (POLYGON_SIZE*self.ratioW)*2/3
            port_y = self.y + (POLYGON_SIZE*self.ratioH)/2
            for inp in self.in_list :
                self.in_dict[inp['name']] = GRtcIn(self, self.ns_dict, self.fullpath,
                                            inp,
                                            port_x, port_y)
                port_y = port_y + (POLYGON_SIZE*self.ratioH)*2
#
            # アウトポートの作成（回転処理時は行わない）
            port_x = self.x + (BOX_WIDTH*self.ratioW) - (POLYGON_SIZE*self.ratioW)/3
            port_y = self.y + (POLYGON_SIZE*self.ratioH)/2
            self.out_dict = {}
            for outp in self.out_list :
                self.out_dict[outp['name']] = GRtcOut(self, self.ns_dict, self.fullpath,
                                         outp, 
                                         port_x, port_y)
                port_y = port_y + (POLYGON_SIZE*self.ratioH)*2

        self.portToFlash()


    def dmove(self, dc, movex, movey):
        """コンポーネント図形の移動処理
        インポート、アウトポート図形および関連する線も移動

        [引数]
        dc     -- 描画するデバイス・コンテキストを指定
        movex  -- ｘ座標の相対移動量を指定
        movey  -- ｙ座標の相対移動量を指定

        [戻り値]
        void
        """
        canvas = self.body.GetCanvas()

        self.x = self.body.GetX() + movex
        self.y = self.body.GetY() + movey
        self.body.Erase(dc)
        self.body.Move(dc, self.x, self.y)
        # インポート、アウトポート図形の移動
        for inp in self.in_list:
            if inp['name'] in self.in_dict.keys():
                self.in_dict[inp['name']].dmove(dc, movex, movey)
        for outp in self.out_list:
            if outp['name'] in self.out_dict.keys():
                self.out_dict[outp['name']].dmove(dc, movex, movey)


    def selected(self):
        """コンポーネント図形の選択処理（色の変更）

        [引数]
        なし

        [戻り値]
        void
        """
        self.baseBox.SetBrush(wx.Brush(wx.NamedColor(SELECTED_COLOR)))
        self.body.Flash()
        # インポート／アウトポート図形をフラッシュ（再描画）し図形を最前面に表示
        self.portToFlash()
        
    def unselected(self,dc):
        """コンポーネント図形の非選択処理（色の変更）

        [引数]
        dc     -- 描画していたデバイス・コンテキストを指定

        [戻り値]
        void
        """
        self.checkCompState()
        setBodyColor(self.baseBox, self.state)
        # インポート／アウトポート図形をフラッシュ（再描画）し図形を最前面に表示
        self.portToFlash()

    def updatePolygonSize(self, x, y, ratioW, ratioH):
        """コンポーネント図形（インポート／アウトポート）のサイズ変更処理
        コンポーネント図形本体のサイズ変更はデフォルト（システム側）で行われる

        [引数]
        x      -- 描画するｘ座標を指定
        y      -- 描画するｙ座標を指定
        ratioW -- サイズ変更するWidthの比率を指定
        ratioH -- サイズ変更するHeightの比率を指定

        [戻り値]
        void
        """
        self.x = x
        self.y = y
        self.ratioW = ratioW
        self.ratioH = ratioH
        self.x_size, self.y_size = self.baseBox.GetBoundingBoxMin()

        canvas = self.body.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)

        # 最小サイズのチェック：初期サイズより小さかったら初期サイズに変更
        if self.lastRot == 'TB':
            minW, minH = self.minHeight, self.minWidth
        else:
            minH, minW = self.minHeight, self.minWidth
        if minW > self.x_size or minH > self.y_size:
            self.ratioW = 1.0
            self.ratioH = 1.0
            self.x_size = minW
            self.y_size = minH
            # 一度図形を削除し、上記で設定したサイズで再作成
            self.body.Select(False, dc)
            tmp = canvas.selected.index(self.baseBox)
            del canvas.selected[tmp]
            self.removeWidget(dc,1)
            self.createWidget(1)
            self.baseBox.Select(True, dc)
            canvas.selected.append(self.baseBox)


        x_size = self.px_size 
        y_size = self.py_size 
        # インポートが存在する時、インポートのサイズを変更する
        if len(self.in_dict) > 0:
            in_pos = self.in_dict[self.in_list[0]['name']].position

            port_x = self.x - self.x_size/2 + x_size -1
            port_y = self.y - self.y_size/2 + y_size -1
            # ポートのpositionによる位置調整
            if in_pos == 'Right':
                port_x = self.x + self.x_size/2 + x_size/6
            elif in_pos == 'Left':
                port_x = self.x - self.x_size/2 - x_size/6
            elif in_pos == 'Top':
                port_y = self.y - self.y_size/2 - y_size/6
            elif in_pos == 'Bottom':
                port_y = self.y + self.y_size/2 + y_size/6

            for inp in self.in_list:
                if inp['name'] in self.in_dict.keys():
                    self.in_dict[inp['name']].updatePolygonSize(port_x, port_y, ratioW, ratioH)
                    # ポートのpositionによる位置調整
                    if in_pos == 'Right' or in_pos == 'Left':
                        port_y = port_y + y_size*2
                    else:
                        port_x = port_x + x_size*2
#

        # アウトポートが存在する時、アウトポートのサイズを変更する
        if len(self.out_dict) > 0:
            out_pos = self.out_dict[self.out_list[0]['name']].position

            port_x = self.x - self.x_size/2 + x_size -1
            port_y = self.y - self.y_size/2 + y_size -1
            # ポートのpositionによる位置調整
            if out_pos == 'Right':
                port_x = self.x + self.x_size/2 + x_size/6
            elif out_pos == 'Left':
                port_x = self.x - self.x_size/2 - x_size/6
            elif out_pos == 'Top':
                port_y = self.y - self.y_size/2 - y_size/6
            elif out_pos == 'Bottom':
                port_y = self.y + self.y_size/2 + y_size/6

            for outp in self.out_list:
                if outp['name'] in self.out_dict.keys():
                    self.out_dict[outp['name']].updatePolygonSize(port_x, port_y, ratioW, ratioH)
                    # ポートのpositionによる位置調整
                    if out_pos == 'Right' or out_pos == 'Left':
                        port_y = port_y + y_size*2
                    else:
                        port_x = port_x + x_size*self.ratioW*2

        # コンポーネント名称の表示位置調整
        if self.lastRot == 'LR':
            self.text_y = self.y + self.y_size/2 + POLYGON_SIZE
        else:
            self.text_y = self.y + self.y_size/2 + self.py_size
        self.text.Erase(dc)
        self.text.SetY(self.text_y)
        self.text.Flash()

        # ビットマップの表示位置調整
        tmpw, tmph = self.bmp.GetBoundingBoxMin()
        pos_y = self.y - self.y_size/2 + tmph/2
        if self.lastRot != 'LR':
            pos_y = pos_y + self.py_size/2
        self.bmp.Erase(dc)
        self.bmp.SetY(pos_y)
        self.bmp.Flash()

        self.selected()
        self.portToFlash()

    def reversesBody(self):
        """左右反転（回転）処理

        [引数]
        なし

        [戻り値]
        void
        """
        # 左右トグルフラグの更新
        self.revTogle = self.revTogle + 1
        if self.revTogle % 2 == 0 :
            self.revTogle = 0
        
        self.x = self.baseBox.GetX()
        self.y = self.baseBox.GetY()
        tmp = max(len(self.in_list), len(self.out_list))
        center_x = self.baseBox.GetX()
        center_y = self.baseBox.GetY()
        # 最後に左右／上下どちらの回転処理が行われたかでWidth/Heightの入れ替え
        if self.lastRot == 'LR':
            self.x_size, self.y_size = self.baseBox.GetBoundingBoxMin()
            self.xy_swap = 0
        else:
            self.y_size, self.x_size = self.baseBox.GetBoundingBoxMin()
            self.xy_swap = 1

        canvas = self.body.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        # 一度図形を削除し、上記で設定したサイズで再作成
        self.removeWidget(dc,1)
        self.createWidget(1)

        # 複数ポートの表示座標を決める基準（POLYGON_SIZE）を決定
        if self.xy_swap == 1:
            p_size = self.px_size
        else:
            p_size = self.py_size
        if self.revTogle == 1:  # inport is right side of body
            port_x = self.x - self.x_size/2 - p_size/6
            port_y = self.y - self.y_size/2 + p_size-1
            for inp in self.in_list:
                if inp['name'] in self.in_dict.keys():
                    self.in_dict[inp['name']].position = 'Left'
                    self.in_dict[inp['name']].updatePolygonSize(port_x, port_y, 1, 1)
                    port_y = port_y + p_size*2
#
            port_x = self.x + self.x_size/2 + p_size/6
            port_y = self.y - self.y_size/2 + p_size-1
            for outp in self.out_list:
                if outp['name'] in self.out_dict.keys():
                    self.out_dict[outp['name']].position = 'Right'
                    self.out_dict[outp['name']].updatePolygonSize(port_x, port_y, 1, 1)
                    port_y = port_y + p_size*2

        else :                  # inport is left side of body
            port_x = self.x + self.x_size/2 + p_size/6
            port_y = self.y - self.y_size/2 + p_size-1
            for inp in self.in_list:
                if inp['name'] in self.in_dict.keys():
                    self.in_dict[inp['name']].position = 'Right'
                    self.in_dict[inp['name']].updatePolygonSize(port_x, port_y, 1, 1)
                    port_y = port_y + p_size*2
#
            port_x = self.x - self.x_size/2 - p_size/6
            port_y = self.y - self.y_size/2 + p_size-1
            for outp in self.out_list:
                if outp['name'] in self.out_dict.keys():
                    self.out_dict[outp['name']].position = 'Left'
                    self.out_dict[outp['name']].updatePolygonSize(port_x, port_y, 1, 1)
                    port_y = port_y + p_size*2

        # テキストの表示位置を再計算
        self.text_y = self.y + self.y_size/2 + POLYGON_SIZE
        self.text.Erase(dc)
        self.text.SetY(self.text_y)
        self.text.Flash()

        self.lastRot = 'LR'
        self.xy_swap = 0


    def rotatesBody(self):   ## +-90 degrees
        """上下回転処理

        [引数]
        なし

        [戻り値]
        void
        """
        tmp = max(len(self.in_list), len(self.out_list))
        self.x = center_x = self.baseBox.GetX()
        self.y = center_y = self.baseBox.GetY()
        # 最後に左右／上下どちらの回転処理が行われたかでWidth/Heightの入れ替え
        if self.lastRot == 'LR':
            self.y_size, self.x_size = self.baseBox.GetBoundingBoxMin()
            self.xy_swap = 1
        else:
            self.x_size, self.y_size = self.baseBox.GetBoundingBoxMin()
            self.xy_swap = 0

        canvas = self.body.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)

        # 一度図形を削除し、上記で設定したサイズで再作成
        self.removeWidget(dc,1)
        self.createWidget(1)


        # 上下トグルフラグの更新
        self.rotTogle = self.rotTogle + 1
        if self.rotTogle % 2 == 0 :
            self.rotTogle = 0

        # 複数ポートの表示座標を決める基準（POLYGON_SIZE）を決定
        if self.xy_swap == 1:
            p_size = self.py_size
        else:
            p_size = self.px_size
        # インポート／アウトポートのサイズ変更
        if self.rotTogle == 1:  # inport is top side of body
            port_x = self.x - self.x_size/2 + p_size-1
            port_y = self.y - self.y_size/2 - p_size/6
            for inp in self.in_list:
                if inp['name'] in self.in_dict.keys():
                    self.in_dict[inp['name']].position = 'Top'
                    self.in_dict[inp['name']].updatePolygonSize(port_x, port_y, 1, 1)
                    port_x = port_x + p_size*2
#
            port_x = self.x - self.x_size/2 + p_size-1
            port_y = self.y + self.y_size/2 + p_size/6
            for outp in self.out_list:
                if outp['name'] in self.out_dict.keys():
                    self.out_dict[outp['name']].position = 'Bottom'
                    self.out_dict[outp['name']].updatePolygonSize(port_x, port_y, 1, 1)
                    port_x = port_x + p_size*2

        else :                  # inport is bottom side of body
            port_x = self.x - self.x_size/2 + p_size-1
            port_y = self.y + self.y_size/2 + p_size/6
            for inp in self.in_list:
                if inp['name'] in self.in_dict.keys():
                    self.in_dict[inp['name']].position = 'Bottom'
                    self.in_dict[inp['name']].updatePolygonSize(port_x, port_y, 1, 1)
                    port_x = port_x + p_size*2
#
            port_x = self.x - self.x_size/2 + p_size-1
            port_y = self.y - self.y_size/2 - p_size/6
            for outp in self.out_list:
                if outp['name'] in self.out_dict.keys():
                    self.out_dict[outp['name']].position = 'Top'
                    self.out_dict[outp['name']].updatePolygonSize(port_x, port_y, 1, 1)
                    port_x = port_x + p_size*2

        # テキストの表示位置調整
        self.text_y = self.y + self.y_size/2 + self.py_size
        self.text.Erase(dc)
        self.text.SetY(self.text_y)
        self.text.Flash()

        # ビットマップの表示位置調整
        tmpw, tmph = self.bmp.GetBoundingBoxMin()
        pos_y = self.y - self.y_size/2 + tmph/2 + self.py_size/2
        self.bmp.Erase(dc)
        self.bmp.SetY(pos_y)
        self.bmp.Flash()

        self.lastRot = 'TB'
        self.xy_swap = 0

#----------------------------------------------------------------------
class MyEvtHandlerBmp(ogl.ShapeEvtHandler):
    """ビットマップ用のダミーイベントクラス"""
    def __init__(self, log, frame):
        """クラスの初期化

        [引数]
        log     -- ログ出力クラスのオブジェクト
        frame   -- ステータスバーのオブジェクト
        ※上記引数は、demoプログラムの名残：削除可

        [戻り値]
        void
        """
        ogl.ShapeEvtHandler.__init__(self)
        self.log = log
        self.statbarFrame = frame

    def OnEndDragLeft(self, x, y, keys = 0, attachment = 0):
        """ドラッグ終了時に呼ばれるイベントハンドラ
        処理を未実装（passのみ）でイベントを登録し、デフォルトの動作を抑止

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント

        [戻り値]
        void
        """
        pass

    def OnLeftClick(self, x, y, keys = 0, attachment = 0):
        """マウス左クリック時に呼ばれるイベントハンドラ
        処理を未実装（passのみ）でイベントを登録し、デフォルトの動作を抑止

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント

        [戻り値]
        void
        """
        pass

    def OnSizingEndDragLeft(self, pt, x, y, keys, attch):
        """サイズ変更終了時に呼ばれるイベントハンドラ
        処理を未実装（passのみ）でイベントを登録し、デフォルトの動作を抑止

        [引数]
        pt         -- コントロールポイント
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント

        [戻り値]
        void
        """
        pass

    def OnBeginDragLeft(self, x, y, keys, attachment):
        """ドラッグ開始時に呼ばれるイベントハンドラ
        処理を未実装（passのみ）でイベントを登録し、デフォルトの動作を抑止

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント

        [戻り値]
        void
        """
        pass

#----------------------------------------------------------------------
class MyEvtHandlerOval(ogl.ShapeEvtHandler):
    """線移動用の円のイベントクラス"""
    def __init__(self, log, frame):
        """クラスの初期化（ShapeEvtHandlerの作成）

        [引数]
        log     -- ログ出力クラスのオブジェクト
        frame   -- ステータスバーのオブジェクト
        ※上記引数は、demoプログラムの名残：削除可

        [戻り値]
        void
        """
        ogl.ShapeEvtHandler.__init__(self)
        self.log = log
        self.statbarFrame = frame

    def OnEndDragLeft(self, x, y, keys = 0, attachment = 0):
        """ドラッグ終了時に呼ばれるイベントハンドラ
        ドラッグフラグをoffにする

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        self.dragOn = 0

    def OnBeginDragLeft(self, x, y, keys=0, attachment=0):
        """ドラッグ開始時に呼ばれるイベントハンドラ
        ドラッグフラグをonにする

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        self.dragOn = 1

    def OnDragLeft(self, draw, x, y, keys, attachment):
        """ドラッグ開始時に呼ばれるイベントハンドラ
        移動用の円をドラッグで移動（円のdmoveメソッド内で関連する線も移動させる）

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
#        self.base_OnBeginDragLeft(x, y, keys, attachment)
        shape = self.GetShape()

        if self.dragOn == 1:

            # メモリDC（BufferedDC)の生成：メモリ上に絵を描き、Redrawで表に表示
            canvas = shape.GetCanvas()
            dc = getBufferedDC(canvas)
            canvas.PrepareDC(dc)

            movex = x - shape.lastx
            movey = y - shape.lasty

            shape.parent.dmove(dc,movex,movey)

            shape.lastx = shape.GetX()
            shape.lasty = shape.GetY()

            canvas.Redraw(dc)

#----------------------------------------------------------------------
class MyEvtHandlerDummy(ogl.ShapeEvtHandler):
    """ダミーイベントハンドラ"""
    def __init__(self, log, frame):
        """クラスの初期化

        [引数]
        log     -- ログ出力クラスのオブジェクト
        frame   -- ステータスバーのオブジェクト
        ※上記引数は、demoプログラムの名残：削除可

        [戻り値]
        void
        """
        ogl.ShapeEvtHandler.__init__(self)
        self.log = log
        ogl.ShapeEvtHandler.__init__(self)
        self.log = log
        self.statbarFrame = frame

    def OnEndDragLeft(self, x, y, keys = 0, attachment = 0):
        """ドラッグ終了時に呼ばれるイベントハンドラ
        処理を未実装（passのみ）でイベントを登録し、デフォルトの動作を抑止

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        pass

    def OnDragLeft(self, draw, x, y, keys=0, attachment=0):
        """ドラッグ時に呼ばれるイベントハンドラ
        処理を未実装（passのみ）でイベントを登録し、デフォルトの動作を抑止

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        pass

    def OnBeginDragLeft(self, x, y, keys, attachment):
        """ドラッグ開始時に呼ばれるイベントハンドラ
        処理を未実装（passのみ）でイベントを登録し、デフォルトの動作を抑止

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        pass

    def OnLeftClick(self, x, y, keys = 0, attachment = 0):
        """マウス左クリックに呼ばれるイベントハンドラ
        処理を未実装（passのみ）でイベントを登録し、デフォルトの動作を抑止

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        pass


#----------------------------------------------------------------------
class MyEvtHandlerLine(ogl.ShapeEvtHandler):
    """線のイベントクラス"""
    def __init__(self ):
        """クラスの初期化

        [引数]
        なし

        [戻り値]
        void
        """
        ogl.ShapeEvtHandler.__init__(self)

    def OnLeftClick(self, x, y, keys = 0, attachment = 0):
        """マウス左クリック時に呼ばれるイベントハンドラ
        線の選択／解除を行う
        選択処理では、移動用の円も作成する

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        shape = self.GetShape()
#        print shape.__class__, shape.GetClassName()
        canvas = shape.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        # 線が選択済み
        if shape in canvas.selected:
            # 選択状態の解除
            for obj in shape.parent.lines:
                obj.Select(False, dc)
            shape.parent.unselected(dc)
            for obj in canvas.selected:
                if shape == obj:
                    obj.parent.unselected(dc)
                    idx = canvas.selected.index(obj)
                    del canvas.selected[idx]
        # 線が未選択
        else:
            # 選択状態に移行
            redraw = False
            shapeList = canvas.GetDiagram().GetShapeList()

            if canvas.selected and keys != 1:
                for s in canvas.selected:
                    s.Select(False, dc)
                    s.parent.unselected(dc)
                canvas.selected = []
                canvas.Redraw(dc)

            shape.parent.selected()

            for obj in shape.parent.lines:
                canvas.selected.append(obj)

            # create oval on line
            line = shape.parent
            if (line.coordT == None) or (len(line.coordT) == 2):
                return

            # 線移動用の円を生成
            num = len(line.coordT)
            line.oval_dict = {}
            for oval_id in range(1,num-2):
                line_pos_0 = line.coordT[oval_id]
                line_pos_1 = line.coordT[oval_id+1]
                if line_pos_0[0] == line_pos_1[0] and line_pos_0[1] != line_pos_1[1]: # width line
                    hight = line_pos_0[1] - line_pos_1[1]
                    pos_y = line_pos_1[1] + (hight/2)
                    pos_x = line_pos_0[0]
                    tag = (oval_id, "oval_width_pos")
                elif  line_pos_0[0] != line_pos_1[0] and line_pos_0[1] == line_pos_1[1] : # length line
                    width = line_pos_0[0] - line_pos_1[0]
                    pos_x = line_pos_1[0] + (width/2)
                    pos_y = line_pos_1[1]
                    tag = (oval_id, "oval_length_pos")
                line.oval_dict[oval_id] = GRectOval(line, tag, pos_x, pos_y)
                line.parent.parent.parent.MyAddOval(line.oval_dict[oval_id].body, pos_x, pos_y)

        canvas.Redraw(dc)

#----------------------------------------------------------------------

class MyEvtHandler(ogl.ShapeEvtHandler):
    """コンポーネント図形（四角形）本体のイベントクラス"""
    def __init__(self, log, frame):
        """クラスの初期化

        [引数]
        log     -- ログ出力クラスのオブジェクト
        frame   -- ステータスバーのオブジェクト
        ※上記引数は、demoプログラムの名残：削除可

        [戻り値]
        void
        """
        ogl.ShapeEvtHandler.__init__(self)
        self.log = log
        self.statbarFrame = frame
        self.dragOn = 0

    def UpdateStatusBar(self, shape):
        """ステータスバーへ表示する情報の更新

        [引数]
        shape   -- 図形のオブジェクトを指定

        [戻り値]
        void
        """
        x,y = shape.GetX(), shape.GetY()
        width, height = shape.GetBoundingBoxMax()
        self.statbarFrame.SetStatusText("Pos: (%d,%d)  Size: (%d, %d)" %
                                        (x, y, width, height))


    def OnLeftClick(self, x, y, keys = 0, attachment = 0):
        """マウス左クリック時に呼ばれるイベントハンドラ
        本体の選択／解除を行う

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        shape = self.GetShape()
#        print shape.__class__, shape.GetClassName()
        canvas = shape.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)

        # クリック時、プロファイル表示
        ref = self.statbarFrame.myDict.GetObjRefToFullpath(shape.parent.fullpath)
        try:
            ref = ref._narrow(RTM.RTCBase)
            self.statbarFrame.profilepanel.RefreshProfile(ref._get_profile())
        except:
            except_mess("obj-ref error:")

        # イベントで呼び出された図形が選択済みの場合
        if shape in canvas.selected:
        #if shape.parent.tag == 'body' or shape.parent.tag == 'line':
            # 選択解除
            canvas.lineFrom = None
            shape.Select(False, dc)
            shape.parent.unselected(dc)
            for obj in canvas.selected:
                if shape == obj:
                    idx = canvas.selected.index(obj)
                    del canvas.selected[idx]
            canvas.Redraw(dc)
        # イベントで呼び出された図形が未選択の場合
        else:
            # 選択状態へ移行
            canvas.lineFrom = None
            redraw = False
            shapeList = canvas.GetDiagram().GetShapeList()

            shape.Select(True, dc)
            shape.parent.selected()

            if canvas.selected and keys != 1:
                for s in canvas.selected:
                    s.Select(False, dc)
                    s.parent.unselected(dc)
                canvas.selected = []
                canvas.Redraw(dc)

            canvas.selected.append(shape)

        self.UpdateStatusBar(shape)


    def OnEndDragLeft(self, x, y, keys = 0, attachment = 0):
        """ドラッグ終了時に呼ばれるイベントハンドラ
        選択状態の全図形を移動

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        if self.dragOn == 1:
            shape = self.GetShape()
#            self.base_OnEndDragLeft(x, y, keys, attachment)

            canvas = shape.GetCanvas()
            dc = wx.ClientDC(canvas)
            canvas.PrepareDC(dc)

            movex = x - shape.lastx
            movey = y - shape.lasty
            # 選択状態の全図形を移動
            for obj in canvas.selected:

                # body/inport/outport
                obj.parent.dmove(dc,movex,movey)

                obj.lastx = obj.GetX()
                obj.lasty = obj.GetY()

                if obj.parent.tag != 'line':
                    obj.Select(True, dc)

            canvas.Redraw(dc)
            self.dragOn = 0

            self.UpdateStatusBar(shape)

    def OnSizingEndDragLeft(self, pt, x, y, keys, attch):
        """サイズ変更終了時に呼ばれるイベントハンドラ

        [引数]
        pt         -- コントロールポイント
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        # デフォルトのサイズ変更イベントハンドラ呼び出し
        self.base_OnSizingEndDragLeft(pt, x, y, keys, attch)

        shape = self.GetShape()
        canvas = shape.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        # サイズ変更時のWidth,Heightの比率算出
        width, height = shape.GetBoundingBoxMax()
        ratioW = width / shape.parent.lastBBoxWidth
        ratioH = height / shape.parent.lastBBoxHeight
        tmpx,tmpy = shape.GetX(), shape.GetY()
        # サイズ変更
        shape.parent.updatePolygonSize(tmpx,tmpy,ratioW,ratioH)
        self.UpdateStatusBar(shape)
        canvas.Redraw(dc)
        shape.parent.lastBBoxWidth, shape.parent.lastBBoxHeight = shape.GetBoundingBoxMax()

    def OnDragLeft(self, draw, x, y, keys, attachment):
        """ドラッグ時に呼ばれるイベントハンドラ
        選択状態の全図形を移動

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        shape = self.GetShape()
        canvas = shape.GetCanvas()
        if self.dragOn == 1:
            if shape not in canvas.selected:
                pass
            else:
                width, height = shape.GetBoundingBoxMax()
                self.statbarFrame.SetStatusText("Pos: (%d,%d)  Size: (%d, %d)" %
                                                (x, y, width, height))
#                self.base_OnDragLeft(1, x, y, keys, attachment)

                # メモリDC(BufferedDC)の生成：メモリ上に絵を描いて、Redrawdeで表に表示
                dc = getBufferedDC(canvas)
                canvas.PrepareDC(dc)

                # 選択済みの図形を移動
                movex = x - shape.lastx
                movey = y - shape.lasty
                for obj in canvas.selected:
                    obj.DeleteControlPoints()
                    # inport/outport
                    obj.parent.dmove(dc,movex,movey)

                    obj.lastx = obj.GetX()
                    obj.lasty = obj.GetY()

                canvas.Redraw(dc)

    def OnBeginDragLeft(self, x, y, keys, attachment):
        """ドラッグ開始時に呼ばれるイベントハンドラ

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        shape = self.GetShape()
        canvas = shape.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        if shape not in canvas.selected:
            shape.DeleteControlPoints()

            if canvas.selected and keys != 1:
                for s in canvas.selected:
                    s.Select(False, dc)
                    s.parent.unselected(dc)
                canvas.selected = []
                canvas.Redraw(dc)

            shape.Select(True, dc)
            shape.parent.selected()
            canvas.selected.append(shape)

        else:
            for s in canvas.selected:
                if s.parent.tag == 'line':
                    s.Select(False, dc)
                    s.parent.unselected(dc)
                    idx = canvas.selected.index(s)
                    del canvas.selected[idx]
            canvas.Redraw(dc)

            self.log.write("OnBeginDragLeft: %s, %s, %s\n" % (x, y, keys))
#            self.base_OnBeginDragLeft(x, y, keys, attachment)
        self.dragOn = 1

#----------------------------------------------------------------------

class MyPortEvtHandler(ogl.ShapeEvtHandler):
    """ポートのイベントクラス"""
    def __init__(self, log, frame):
        """クラスの初期化

        [引数]
        log     -- ログ出力クラスのオブジェクト
        frame   -- ステータスバーのオブジェクト
        ※上記引数は、demoプログラムの名残：削除可

        [戻り値]
        void
        """
        ogl.ShapeEvtHandler.__init__(self)
        self.log = log
        self.statbarFrame = frame
        self.dragOn = 0

    def UpdateStatusBar(self, shape):
        """ステータスバーへ表示する情報の更新

        [引数]
        shape   -- 図形のオブジェクトを指定

        [戻り値]
        void
        """
        x,y = shape.GetX(), shape.GetY()
        width, height = shape.GetBoundingBoxMax()
        self.statbarFrame.SetStatusText("Pos: (%d,%d)  Size: (%d, %d)" %
                                        (x, y, width, height))


    def OnLeftClick(self, x, y, keys = 0, attachment = 0):
        """マウス左クリック時に呼ばれるイベントハンドラ
        線を引く為の、開始点or終了点を設定
        終了点を設定後は、２点間で線を引く

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        shape = self.GetShape()
#        print shape.__class__, shape.GetClassName()
        canvas = shape.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        ref = shape.parent.ns_dict.GetObjRefToFullpath(shape.parent.fullpath)
        try:
            ref = ref._narrow(RTM.RTCBase)
            tmp = shape.parent.ns_dict.GetCompState(shape.parent.fullpath)
            if tmp >= RTM.RTComponent.RTC_ABORTING or tmp == 0:
                ref = None
        except :
            except_mess("except error:")
            ref = None

        if not ref:
            return

        if canvas.lineFrom == None:
            canvas.lineFrom = shape
        elif canvas.lineFrom != shape:
            if canvas.lineFrom.parent.tag == 'in':
                checktag = 'out'
            else:
                checktag = 'in'
            if shape.parent.tag != checktag:
                return
            if shape.parent.fullpath == canvas.lineFrom.parent.fullpath:
                return

            canvas.lineTo = shape
            line = GRtcLine(canvas,shape.parent)
            line.setLine2port(canvas, dc)
            line.g_outp.connect(line.idx, line.g_outp.subscription_type)

            canvas.lineFrom = None
            canvas.lineTo = None
            canvas.Redraw(dc)

        self.UpdateStatusBar(shape)


    def OnEndDragLeft(self, x, y, keys = 0, attachment = 0):
        """ドラッグ終了時に呼ばれるイベントハンドラ
        ポート上にマウスカーソルがあれば線を引く

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        shape = self.GetShape()
        ref = shape.parent.ns_dict.GetObjRefToFullpath(shape.parent.fullpath)
        try:
            ref = ref._narrow(RTM.RTCBase)
            tmp = shape.parent.ns_dict.GetCompState(shape.parent.fullpath)
            if tmp >= RTM.RTComponent.RTC_ABORTING or tmp == 0:
                ref = None
        except :
            except_mess("except error:")
            ref = None
        if not ref:
            return

        if self.dragOn == 1:
#            shape.parent.parent.blink.Stop()
            self.dragOn = 0
            canvas = shape.GetCanvas()
            dc = wx.ClientDC(canvas)
            canvas.PrepareDC(dc)
            canvas.moveLine.removeWidget(dc)
            canvas.Redraw(dc)
            tmpShape = canvas.FindShape(x,y)
            if tmpShape == 0 or not hasattr(tmpShape[0], "parent") or not hasattr(tmpShape[0].parent, "ns_dict"):
                return
            ref = tmpShape[0].parent.ns_dict.GetObjRefToFullpath(tmpShape[0].parent.fullpath)
            try:
                ref = ref._narrow(RTM.RTCBase)
                tmp = tmpShape[0].parent.ns_dict.GetCompState(tmpShape[0].parent.fullpath)
                if tmp >= RTM.RTComponent.RTC_ABORTING or tmp == 0:
                    ref = None
                if shape.parent.fullpath == tmpShape[0].parent.fullpath:
                    ref = None
            except :
                except_mess("except error:")
                ref = None
            if not ref:
                return

            if canvas.lineFrom.parent.tag == 'in':
                checktag = 'out'
            else:
                checktag = 'in'
            if tmpShape[0].parent.tag == checktag:
                canvas.lineTo = tmpShape[0]
                line = GRtcLine(canvas,shape.parent)
                line.setLine2port(canvas, dc)
                line.g_outp.connect(line.idx, line.g_outp.subscription_type)

                canvas.lineFrom = None
                canvas.lineTo = None
                canvas.Redraw(dc)
        else:
            pass

    def OnDragLeft(self, draw, x, y, keys, attachment):
        """ドラッグ時に呼ばれるイベントハンドラ
        マウスカーソルと始点間で線を引く

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        shape = self.GetShape()
        if self.dragOn == 1:
            canvas = shape.GetCanvas()
            dc = getBufferedDC(canvas)
            canvas.PrepareDC(dc)

            # create line
            canvas.moveLine.setPoints(shape.GetX(), shape.GetY(), x, y)
            canvas.moveLine.lines[0].SetPen(wx.Pen(SELECTED_COLOR, 1))
            canvas.moveLine.lines[0].Show(True)
            canvas.Redraw(dc)
        else:
            pass

    def OnBeginDragLeft(self, x, y, keys, attachment):
        """ドラッグ開始時に呼ばれるイベントハンドラ
        ドラッグによる線引き処理の開始(オブジェクトリファレンスが存在する場合）

        [引数]
        x          -- イベント時のｘ座標
        y          -- イベント時のｙ座標
        keys       -- キー押下状態（SHIFT、CTRL）
        attachment -- アタッチメント（未使用）

        [戻り値]
        void
        """
        self.log.write("OnBeginDragLeft: %s, %s, %s\n" % (x, y, keys))
        self.dragOn = 1
        shape = self.GetShape()
        ref = shape.parent.ns_dict.GetObjRefToFullpath(shape.parent.fullpath)
        try:
            ref = ref._narrow(RTM.RTCBase)
            tmp = shape.parent.ns_dict.GetCompState(shape.parent.fullpath)
            if tmp >= RTM.RTComponent.RTC_ABORTING or tmp == 0:
                ref = None
        except :
            ref = None
            except_mess("except error:")
        if ref:
            canvas = shape.GetCanvas()
            dc = wx.ClientDC(canvas)
            canvas.PrepareDC(dc)
            canvas.lineFrom = shape
            # make line
            canvas.moveLine = GRtcLine(canvas, shape.parent)
            canvas.moveLine.setPoints(shape.GetX(), shape.GetY(), x, y)
            canvas.moveLine.lines[0].SetPen(wx.Pen(SELECTED_COLOR, 1))
            canvas.moveLine.lines[0].Show(True)
#            shape.parent.parent.blink.setBlinkState(shape.parent.parent, 'inactive', 'active')
#            shape.parent.parent.blink.Start(500)
        else:
            self.dragOn = 0


#----------------------------------------------------------------------

class RtdSystemDraw(ogl.ShapeCanvas):
    """図形描画用のキャンバス生成クラス"""
    def __init__(self, parent, log, frame):
        """クラスの初期化

        [引数]
        log     -- ログ出力クラスのオブジェクト
        frame   -- ステータスバーのオブジェクト
        ※上記引数は、demoプログラムの名残：削除可
        parent  -- 親ウィンドウを指定

        [戻り値]
        void
        """
        ogl.ShapeCanvas.__init__(self, parent)

        maxWidth  = 1000
        maxHeight = 1000
#        self.SetScrollbars(20, 20, maxWidth/20, maxHeight/20)

        self.x_size = maxWidth
        self.y_size = maxHeight
        self.log = log
        self.frame = frame
        self.SetBackgroundColour(wx.WHITE)
        self.diagram = ogl.Diagram()
        self.save_gdi = []
        self.SetDiagram(self.diagram)
        self.diagram.SetCanvas(self)
        self.tooltip = None
        self.rtc_dict = {}
        self.rtc_list = []
        canvas = self.diagram.GetCanvas()
        canvas.lineFrom = None
        canvas.lineTo = None
        canvas.line_idx = 0
        canvas.line = {}
        canvas.moveLine = None
        canvas.selected = []
        canvas.viewMode = False

        rRectBrush = wx.Brush("MEDIUM TURQUOISE", wx.SOLID)
        dsBrush = wx.Brush("WHITE", wx.SOLID)

        # ドラッグ＆ドロップ：ドロップターゲットの設定
        dt = MyTextDropTarget(self, log)
        self.SetDropTarget(dt)

        # イベントの割付
        self.Bind(wx.EVT_WINDOW_DESTROY, self.OnDestroy)
        self.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_MIDDLE_DOWN, self.OnMiddleDown)
        self.Bind(wx.EVT_RIGHT_DOWN, self.OnRightDown)
        self.Bind(wx.EVT_RIGHT_UP,   self.OnRightUp)

    def changeCompColor(self, fullname, state) :
        """コンポーネント単体の色を変更（rtc_state()参照）

        [引数]
        fullname   -- コンポーネントのフルパス名

        [戻り値]
        void
        """
        if fullname in self.rtc_list:
#            self.rtc_dict[fullname].refresh()
            self.rtc_dict[fullname].changeBodyColor(state)
            self.rtc_dict[fullname].state = state

    def search_g_inp(self, inp_ref) :
        """インポートの検索

        [引数]
        inp_ref   -- インポートのオブジェクトリファレンス

        [戻り値]
        g_inp -- インポートのディクショナリー
        """
        print "len rtc_list:",len(self.rtc_list)
        for rtc_name in self.rtc_list :
            g_rtc = self.rtc_dict[rtc_name]
            ref = g_rtc.ns_dict.GetObjRefToFullpath(g_rtc.fullpath)
            if ref :
                print "len in_list:",len(g_rtc.in_list)
                for inp in g_rtc.in_list :
                    print "inp_ref:",inp_ref, " == ", g_rtc.in_dict[inp['name']].inport['ref']
                    if inp_ref._is_equivalent(g_rtc.in_dict[inp['name']].inport['ref']) :
                        print "_is_equivalent is OK!!!"
                        return g_rtc.in_dict[inp['name']]

    def openFileDialog(self):
        wildcard = "*.xml |*.xml| *.* |*.*"
        dialog = wx.FileDialog( self, strOPEN_TITLE, defaultDir=os.getcwd(),
                                defaultFile="", wildcard=wildcard, style=wx.OPEN)

        if dialog.ShowModal() != wx.ID_OK:
            return None

        openFileName = dialog.GetPath()
        dialog.Destroy()
        return openFileName

    def loadXML(self):
        if len(self.rtc_dict) > 0:
            ret = self.askDialog(strDEL_SYS)
            if ret != wx.ID_OK:
                return

        openFileName = self.openFileDialog()
        print "open file is :",openFileName
        if openFileName == None:
            return

        # delete
        self.deleteAllShape()

        rtxml = RtmParser.RtmParser()
        dict = rtxml.readXML(openFileName)

        canvas = self.diagram.GetCanvas()
        canvas.viewMode = True
        self.createGRtc_from_dict(dict)

    def saveFileDialog(self):
        wildcard = "*.xml |*.xml| *.* |*.*"
        dialog = wx.FileDialog( self, strSAVE_AS_TITLE, defaultDir=os.getcwd(),
                                defaultFile="", wildcard=wildcard, style=wx.SAVE)

        if dialog.ShowModal() != wx.ID_OK:
            return None

        saveFileName = dialog.GetPath()
        dialog.Destroy()
        return saveFileName

    def makeDumpData(self):
        canvas = self.diagram.GetCanvas()

        dict = {}
        dict['rtc'] = {}
        dict['line'] = []
        for fullname in self.rtc_list:
            comp = self.rtc_dict[fullname]
            name = fullname

            x_size, y_size = comp.baseBox.GetBoundingBoxMin()
            if comp.lastRot == 'TB':
                y_size, x_size = x_size, y_size

            x = comp.baseBox.GetX() - x_size/2
            y = comp.baseBox.GetY() - y_size/2
#            print "Comp name:",name," x=",x," y=",y
            dict['rtc'][name] = {}
            dict['rtc'][name]['x'] = x
            dict['rtc'][name]['y'] = y

            if comp.lastRot == 'LR':
                if comp.revTogle == 1:
                    dict['rtc'][name]['rot'] = 'Right'
                else:
                    dict['rtc'][name]['rot'] = 'Left'
            else:
                if comp.rotTogle == 1:
                    dict['rtc'][name]['rot'] = 'Top'
                else:
                    dict['rtc'][name]['rot'] = 'Bottom'
                  
#            comp.rotTogle  # 1 or 0 # 1 :inport's is top side of body
#            comp.revTogle  # 1 or 0 # 1 :inport's is right side of body
#            comp.lastRot   # LR(Left/Right) or TB(Top/Bottom)

        for line_idx in canvas.line:
            out_obj = canvas.line[line_idx].g_outp
            in_obj = canvas.line[line_idx].g_inp
            tmp = {}
#            tmp['type'] =

            tmp['pos'] = canvas.line[line_idx].coordT
            tmp['in-comp'] = in_obj.fullpath
            tmp['in-name'] = in_obj.inport['name']
            tmp['out-comp'] = out_obj.fullpath
            tmp['out-name'] = out_obj.outport['name']
            dict['line'].append(tmp)

        return dict

    def saveXML(self, saveFileName):

        dict = {}
        dict = self.makeDumpData()

        rtxml = RtmParser.RtmParser()

        rtxml.writeXML(saveFileName, dict)

    def saveAsXML(self):
        saveFileName = self.saveFileDialog()
        if saveFileName == None:
            return

        self.saveXML(saveFileName)

    def refresh(self):
        """リフレッシュ処理
        コンポーネント及び、表示図形のリフレッシュを行う

        [引数]
        なし

        [戻り値]
        void
        """
        canvas = self.diagram.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)
        for obj in canvas.selected:
            obj.Select(False, dc)
            obj.parent.unselected(dc)
        canvas.selected = []
        for rtc_name in self.rtc_list:
            self.rtc_dict[rtc_name].refresh()
        for rtc_name in self.rtc_list:
            self.rtc_dict[rtc_name].refresh_outp()

    def reConnect(self):
        """再接続処理
        画面上のコンポーネントの再接続処理を呼び出す

        [引数]
        なし

        [戻り値]
        void
        """
# assembly dummy process
#        return
# assembly dummy process
        for rtc_name in self.rtc_list:
            self.rtc_dict[rtc_name].reConnectLine()

    def remakeLines(self):
# assembly dummy process
#        return
# assembly dummy process
        for rtc_name in self.rtc_list:
            self.rtc_dict[rtc_name].remakeLines()

    def createGRtc_from_dict(self,dict):
        """ディクショナリーからコンポーネント図形を生成

        [引数]
        dict   -- アセンブリのディクショナリー

        [戻り値]
        void
        """
        canvas = self.diagram.GetCanvas()
        dc = wx.ClientDC(canvas)
        canvas.PrepareDC(dc)

        rtc_list = dict['rtc'].keys()
        self.rtc_dict = {}
        new_list = []
        pos_x = 0
        pos_y = 0
        for rtc_name in rtc_list:

            # obj-ref error check
            try:
                ref = self.frame.myDict.GetObjRefToFullpath(rtc_name)
                if ref == None:
                    print 'Component Create error!: %s'%rtc_name
                    continue
                ref = ref._narrow(RTM.RTCBase)
                tmp = self.frame.myDict.GetCompState(rtc_name)
#                if tmp >= RTM.RTComponent.RTC_ABORTING or tmp == 0:
#                    ref = None
            except:
                err_mess = 'Component Create error:%s\n'%rtc_name
                except_mess(err_mess)
                continue

            new_list.append(rtc_name)

            pos_x = dict['rtc'][rtc_name]['x']
            pos_y = dict['rtc'][rtc_name]['y']
            comp = GRtc(self, rtc_name, pos_x, pos_y)
            comp.changeBodyColor('virtual')
            self.rtc_dict[rtc_name] = comp

            if dict['rtc'][rtc_name]['rot'] == 'Left':
                comp.reversesBody()
            elif dict['rtc'][rtc_name]['rot'] == 'Top':
                comp.rotatesBody()
            elif dict['rtc'][rtc_name]['rot'] == 'Bottom':
                comp.rotatesBody()
                comp.rotatesBody()

            comp.refresh()

        if len(new_list) == 0:
            canvas.viewMode = False
            return

        self.rtc_list = new_list

        for line_num in range(len(dict['line'])):
            line_dict = dict['line'][line_num]
            out_comp_name = line_dict['out-comp']
            out_name = line_dict['out-name']

            in_comp_name = line_dict['in-comp']
            in_name = line_dict['in-name']

            #error check
            if not self.rtc_dict.has_key(out_comp_name):
                print 'Assembly Check: Port Connection Error!:',out_comp_name
                continue
            if not self.rtc_dict[out_comp_name].out_dict.has_key(out_name):
                print 'Assembly Check: Port Connection Error!: ',out_comp_name,":",out_name
                continue
            if not self.rtc_dict.has_key(in_comp_name):
                print 'Assembly Check: Port Connection Error!:',in_comp_name
                continue
            if not self.rtc_dict[in_comp_name].in_dict.has_key(in_name):
                print 'Assembly Check: Port Connection Error!: ',in_comp_name,":",in_name
                continue

            outp_obj = self.rtc_dict[out_comp_name].out_dict[out_name]
            inp_obj = self.rtc_dict[in_comp_name].in_dict[in_name]

            canvas.lineFrom = outp_obj.body
            canvas.lineTo = inp_obj.body
            line = GRtcLine(canvas,outp_obj)
            line.setLine2port(canvas, dc)
            line.g_outp.connect(line.idx, line.g_outp.subscription_type)

            pos = []
            pos_list = []
            pos_list = string.splitfields(line_dict['pos'],',')
            for num in range(len(pos_list)/2):
                pos.append((string.atof(pos_list[num*2]),string.atof(pos_list[num*2+1])))

            #line.move_child
            line.childMove(dc, pos)

            canvas.lineFrom = None
            canvas.lineTo = None
        canvas.Redraw(dc)

    def MyAddBmp(self, shape, x, y, pen):
        """ビットマップ図形の登録
        キャンバス、ダイアグラム、イベントとの関連付け

        [引数]
        shape   -- ビットマップ図形を指定
        x       -- 表示するｘ座標
        y       -- 表示するｙ座標
        pen     -- ペン（色、太さ）を指定

        [戻り値]
        shape -- wx.Shape 図形オブジェクト
        """
        shape.SetDraggable(False, False)
        shape.SetCanvas(self)
        if pen:    shape.SetPen(pen)
        shape.SetX(x)
        shape.SetY(y)
        self.diagram.AddShape(shape)
        evthandler = MyEvtHandlerBmp(self.log, self.frame)
        evthandler.SetShape(shape)
        evthandler.SetPreviousHandler(shape.GetEventHandler())
        shape.SetEventHandler(evthandler)

        shape.Show(True)
        return shape

    def MyAddOval(self, shape, x, y):
        """円図形の登録
        キャンバス、ダイアグラム、イベントとの関連付け

        [引数]
        shape   -- ビットマップ図形を指定
        x       -- 表示するｘ座標
        y       -- 表示するｙ座標

        [戻り値]
        shape -- wx.Shape 図形オブジェクト
        """
        shape.SetDraggable(True, False)
        shape.SetCanvas(self)
        shape.SetX(x)
        shape.SetY(y)
        self.diagram.AddShape(shape)
        evthandler = MyEvtHandlerOval(self.log, self.frame)
        evthandler.SetShape(shape)
        evthandler.SetPreviousHandler(shape.GetEventHandler())
        shape.SetEventHandler(evthandler)

        shape.Show(True)
        return shape

    def MyAddText(self, shape, x, y, pen, brush=None ):
        """テキストの登録
        キャンバス、ダイアグラム、イベントとの関連付け

        [引数]
        shape   -- ビットマップ図形を指定
        x       -- 表示するｘ座標
        y       -- 表示するｙ座標
        pen     -- ペン（色、太さ）を指定
        burush  -- ブラシ（色）を指定

        [戻り値]
        shape -- wx.Shape 図形オブジェクト
        """
        shape.SetDraggable(False, False)
        shape.SetCanvas(self)
        shape.SetX(x)
        shape.SetY(y)
        if pen:    shape.SetPen(pen)
        if brush:  shape.SetBrush(brush)
        self.diagram.AddShape(shape)
        evthandler = MyEvtHandlerDummy(self.log, self.frame)
        evthandler.SetShape(shape)
        evthandler.SetPreviousHandler(shape.GetEventHandler())
        shape.SetEventHandler(evthandler)

        shape.Show(True)
        return shape

    def MyAddShape(self, shape, x, y, pen, brush, text, inoutPort=0):
        """コンポーネント図形、ポート図形の登録
        キャンバス、ダイアグラム、イベントとの関連付け

        [引数]
        shape   -- ビットマップ図形を指定
        x       -- 表示するｘ座標
        y       -- 表示するｙ座標
        pen     -- ペン（色、太さ）を指定
        burush  -- ブラシ（色）を指定
        text    -- 図形に表示する文字列を指定
        inoutPort -- ポート指定フラグ
                     0:コンポーネント本体 / 1:Inpot,Outport

        [戻り値]
        shape -- wx.Shape 図形オブジェクト
        """
        shape.SetDraggable(True, True)
        shape.SetCanvas(self)
        shape.SetX(x)
        shape.SetY(y)
        if pen:    shape.SetPen(pen)
        if brush:  shape.SetBrush(brush)
        if text:   shape.AddText(text)
        self.diagram.AddShape(shape)
        shape.Show(True)

        if inoutPort == 1:   # make in/out port event handler
            evthandler = MyPortEvtHandler(self.log, self.frame)
            evthandler.SetShape(shape)
            evthandler.SetPreviousHandler(shape.GetEventHandler())
            shape.SetEventHandler(evthandler)
        elif inoutPort == 0:                # make body event handler
            evthandler = MyEvtHandler(self.log, self.frame)
            evthandler.SetShape(shape)
            evthandler.SetPreviousHandler(shape.GetEventHandler())
            shape.SetEventHandler(evthandler)

        return shape

    def OnDestroy(self, evt):
        """ウィンドウ削除イベントハンドラ

        [引数]
        evt     -- イベント

        [戻り値]
        void
        """
        # Do some cleanup
        for shape in self.diagram.GetShapeList():
            if shape.GetParent() == None:
                shape.SetCanvas(None)
                shape.Destroy()

        self.diagram.Destroy()

    def deleteShape(self,obj):
        """図形を削除する

        [引数]
        削除を行うShapeオブジェクト

        [戻り値]
        void
        """
        canvas = self.diagram.GetCanvas()
        dc = wx.ClientDC(self)
        canvas.PrepareDC(dc)
        obj.parent.removeWidget(dc)

        if hasattr(obj, "parent") and obj.parent.tag == 'body':
            rtc_name = obj.parent.fullpath
            if rtc_name in self.rtc_list:
                tmp = self.rtc_list.index(rtc_name)
                del self.rtc_list[tmp]
                del self.rtc_dict[rtc_name]
        if hasattr(obj, "parent") and obj.parent.tag == 'line':
            idx = obj.parent.idx
            if idx in canvas.line.keys():
                del canvas.line[idx]

        del obj

    def deleteAllShape(self):
        """すべての図形を削除する

        [引数]
        なし

        [戻り値]
        void
        """
        canvas = self.diagram.GetCanvas()
        dc = wx.ClientDC(self)
        canvas.PrepareDC(dc)

        for s in canvas.selected:
            s.Select(False, dc)
            s.parent.unselected(dc)

        shapeList = canvas.GetDiagram().GetShapeList()
        for obj in shapeList:
            self.deleteShape(obj)

        canvas.selected = []
        canvas.line = {}

        canvas.Redraw(dc)

    def deleteSelectedShape(self):
        """選択中の図形を削除する

        [引数]
        なし

        [戻り値]
        void
        """
        canvas = self.diagram.GetCanvas()
        dc = wx.ClientDC(self)
        canvas.PrepareDC(dc)
        for obj in canvas.selected:
            self.deleteShape(obj)

        canvas.selected = []
        bdc = getBufferedDC(canvas)
        canvas.PrepareDC(dc)
        canvas.Redraw(dc)


    def OnKeyDown(self, evt):
        """キー押下時のイベントハンドラ
        選択中の図形を削除する

        [引数]
        evt     -- イベント

        [戻り値]
        void
        """
        evtKey = evt.GetKeyCode()
        # DELETE キーの時、選択されている図形を削除
        if evtKey == wx.WXK_DELETE:
            self.deleteSelectedShape()

        canvas = self.diagram.GetCanvas()
        dc = wx.ClientDC(self)
        canvas.PrepareDC(dc)
        bdc = getBufferedDC(canvas)
        canvas.PrepareDC(bdc)
        canvas.Redraw(bdc)

    def OnPopupStart(self, evt):
        """コンポーネント本体のスタートを行うイベントハンドラ

        [引数]
        evt    --  イベント(wx.CommandEvent)

        [戻り値]
        void
        """
        pt = self.pt
        tmpShape = self.FindShape(pt[0],pt[1])
        if tmpShape != 0:
            tmpShape[0].parent.ref_start()

    def OnPopupStop(self, evt):
        """コンポーネント本体のストップを行うイベントハンドラ

        [引数]
        evt    --  イベント(wx.CommandEvent)

        [戻り値]
        void
        """
        pt = self.pt
        tmpShape = self.FindShape(pt[0],pt[1])
        if tmpShape != 0:
            tmpShape[0].parent.ref_stop()

    def OnPopupReset(self, evt):
        pt = self.pt
        tmpShape = self.FindShape(pt[0],pt[1])
        if tmpShape != 0:
            tmpShape[0].parent.ref_reset()

    def OnPopupExit(self, evt):
        pt = self.pt
        tmpShape = self.FindShape(pt[0],pt[1])
        if tmpShape != 0:
            tmpShape[0].parent.ref_exit()

    def OnPopupKill(self, evt):
        pt = self.pt
        tmpShape = self.FindShape(pt[0],pt[1])
        if tmpShape != 0:
            tmpShape[0].parent.ref_kill()

    def OnPopupDelete(self, evt):
        """コンポーネント本体の削除を行うイベントハンドラ

        [引数]
        evt    --  イベント(wx.CommandEvent)

        [戻り値]
        void
        """
        canvas = self.diagram.GetCanvas()
        dc = wx.ClientDC(self)
        canvas.PrepareDC(dc)
        pt = self.pt
        tmpShape = self.FindShape(pt[0],pt[1])
        if tmpShape != 0:
            tmpShape[0].parent.removeWidget(dc)

#            rtc_name = tmpShape[0].parent.rtc.ref_key
            rtc_name = tmpShape[0].parent.fullpath
            del self.rtc_dict[rtc_name]
            tmp = self.rtc_list.index(rtc_name)
            del self.rtc_list[tmp]

            for obj in canvas.selected:
                if obj == tmpShape[0]:
                    tmp = canvas.selected.index(tmpShape[0])
                    del canvas.selected[tmp]
        bdc = getBufferedDC(canvas)
        canvas.PrepareDC(bdc)
#        canvas.Redraw(bdc)
        canvas.Redraw(dc)

    def OnPopupSub(self, evt):
        """サブメニューの実験用ダミーイベントハンドラ

        [引数]
        evt     --  イベント(wx.CommandEvent)

        [戻り値]
        void
        """
        print "OnPopupSub!!!"

    def OnPopupDelSelectItem(self, evt):
        """選択されたアイテムの削除を行うイベントハンドラ

        [引数]
        evt    --  イベント(wx.CommandEvent)

        [戻り値]
        void
        """
        self.deleteSelectedShape()

    def OnPopupLoadXML(self, evt):
        """XMLファイル(comp_data.xml)を読み込むイベントハンドラ

        [引数]
        evt    --  イベント(wx.CommandEvent)

        [戻り値]
        void
        """
        self.loadXML()

    def OnPopupSaveDefaultXML(self, evt):
        """XMLファイル(アセンブリ)を書き込むイベントハンドラ
        上書き保存
        ファイル名:System?.xml  ・・・ ? には画面番号が入る

        [引数]
        evt    --  イベント(wx.CommandEvent)

        [戻り値]
        void
        """
        filename = "System%d.xml" % self.frame.drawCurNum
        self.saveXML(filename)

    def OnPopupSaveXML(self, evt):
        """XMLファイル(アセンブリ)を書き込むイベントハンドラ
        ファイル名変更で保存
        ファイル名は、ダイアログでユーザ任意指定

        [引数]
        evt    --  イベント(wx.CommandEvent)

        [戻り値]
        void
        """
        self.saveAsXML()

    def OnPopupRefresh(self, evt):
        """Refresh処理を行うイベントハンドラ
        古い接続(コンポーネント上にだけsubuscribe情報がある。画面に線が表示されていない状態)が
        あるときに、ok/cancelダイアログを表示し、再接続と最新のステータスで色を変更する

        [引数]
        evt    --  イベント(wx.CommandEvent)

        [戻り値]
        void
        """
        canvas = self.diagram.GetCanvas()
        # yes/no dialog
        val = self.DelOldConnectAskDialog()
        if val != wx.ID_OK:
            return

        self.refresh()
        self.reConnect()

    def checkOtherConnect(self):
        ret = False
        for rtc_name in self.rtc_list:
            ret = self.rtc_dict[rtc_name].checkOtherConnect()
            if ret == True:
                break
        return ret

    def askDialog(self, str):
        """ダイアログの表示機能
        ok/cancel ダイアログを表示する

        [引数]
        str  ---  ダイアログに表示するメッセージ

        [戻り値]
        val  ---  ダイアログの戻り値(wx.ID_OK/wx.ID_CANCEL)
        void
        """

        # yes/no dialog
        val = wx.ID_OK
        dlg = RtmDialog(self, -1, str)
        dlg.CenterOnParent()
        val = dlg.ShowModal()
        dlg.Destroy()

        return val

    def DelOldConnectAskDialog(self):
        """ダイアログの表示機能
        古い接続(コンポーネント上にsubscribe情報があり画面上には線が表示されていない)があれば、
        ok/cancel ダイアログを表示する

        [引数]
        なし

        [戻り値]
        val  ---  ダイアログの戻り値(wx.ID_OK/wx.ID_CANCEL)
                  古い情報がない時は、wx.ID_OKを返す
        void
        """
# assembly dummy
#        return wx.ID_OK
# assembly dummy
        # yes/no dialog
        val = wx.ID_OK
        connect_flag = self.checkOtherConnect()
        if connect_flag == True:
            val = askDialog(strASKMESSAGE)
        return val

    def OnPopupConnectView(self, evt):
        """Connectメニューの処理を行うイベントハンドラ
        アセンブリファイル読み込み後の接続（subscribe）処理

        [引数]
        evt    --  イベント(wx.CommandEvent)

        [戻り値]
        void
        """
        canvas = self.diagram.GetCanvas()
        # yes/no dialog
        val = self.DelOldConnectAskDialog()
        if val != wx.ID_OK:
            return

        canvas.viewMode = False
        self.refresh()
        self.reConnect()

    def OnPopupDeleteView(self, evt):
        """Deleteメニューの処理を行うイベントハンドラ
        アセンブリファイル読み込み後の表示画像の全削除処理

        [引数]
        evt    --  イベント(wx.CommandEvent)

        [戻り値]
        void
        """
        self.deleteAllShape()
        canvas = self.diagram.GetCanvas()
        canvas.viewMode = False

    def makeBodyPopupMenu(self, evt):
        """コンポーネント上のポップアップメニュー（コンテキストメニュー）作成
        コンポーネント図形本体のOn/Off、削除をメニュー表示

        [引数]
        evt     --  イベント(wx.MouseEvent)を指定

        [戻り値]
        void
        """
        # メニュー関連のイベントはCommandEventで、マウスカーソル座標を取得出来ない
        # ここでは、座標をself.ptに格納し直後のメニューイベントで使用する
        self.pt = evt.GetPosition()
        if not hasattr(self, "popupID1"):
            self.popupID1 = wx.NewId()         # start menu
            self.popupID2 = wx.NewId()         # stop menu
            self.popupID3 = wx.NewId()         # delete menu
            self.popupID4 = wx.NewId()         # reset menu
            self.popupID5 = wx.NewId()         # exit menu
            self.popupID6 = wx.NewId()         # kill menu

            self.Bind(wx.EVT_MENU, self.OnPopupStart, id = self.popupID1)
            self.Bind(wx.EVT_MENU, self.OnPopupStop, id = self.popupID2)
            self.Bind(wx.EVT_MENU, self.OnPopupDelete, id = self.popupID3)
            self.Bind(wx.EVT_MENU, self.OnPopupReset, id = self.popupID4)
            self.Bind(wx.EVT_MENU, self.OnPopupExit, id = self.popupID5)
            self.Bind(wx.EVT_MENU, self.OnPopupKill, id = self.popupID6)


        menu = wx.Menu()
        menu.Append(self.popupID1, strSTART)
        menu.Append(self.popupID2, strSTOP)
        menu.Append(self.popupID4, strRESET)
        menu.Append(self.popupID5, strEXIT)
        menu.Append(self.popupID6, strKILL)
        menu.AppendSeparator()
        menu.Append(self.popupID3, strDELITEM)

        self.PopupMenu(menu, evt.GetPosition())
        menu.Destroy()

    def makeBackgroundPopupMenu(self, evt ):
        """バックグランド上のポップアップメニュー（コンテキストメニュー）作成
        削除、ロード、セーブのメニュー表示

        [引数]
        evt     --  イベント(wx.MouseEvent)を指定

        [戻り値]
        void
        """
        # メニュー関連のイベントはCommandEventで、マウスカーソル座標を取得出来ない
        # ここでは、座標をself.ptに格納し直後のメニューイベントで使用する
        self.pt = evt.GetPosition()
        if not hasattr(self, "popupIDa"):
            self.popupIDa = wx.NewId()         # selected item delete menu
            self.popupIDb = wx.NewId()         # refresh menu
            self.popupIDc = wx.NewId()         # xml file load menu
            self.popupIDd = wx.NewId()         # xml file save menu
            self.popupIDe = wx.NewId()         # xml file save as menu

            self.Bind(wx.EVT_MENU, self.OnPopupDelSelectItem,id = self.popupIDa)
            self.Bind(wx.EVT_MENU, self.OnPopupRefresh, id = self.popupIDb)
            self.Bind(wx.EVT_MENU, self.OnPopupLoadXML, id = self.popupIDc)
            self.Bind(wx.EVT_MENU, self.OnPopupSaveDefaultXML, id = self.popupIDd)
            self.Bind(wx.EVT_MENU, self.OnPopupSaveXML, id = self.popupIDe)

        menu = wx.Menu()
        menu.Append(self.popupIDa, strDEL_SELECT)
        menu.AppendSeparator()
        menu.Append(self.popupIDb, strREFRESH)
        menu.AppendSeparator()
        menu.Append(self.popupIDc, strOPEN) 
        menu.Append(self.popupIDd, strSAVE) 
        menu.Append(self.popupIDe, strSAVE_AS) 
# assembly disable
#        menu.FindItemById(self.popupIDc).Enable(False)
#        menu.FindItemById(self.popupIDd).Enable(False)
#        menu.FindItemById(self.popupIDe).Enable(False)
# assembly disable


        self.PopupMenu(menu, evt.GetPosition())
        menu.Destroy()

    def makeViewModePopupMenu(self, evt ):
        """バックグランド上のポップアップメニュー（コンテキストメニュー）作成
        Connect、Delete のメニュー表示

        [引数]
        evt     --  イベント(wx.MouseEvent)を指定

        [戻り値]
        void
        """
        # メニュー関連のイベントはCommandEventで、マウスカーソル座標を取得出来ない
        # ここでは、座標をself.ptに格納し直後のメニューイベントで使用する
        self.pt = evt.GetPosition()
        if not hasattr(self, "popupID01"):
            self.popupID01 = wx.NewId()         # selected item delete menu
            self.popupID02 = wx.NewId()         # xml file load menu

            self.Bind(wx.EVT_MENU, self.OnPopupConnectView,id = self.popupID01)
            self.Bind(wx.EVT_MENU, self.OnPopupDeleteView, id = self.popupID02)

        menu = wx.Menu()
        menu.Append(self.popupID01, strASM_CONNECT)
        menu.AppendSeparator()
        menu.Append(self.popupID02, strASM_DELETE)

        self.PopupMenu(menu, evt.GetPosition())
        menu.Destroy()

    def OnRightDown(self, evt):
        """右クリック・ダウンのイベントハンドラ
        マウスカーソルがInport/Outport上だと、ツールチップ（バルーンヘルプ）の表示を行う
        コンポーネント本体上では、ポップアップメニューの表示を行う

        [引数]
        evt     -- イベント

        [戻り値]
        void
        """
        self.log.write("OnRightDown")
        canvas = self.diagram.GetCanvas()
        dc = wx.ClientDC(self)
        canvas.PrepareDC(dc)
        pt = evt.GetPosition()
        tmpShape = self.FindShape(pt[0],pt[1])
        if tmpShape != 0:
            if hasattr(tmpShape[0], 'parent'):
                # Inport/Outportの処理
                if tmpShape[0].parent.tag == 'in' or tmpShape[0].parent.tag == 'out':
                    # ツールチップの作成
                    self.tooltip = makeToolTip(tmpShape[0].parent,pt,dc)

                    self.tooltip.body.SetCanvas(canvas)
                    self.diagram.AddShape(self.tooltip.body)
                    self.tooltip.body.Show(True)
                # コンポーネント本体の処理
                elif tmpShape[0].parent.tag == 'body' and canvas.viewMode == False:
                    self.makeBodyPopupMenu(evt)
        else:
            if canvas.viewMode == False:
                self.makeBackgroundPopupMenu(evt)
            else:
                self.makeViewModePopupMenu(evt)

        canvas.Redraw(dc)
        evt.Skip()

    def OnRightUp(self, evt):
        """右クリック・アップのイベントハンドラ
        ツールチップ（バルーンヘルプ）の削除を行う

        [引数]
        evt     -- イベント

        [戻り値]
        void
        """
        self.log.write("OnRightUp")
        canvas = self.diagram.GetCanvas()
        dc = wx.ClientDC(self)
        canvas.PrepareDC(dc)
        pt = evt.GetPosition()
        if self.tooltip != None:
            # ツールチップの削除
            self.tooltip.removeWidget(dc)
            del self.tooltip
            self.tooltip = None
        canvas.Redraw(dc)
        evt.Skip()

    def OnLeftDown(self, evt):
        """左クリック・ダウンのイベントハンドラ
        キャンバス上の図形がない空間で左クリックされたら図形選択を解除する

        [引数]
        evt     -- イベント

        [戻り値]
        void
        """
        pt = evt.GetPosition()
        tmpShape = self.FindShape(pt[0],pt[1])
        if tmpShape == 0:
            canvas = self.diagram.GetCanvas()
            dc = wx.ClientDC(self)
            canvas.PrepareDC(dc)
            for s in canvas.selected:
                s.Select(False, dc)
                s.parent.unselected(dc)
            canvas.selected = []
            canvas.Redraw(dc)
        else:
            evt.Skip()

    def OnMiddleDown(self, evt):
        """マウス中ボタン・ダウンのイベントハンドラ
        コンポーネント図形の本体上で押されたら、回転処理を行う

        [引数]
        evt   -- イベント

        [戻り値]
        void
        """
        pt = evt.GetPosition()
        isShift = evt.ShiftDown()
        canvas = self.diagram.GetCanvas()
        dc = wx.ClientDC(self)
        canvas.PrepareDC(dc)
        for s in canvas.selected:
            s.Select(False, dc)
            s.parent.unselected(dc)
        canvas.selected = []
        canvas.Redraw(dc)

        tmpShape = self.FindShape(pt[0],pt[1])
        if tmpShape != 0 and tmpShape[0].parent.tag == 'body' and isShift != True:  ## Middle Button
            self.log.write("OnMiddleDown: " )
            tmpShape[0].parent.reversesBody()
        elif tmpShape != 0 and tmpShape[0].parent.tag == 'body' and isShift == True:  ## Shift Key Down & Middle Button
            self.log.write("OnMiddleDown + Shift Key: " )
            tmpShape[0].parent.rotatesBody()
        else:
            evt.Skip()

        canvas.Redraw(dc)
        
#----------------------------------------------------------------------

def runTest(frame, nb, log):
    # This creates some pens and brushes that the OGL library uses.
    # It should be called after the app object has been created, but
    # before OGL is used.
    ogl.OGLInitialize()
    wx.lib.colourdb.updateColourDB()
    win = TestWindow(nb, log, frame)

    return win
    
#----------------------------------------------------------------------

# The OGL library holds some resources that need to be freed before
# the app shuts down.
class __Cleanup:
    def __del__(self, cleanup=ogl.OGLCleanUp):
        cleanup()

# When this module gets cleaned up by Python then __cu will be cleaned
# up and it's __dell__ is called, which will then call ogl.OGLCleanUp.
__cu = __Cleanup()


overview = """\
The Object Graphics Library is a library supporting the creation and
manipulation of simple and complex graphic images on a canvas.

"""

if __name__ == '__main__':
    import sys,os
    import run
    run.main(['', os.path.basename(sys.argv[0])] + sys.argv[1:])

