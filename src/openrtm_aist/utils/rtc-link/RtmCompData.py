#!/usr/bin/env python
# -*- Python -*-
#
#  @file RtmCompData.py
#  @brief rtc-link component dict manager class
#  @date $Date: 2007-01-21 13:20:47 $
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

#
# RtmCompData.py             Created on: 2004/11/02
#                            Author    : Tsuyoshi Tanabe
"""
    GUI用ディクショナリの管理クラス
"""
#
# ディクショナリで管理するのは、fullpathに対するオブジェクトリファレンスのみに変更
#

# dictionary format
# {'name-service name':{'fullpath':obj-ref}}



import sys,os
import string
import mutex
from omniORB import CORBA
#import RtmParser as rtxml

# RTM のimportは、PYTHONPATHにIDLスタブのあるディレクトリを追加する事。
import RTC
from RTM import *
from RtmDialog import *

NAME_PATH_NUM = 6


#----------------------------------------------------------------------
class RtmDictCore:
    """ディクショナリ管理クラス"""
    def __init__(self, parent):
        """クラスの初期化

        [引数]
        parent -- 親クラス

        [戻り値]
        void
        """
        self.parent = parent
        self.dict = {}
        self.default_ns = 'localhost'
        default_dict = {}

    def __del__(self):
        pass


    def SplitTokenToFullpath(self, fullname):
        """フルパス名称のトークン分割

        [引数]
        fullname -- '/'区切りのフルパス
                    ５階層以上の文字列はエラー

        [戻り値]
        key_list -- トークンのlist
        """
        key_list = string.splitfields(fullname,'/')
        if len(key_list) > NAME_PATH_NUM:
            print "Error: SplitTokenToFullpath: fullpath is too long(please set to 5 or less)"
            key_list = None
        return key_list


    def ClearDictionary(self):
        """ディクショナリの削除

        [引数]
        void

        [戻り値]
        void 
        """
        del self.dict
        self.dict = None

    def GetInPortToRefCallback(self, fullpath):
        """ディクショナリのオブジェクトリファレンスからInPortを取得

        [引数]
        fullpath     -- フルパス指定

        [戻り値]
        inport_list  -- インポートリスト
        """
#        print "                     , ref :",ref
        self.retlist = []
        try:
            ref = self.dict[fullpath]['objref']
            ref = ref._narrow(RTC.RTObject)

            in_list = []
            ret_list = []
            in_list = ref._get_inports()
            tmp = {}
            n = 0
            for inp in in_list:
                inp = inp._narrow(InPort)
                tmp = {}
                tmp['ref'] = inp
                prof = inp._get_profile()
                tmp['name'] = prof.name
                tmp['port_type'] = prof.port_type.name()

#                tmp['name'] = "in:test-dummy" + '%d'% n
#                tmp['name'] = "in" + '%d'% n
#                tmp['port_type'] = "RTM::TimedFloat:test-dummy"
#                tmp['port_type'] = "RTM::TimedFloat"
                ret_list.append(tmp)
                n=n+1
        except:
            except_mess("GetObjRefToInPort error!:")
            ret_list = []

        self.retlist = ret_list
        return ret_list

    def GetOutPortToRefCallback(self, fullpath):
        """ディクショナリのオブジェクトリファレンスからOutPortを取得

        [引数]
        fullpath  -- フルパス指定

        [戻り値]
        outport_list -- アウトポートリスト
        """
        self.retlist = []
        try:
            ref = self.dict[fullpath]['objref']
            ref = ref._narrow(RTC.RTObject)

            out_list = []
            out_list = ref._get_outports()
            ret_list = []
            tmp = {}
            n=0
            for outp in out_list:
                outp = outp._narrow(OutPort)
                tmp = {}
                tmp['ref'] = outp
                prof = outp._get_profile()
                tmp['name'] = prof.name
                tmp['port_type'] = prof.port_type.name()

#                tmp['name'] = "out:test-dummy" + '%d'% n
#                tmp['port_type'] = "TimedFloat:test-dummy"
                ret_list.append(tmp)
                n=n+1
        except:
            except_mess("GetObjRefToOutPort error!:")
            ret_list = []

        self.retlist = ret_list
        return ret_list


    def SetObjRefToFullpath(self, position, comp):
        """ディクショナリにオブジェクトリファレンスを追加

        [引数]
        position -- フルパス指定
        comp      -- objref,id,kind のディクショナリ

        [戻り値]
        void
        """
        if comp == None:
            ref = None
            comp = {}
            comp['id'] = None
            comp['kind'] = 'ns'
            comp['bname'] = None
        else:
            ref = comp['objref']
        if comp['kind'] == 'rtc':
            if not self.dict.has_key(position):
#                print 'check duplicate !!:',comp['objref']
                try:
                    ref = ref._duplicate(ref)
                except:
                    err_mess = 'duplicate exception!!:%s\n' % comp['objref']
                    except_mess(err_mess)
            try:
                ref = ref._narrow(RTC.RTObject)
            except:
                except_mess('SetObjRefToFullPath error!:')
                ref = None
        if comp['kind'] == 'mgr':
            if not self.dict.has_key(position):
#                print 'check duplicate !!:',comp['objref']
                try:
                    ref = ref._duplicate(ref)
                except:
                    ref = None
                    err_mess = 'duplicate exception!!:%s\n' % comp['objref']
                    except_mess(err_mess)
#            try:
#                ref = ref._narrow(RTCManager)
#            except:
#                print 'SetObjRefToFullPath: narrow error!'
#                ref = None

        self.dict[position] = {}
        self.dict[position]['objref'] = ref
        self.dict[position]['id'] = comp['id']
        self.dict[position]['kind'] = comp['kind']
        self.dict[position]['bname'] = comp['bname']


    def GetObjRefToFullpathCallback(self, position):
        """ディクショナリからオブジェクトリファレンスを取得

        [引数]
        position -- フルパス指定

        [戻り値]
        ref      -- オブジェクトリファレンス
        """

        self.ret = None
        if self.dict.has_key(position):
            ref = self.dict[position]['objref']
        else:
            print "Error: GetObjRefToFullpath: fullpath error!"
            ref = None

        self.ret = ref
        return ref

    def GetKindToFullpathCallback(self, position):
        """ディクショナリからKindを取得

        [引数]
        position -- フルパス指定

        [戻り値]
        kind      -- Kind
        """
        kind_list = ['ns','host_cxt','mgr_cxt','mgr','cate_cxt','mod_cxt','rtc']

        self.ret = None
        if self.dict.has_key(position):
            kind = self.dict[position]['kind']
        else:
            print "Error: GetKindToFullpath: fullpath error!"
            kind = 'unknown'

        if kind not in kind_list:
            kind = 'unknown'

        self.ret = kind
        return kind

    def GetIdToFullpathCallback(self, position):
        """ディクショナリからIdを取得

        [引数]
        position -- フルパス指定

        [戻り値]
        Id      -- Id
        """

        self.ret = None
        if self.dict.has_key(position):
            Id = self.dict[position]['id']
        else:
            print "Error: GetIdToFullpath: fullpath error!"
            Id = None

        self.ret = Id
        return Id

    def GetBindingNameToFullpath(self, position):
        """ディクショナリからBinding_nameを取得

        [引数]
        position -- フルパス指定

        [戻り値]
        bname      -- binding name
        """

        self.ret = None
        if self.dict.has_key(position):
            bname = self.dict[position]['bname']
        else:
            print "Error: GetBindingNameToFullpath: fullpath error!"
            bname = None

        self.ret = bname
        return bname


#----------------------------------------------------------------------
class RtmCompData(RtmDictCore):
    """ディクショナリのアクセス・クラス（ラッパー？）
    ここに、ディクショナリを使用するラッパーメソッドを記述する
    """
    def __init__(self, parent):
        """クラスの初期化


        [引数]
        parent -- 親クラス

        [戻り値]
        void
        """
        RtmDictCore.__init__(self,parent)
        self.parent = parent
        self.Mutex = mutex.mutex()

    def GetCompStateCallback(self,fullpath):
        self.ret2 = 0
        tmp = 0
        try:
            ref = self.GetObjRefToFullpathCallback(fullpath)
            ref = ref._narrow(RTC.RTObject)
        except :
            err_mess = 'GetCompState: obj-ref error:%s\n' % fullpath
            except_mess(err_mess)
            tmp = 0
            self.ret2 = 0
            return
        try:
            tmp_port = ref._get_rtc_state()
            tmp_port = tmp_port._narrow(OutPort)
            tmp = tmp_port.get()
            tmp = tmp.value()
            tmp = tmp.data
            print "refresh state:",tmp
        except :
            err_mess =  'GetCompState: rtc_state error:%s\n' % fullpath
            except_mess(err_mess)
#            tmp = RTComponent.RTC_ERROR
            tmp = 0

        self.ret2 = tmp
        return tmp

    def GetCompState(self,fullpath):
        self.Mutex.lock(self.GetCompStateCallback,fullpath)
        self.Mutex.unlock()
        return self.ret2

    def GetCompNameCallback(self,fullpath):
        self.ret2 = ''
        key_list = self.SplitTokenToFullpath(fullpath)
        num = len(key_list)

        self.ret2 = key_list[num-1]
        num = self.ret2.find('|')
        self.ret2 = str(self.ret2[0:num])
        return self.ret2

    def GetCompName(self,fullpath):
        self.Mutex.lock(self.GetCompNameCallback,fullpath)
        self.Mutex.unlock()
        return self.ret2

    def GetCateNameCallback(self,fullpath):
        self.ret2 = ''
        key_list = self.SplitTokenToFullpath(fullpath)
        num = len(key_list)

        for cate_str in key_list:
            num = cate_str.find('|')
            if num > 0:
                if str(cate_str[num+1:]) == 'cate_cxt':
                    self.ret2 = str(cate_str[0:num])
        return self.ret2

    def GetCateName(self,fullpath):
        self.Mutex.lock(self.GetCateNameCallback,fullpath)
        self.Mutex.unlock()
        return self.ret2

    def GetObjRefToFullpath(self, position):
        self.Mutex.lock(self.GetObjRefToFullpathCallback,position)
        self.Mutex.unlock()
        return self.ret

    def GetInPortToRef(self, fullpath):
        self.Mutex.lock(self.GetInPortToRefCallback,fullpath)
        self.Mutex.unlock()
        return self.retlist

    def GetOutPortToRef(self, fullpath):
        self.Mutex.lock(self.GetOutPortToRefCallback,fullpath)
        self.Mutex.unlock()
        return self.retlist

    def TreeListCompStartCallback(self,fullpath):
        """TreeList上のコンポーネントをスタート
        ※SystemDraw用のメソッドと統廃合する可能性あり

        [引数]
        fullpath -- フルパス指定

        [戻り値]
        void
        """
        self.ret2 = RTM_OK
        ret = RTM_OK
        ref = self.GetObjRefToFullpathCallback(fullpath)
        try:
            ref = ref._narrow(RTC.RTObject)
            ret = ref.rtc_start()
            if ret != RTM_OK:
                print "Error rtc_start(): ",fullpath
        except:
            except_mess('rtc_start error!:%s\n'%fullpath)
            ret = RTM_ERR

        # すべてのSystemDraw画面の当該コンポーネントの色を変更する
        # parent:RtdFrame , drawWin:MDI子ウィンドウ(RtdSystemDraw)
        for sys_no in self.parent.drawWin.keys():
            self.parent.drawWin[sys_no].changeCompColor(fullpath, 'active')
#            self.parent.drawWin[sys_no].changeCompColor(fullpath)

        self.ret2 = ret
        return ret

    def TreeListCompStart(self,fullpath):
        self.Mutex.lock(self.TreeListCompStartCallback,fullpath)
        self.Mutex.unlock()
        return self.ret2

    def TreeListCompStopCallback(self,fullpath):
        """TreeList上のコンポーネントをストップ
        ※SystemDraw用のメソッドと統廃合する可能性あり

        [引数]
        fullpath -- フルパス指定

        [戻り値]
        void
        """
        self.ret2 = RTM_OK
        ret = RTM_OK
        ref = self.GetObjRefToFullpathCallback(fullpath)
        try:
            ref = ref._narrow(RTC.RTObject)
            ret = ref.rtc_stop()
            if ret != RTM_OK:
                print "Error rtc_stop(): ",fullpath
        except:
            except_mess('rtc_stop error!:%s\n'%fullpath)
            ret = RTM_ERR

        for sys_no in self.parent.drawWin.keys():
            self.parent.drawWin[sys_no].changeCompColor(fullpath, 'inactive')
#            self.parent.drawWin[sys_no].changeCompColor(fullpath)
        self.ret2 = ret
        return ret

    def TreeListCompStop(self,fullpath):
        self.Mutex.lock(self.TreeListCompStopCallback,fullpath)
        self.Mutex.unlock()
        return self.ret2

    def TreeListCompResetCallback(self,fullpath):
        """TreeList上のコンポーネントをリセット
        ※SystemDraw用のメソッドと統廃合する可能性あり

        [引数]
        fullpath -- フルパス指定

        [戻り値]
        void
        """
        self.ret2 = RTM_OK
        ret = RTM_OK
        ref = self.GetObjRefToFullpathCallback(fullpath)
        try:
            ref = ref._narrow(RTC.RTObject)
            ret = ref.rtc_reset()
            if ret != RTM_OK:
                print "Error rtc_reset(): ",fullpath
        except:
            except_mess('rtc_reset error!:%s\n'%fullpath)
            ret = RTM_ERR

        for sys_no in self.parent.drawWin.keys():
            self.parent.drawWin[sys_no].changeCompColor(fullpath, 'inactive')
#            self.parent.drawWin[sys_no].changeCompColor(fullpath)

        self.ret2 = ret
        return ret

    def TreeListCompReset(self,fullpath):
        self.Mutex.lock(self.TreeListCompResetCallback,fullpath)
        self.Mutex.unlock()
        return self.ret2

    def TreeListCompExitCallback(self,fullpath):
        """TreeList上のコンポーネントをExit
        ※SystemDraw用のメソッドと統廃合する可能性あり

        [引数]
        fullpath -- フルパス指定

        [戻り値]
        void
        """
        self.ret2 = RTM_OK
        ret = RTM_OK
        ref = self.GetObjRefToFullpathCallback(fullpath)
        try:
            ref = ref._narrow(RTC.RTObject)
            ret = ref.rtc_exit()
            if ret != RTM_OK:
                print "Error rtc_exit(): ",fullpath
        except:
            except_mess('rtc_exit error!:%s\n'%fullpath)
            ret = RTM_ERR

        for sys_no in self.parent.drawWin.keys():
            self.parent.drawWin[sys_no].changeCompColor(fullpath, 'inactive')
#            self.parent.drawWin[sys_no].changeCompColor(fullpath)

        self.ret2 = ret
        return ret

    def TreeListCompExit(self,fullpath):
        self.Mutex.lock(self.TreeListCompExitCallback,fullpath)
        self.Mutex.unlock()
        return self.ret2

    def TreeListCompKillCallback(self,fullpath):
        """TreeList上のコンポーネントをkill
        ※SystemDraw用のメソッドと統廃合する可能性あり

        [引数]
        fullpath -- フルパス指定

        [戻り値]
        void
        """
        self.ret2 = RTM_OK
        ret = RTM_OK
        ref = self.GetObjRefToFullpathCallback(fullpath)
        try:
            ref = ref._narrow(RTC.RTObject)
            ret = ref.rtc_kill()
            if ret != RTM_OK:
                print "Error rtc_kill(): ",fullpath
        except:
            except_mess('rtc_kill error!:%s\n'%fullpath)
            ret = RTM_ERR

        for sys_no in self.parent.drawWin.keys():
            self.parent.drawWin[sys_no].changeCompColor(fullpath, 'inactive')
#            self.parent.drawWin[sys_no].changeCompColor(fullpath)

        self.ret2 = ret
        return ret

    def TreeListCompKill(self,fullpath):
        self.Mutex.lock(self.TreeListCompKillCallback,fullpath)
        self.Mutex.unlock()
        return self.ret2

    def GetIdToFullpath(self,fullpath):
        self.Mutex.lock(self.GetIdToFullpathCallback,fullpath)
        self.Mutex.unlock()
        return self.ret

    def GetKindToFullpath(self,fullpath):
        self.Mutex.lock(self.GetKindToFullpathCallback,fullpath)
        self.Mutex.unlock()
        return self.ret

    def setCompBodyColor(self,fullpath, state):
        for sys_no in self.parent.drawWin.keys():
            self.parent.drawWin[sys_no].changeCompColor(fullpath,state)
