#!/usr/bin/env python
# -*- coding: euc-jp -*-
#
## ConnectTest.py
##
## コンポーネント接続テスト
##

from rtc_handle import *
from BasicDataType_idl import *
import time
import commands
import SDOPackage
import os
import sys

##--------------------------------------------------------------------
g_test_name = "<< component connection test >>"

##--------------------------------------------------------------------
## コネクタープロファイルデフォルト定義
g_interface_type1 = "corba_cdr"
g_dataflow_type = "push"
g_subscription_type = "flush"
g_push_policy = "NEW"
g_push_rate = "2000"
g_skip_count = "4"
#g_skip_count = "0"

## ポート番号指定 ( get_ports()より )
g_port1 = 0
g_port2 = 1
g_port3 = 2

## データポート TimedFloat
g_name1 = "out"
g_connector_id1 = "001"
g_data_type1 = "TimedFloat"

## データポート TimedFloatSeq
g_name2 = "seqout"
g_connector_id2 = "002"
g_data_type2 = "TimedFloatSeq"

## サービスポート 
g_name3 = "MyService"
g_connector_id3 = "003"
g_interface_type3 = "MyService"

##--------------------------------------------------------------------
## 送受信結果判定関連
g_diff_send_file = "./original-data"
g_diff_recv_file = "./received-data"
g_check_message = g_diff_recv_file + " file not found."
g_test_result_file = "./ResultTest.log"
g_test_case = ""
g_test_cnt = "count"
g_test_ok = "."
g_test_ng = "F"
g_test_ng_message = "  < received-data >"
g_mess_header = "< "
g_mess_footer = " > "
# テスト結果内容
# 例)ケース1、1回目 -> "<<< case1 count1 >>> OK."
# 例)ケース1、2回目 -> "<<< case1 count2 >>> NG detected."

##--------------------------------------------------------------------
## テストケース番号の初期値設定
##  上から連番を振っている
case_no = 0

## ケース毎のテスト回数
loop_count = 1

## 受信側activate_componentから送信側activate_componentまでのスリープ時間(秒数)
sleep_recv_act_time = 1

## activate_componentからdeactivate_componentまでのスリープ時間(秒数)
sleep_act_time = 2

## forループのスリープ時間(秒数)
sleep_for_time = 1

## connectからdisconnectまでのスリープ時間(秒数)
sleep_connect_time =1


## ネームサーバー定義
#env = RtmEnv(sys.argv, ["localhost:2809"])
#list0 = env.name_space["localhost:2809"].list_obj()
#env.name_space['localhost:2809'].rtc_handles.keys()
#ns = env.name_space['localhost:2809']

##
# @if jp
# @brief ポートを接続する。
# @else
# @brief Connects ports. 
# @endif
def connect_ports():

    errorFlag = True

    # データポート1 TimedFloat
    ret,porf = g_out_ports[g_port1].connect(g_conprof1)
    if ret!=RTC.RTC_OK:
        errorFlag = False

    # データポート2 TimedFloatSeq
    ret,porf = g_out_ports[g_port2].connect(g_conprof2)
    if ret!=RTC.RTC_OK:
        errorFlag = False

    # サービスポート MyService
    ret,porf = g_out_ports[g_port3].connect(g_conprof3)
    if ret!=RTC.RTC_OK:
        errorFlag = False
    

    return errorFlag
##
# @if jp
# @brief ポートを切断する。
# @else
# @brief Disconnects ports. 
# @endif
def disconnect_ports():

    errorFlag = True

    ret = g_in_ports[g_port3].disconnect(g_conprof3.connector_id)
    if ret!=RTC.RTC_OK:
        errorFlag = False
    ret = g_in_ports[g_port2].disconnect(g_conprof2.connector_id)
    if ret!=RTC.RTC_OK:
        errorFlag = False
    ret = g_in_ports[g_port1].disconnect(g_conprof1.connector_id)
    if ret!=RTC.RTC_OK:
        errorFlag = False

    return errorFlag

##
# @if jp
# @brief ポートを活性化する。
# @else
# @brief Activate components. 
# @endif
def activate_components(sleep_recv_act_time):
    errorFlag = True

    ret = ec_recv[0].activate_component(g_compo_recv.rtc_ref)
    if ret!=RTC.RTC_OK:
        errorFlag = False

    time.sleep(sleep_recv_act_time)

    ret = ec_send[0].activate_component(g_compo_send.rtc_ref)

    if ret!=RTC.RTC_OK:
        errorFlag = False

    return errorFlag
##
# @if jp
# @brief ポートを非活性化する。
# @else
# @brief Deactivate components. 
# @endif
def deactivate_components():
    errorFlag = True

    ret = ec_send[0].deactivate_component(g_compo_send.rtc_ref)
    if ret!=RTC.RTC_OK:
        errorFlag = False
    ret = ec_recv[0].deactivate_component(g_compo_recv.rtc_ref)
    if ret!=RTC.RTC_OK:
        errorFlag = False

    return errorFlag
##
# @if jp
# @brief AutoTestOut と AutoTestIn を起動させる。
# @else
# @brief AutoTestOut and AutoTestIn are started. 
# @endif
def components_entry():
    global g_compo_send,g_compo_recv
    global ec_send,ec_recv
    global g_out_ports,g_in_ports
    global g_conprof1,g_conprof2,g_conprof3
    global env

    os.system('./AutoTestOutComp >/dev/null 2>&1 &') 
    os.system('./AutoTestInComp >/dev/null 2>&1 &') 

    time.sleep(2)

    #env = RtmEnv(sys.argv, ["localhost:2809"])
    list0 = env.name_space["localhost:2809"].list_obj()
    env.name_space['localhost:2809'].rtc_handles.keys()
    ns = env.name_space['localhost:2809']

    g_compo_send = ns.rtc_handles["AutoTestOut0.rtc"]
    g_compo_recv = ns.rtc_handles["AutoTestIn0.rtc"]

    ec_send = g_compo_send.rtc_ref.get_owned_contexts()
    ec_recv = g_compo_recv.rtc_ref.get_owned_contexts()

    g_out_ports = g_compo_send.rtc_ref.get_ports()
    g_in_ports = g_compo_recv.rtc_ref.get_ports()

    g_conprof3 = RTC.ConnectorProfile(g_name3, g_connector_id3, [g_out_ports[g_port3], g_in_ports[g_port3]], [SDOPackage.NameValue("dataport.interface_type",any.to_any(g_interface_type3))])

    return

##
# @if jp
# @brief AutoTestOut と AutoTestIn を終了させる。
# @else
# @brief  AutoTestOut and AutoTestIn are ended. 
# @endif
def components_exit():
    g_compo_send.rtc_ref.exit();
    g_compo_recv.rtc_ref.exit();

    time.sleep(1)
    return
    
##
# @if jp
# @brief グローバル変数の初期化
# @else
# @brief Initialization of global variable
# @endif
def initGlobal():
    global env

    env = RtmEnv(sys.argv, ["localhost:2809"])

    return

##--------------------------------------------------------------------
## 内部関数：コネクタープロファイル設定(データポート)
##
## (引数)
## subscription_type : "flush", "new", "periodic"
## push_policy       : "ALL", "FIFO", "SKIP", "NEW", ""
## connect_direction : 0:outport -> inport, 1:inport -> outport
##--------------------------------------------------------------------
def make_connecter_profile(subscription_type, push_policy, connect_direction):
    global g_conprof1, g_conprof2, g_conprof3

    if connect_direction == 0:
        ## outport -> inport Set
        g_conprof1 = RTC.ConnectorProfile(g_name1, g_connector_id1, [g_out_ports[g_port1], g_in_ports[g_port1]], [SDOPackage.NameValue("dataport.data_type",any.to_any(g_data_type1)),SDOPackage.NameValue("dataport.interface_type",any.to_any(g_interface_type1)),SDOPackage.NameValue("dataport.dataflow_type",any.to_any(g_dataflow_type)),SDOPackage.NameValue("dataport.subscription_type",any.to_any(subscription_type)),SDOPackage.NameValue("dataport.publisher.push_policy",any.to_any(push_policy)),SDOPackage.NameValue("dataport.publisher.push_rate",any.to_any(g_push_rate)),SDOPackage.NameValue("dataport.publisher.skip_count",any.to_any(g_skip_count))])

        g_conprof2 = RTC.ConnectorProfile(g_name2, g_connector_id2, [g_out_ports[g_port2], g_in_ports[g_port2]], [SDOPackage.NameValue("dataport.data_type",any.to_any(g_data_type2)),SDOPackage.NameValue("dataport.interface_type",any.to_any(g_interface_type1)),SDOPackage.NameValue("dataport.dataflow_type",any.to_any(g_dataflow_type)),SDOPackage.NameValue("dataport.subscription_type",any.to_any(subscription_type)),SDOPackage.NameValue("dataport.publisher.push_policy",any.to_any(push_policy)),SDOPackage.NameValue("dataport.publisher.push_rate",any.to_any(g_push_rate)),SDOPackage.NameValue("dataport.publisher.skip_count",any.to_any(g_skip_count))])

        #print "outport -> inport set >>>"
        #print "g_conprof1=",g_conprof1
        #print "g_conprof2=",g_conprof2
    else:
        ## inport -> outport Set
        g_conprof1 = RTC.ConnectorProfile(g_name1, g_connector_id1, [g_in_ports[g_port1], g_out_ports[g_port1]], [SDOPackage.NameValue("dataport.data_type",any.to_any(g_data_type1)),SDOPackage.NameValue("dataport.interface_type",any.to_any(g_interface_type1)),SDOPackage.NameValue("dataport.dataflow_type",any.to_any(g_dataflow_type)),SDOPackage.NameValue("dataport.subscription_type",any.to_any(subscription_type)),SDOPackage.NameValue("dataport.publisher.push_policy",any.to_any(push_policy)),SDOPackage.NameValue("dataport.publisher.push_rate",any.to_any(g_push_rate)),SDOPackage.NameValue("dataport.publisher.skip_count",any.to_any(g_skip_count))])

        g_conprof2 = RTC.ConnectorProfile(g_name2, g_connector_id2, [g_in_ports[g_port2], g_out_ports[g_port2]], [SDOPackage.NameValue("dataport.data_type",any.to_any(g_data_type2)),SDOPackage.NameValue("dataport.interface_type",any.to_any(g_interface_type1)),SDOPackage.NameValue("dataport.dataflow_type",any.to_any(g_dataflow_type)),SDOPackage.NameValue("dataport.subscription_type",any.to_any(subscription_type)),SDOPackage.NameValue("dataport.publisher.push_policy",any.to_any(push_policy)),SDOPackage.NameValue("dataport.publisher.push_rate",any.to_any(g_push_rate)),SDOPackage.NameValue("dataport.publisher.skip_count",any.to_any(g_skip_count))])

        #print "inport -> outport set >>>"
        #print "g_conprof1=",g_conprof1
        #print "g_conprof2=",g_conprof2
    return


##--------------------------------------------------------------------
## 内部関数：受信ファイル削除
##
## (引数)
## なし
##--------------------------------------------------------------------
def delete_recv_file():
    ## ファイルが存在する場合
    if os.path.isfile(g_diff_recv_file) == True:
        os.remove(g_diff_recv_file)
    return


##--------------------------------------------------------------------
## 内部関数：送受信ファイルのデータ比較
##
## (引数)
## なし
## (戻り値)  True : 一致、  False : 不一致
##--------------------------------------------------------------------
def diff_file():
    bret = True

    ## if connect_direction == 0:
    ## else:
    ## 送信ファイル有無判定
    if os.path.isfile(g_diff_send_file) == False:
        print "send_file (%s) not found." % send_file
        return False

    ## 受信ファイル有無判定
    if os.path.isfile(g_diff_recv_file) == False:
        print "recv_file (%s) not found." % recv_file
        return False

    ## 送受信データ差分判定
    f_send = open(g_diff_send_file, 'r')
    f_recv = open(g_diff_recv_file, 'r')

    while(1):
        str_send = f_send.readline()
        str_recv = f_recv.readline()
        if len(str_send) == 0:
            break

        #print "original send date=(%s)" % str_send
        #print ''.join(['%x ' % ord(s) for s in str_send])
        #print "original recv date=(%s)" % str_recv
        #print ''.join(['%x ' % ord(s) for s in str_recv])

        ## 末尾の改行、復帰コード削除
        str_send2 = str_send.rstrip('\n')
        str_send2 = str_send2.rstrip('\r')
        str_recv2 = str_recv.rstrip('\n')
        str_recv2 = str_recv2.rstrip('\r')

        #print "rstrip after send date=(%s)" % str_send2
        #print "rstrip after recv date=(%s)" % str_recv2

        ## データ比較
        list_send = str_send2.split(" ")
        list_recv = str_recv2.split(" ")
        for ic in range(len(list_send)):
            try:
                float_send = float(list_send[ic]);
                float_recv = float(list_recv[ic]);
                if float_send != float_recv:
                    bret = False
                    break;
            except:
                if str_send2 != str_recv2:
                    bret = False
                    break;
         

    f_recv.close()
    f_send.close()
    return bret


##
# @if jp
# @brief 接続と切断だけを繰り返す。
#
#
#
# @else
# @brief Only the connection and cutting are repeated. 
#
#
#
# @endif
def test_connect_disconnect(message,arg0,arg1,arg2):

    errorFlag = True
    for i in range(loop_count):

        ## 1 コネクタープロファイル設定
        make_connecter_profile(arg0,arg1,arg2)

        ## 3 ポート接続
        bool_ret = connect_ports()
        if bool_ret!=True:
            errorFlag = False

        time.sleep(sleep_connect_time)

        ## 6 ポート切断
        bool_ret = disconnect_ports()
        if bool_ret!=True:
            errorFlag = False

        ## テスト結果出力
        fout = open(g_test_result_file, 'a')
        if errorFlag:
            message = g_test_ok
        else :
            message = g_test_ng
        print message ,
        sys.stdout.flush()
        fout.write(message)
        fout.close()

        time.sleep(sleep_for_time)

    return errorFlag

    
##
# @if jp
# @brief ActivateとDeactivateだけを繰り返す。
#
# 接続した状態で Activate と Deactivate を繰り返す。
# データのチェックは行われない。
#
# @else
# @brief Only activation and deactivation are repeated. 
#
# Activate and Deactivate are repeated while connected. 
# Data is not checked. 
#
# @endif
def test_activate_deactivate_2(message,arg0,arg1,arg2):
    
    errorFlag = True
    ## 1 コネクタープロファイル設定
    make_connecter_profile(arg0,arg1,arg2)
    
    ## 3 ポート接続
    bool_ret = connect_ports()
    if bool_ret!=True:
        errorFlag = False
    
    for i in range(loop_count):
    
        ## 2 受信データファイル削除
        delete_recv_file()
    
        ## 4 アクティベート
        bool_ret = activate_components(sleep_recv_act_time)
        if bool_ret!=True:
            errorFlag = False
    
        time.sleep(sleep_act_time)
    
        ## 5 ディアクティベート
        bool_ret = deactivate_components()
        if bool_ret!=True:
            errorFlag = False
    
        ## 受信ファイル有無判定
        if os.path.isfile(g_diff_recv_file) == False:
            errorFlag = False
            fout = open(g_test_result_file, 'a')
            #message = message + g_check_message
            message = g_test_ng
            fout.write(message)
            fout.close()
            print message ,
            sys.stdout.flush()
            time.sleep(sleep_for_time)
            continue
    
        ## ファイルサイズをチェックする
        time.sleep(sleep_act_time)
        fout = open(g_test_result_file, 'a')
        fsize=os.path.getsize(g_diff_recv_file)
        if fsize > 0:
            message = g_test_ok
            print message ,
            sys.stdout.flush()
            fout.write(message)
        else:
            errorFlag = False
            # テスト結果 NG
            message = g_test_ng
            print message ,
            sys.stdout.flush()
            fout.write(message)

        fout.close()
    
        time.sleep(sleep_for_time)
    
    ## 6 ポート切断
    bool_ret = disconnect_ports()

    return errorFlag
##
# @if jp
# @brief ActivateとDeactivateだけを繰り返す。
#
# 接続した状態で Activate と Deactivate を繰り返す。
#
# @else
# @brief Only activation and deactivation are repeated. 
#
# Activate and Deactivate are repeated while connected. 
#
# @endif
def test_activate_deactivate(message,arg0,arg1,arg2):
    
    errorFlag = True
    ## 1 コネクタープロファイル設定
    make_connecter_profile(arg0,arg1,arg2)
    
    ## 3 ポート接続
    bool_ret = connect_ports()
    if bool_ret!=True:
        errorFlag = False
    
    for i in range(loop_count):
    
        ## 2 受信データファイル削除
        delete_recv_file()
    
        ## 4 アクティベート
        bool_ret = activate_components(sleep_recv_act_time)
        if bool_ret!=True:
            errorFlag = False
    
        time.sleep(sleep_act_time)
    
        ## 5 ディアクティベート
        bool_ret = deactivate_components()
        if bool_ret!=True:
            errorFlag = False
    
        ## 受信ファイル有無判定
        if os.path.isfile(g_diff_recv_file) == False:
            errorFlag = False
            fout = open(g_test_result_file, 'a')
            #message = message + g_check_message
            message = g_test_ng
            fout.write(message)
            fout.close()
            print message ,
            sys.stdout.flush()
            time.sleep(sleep_for_time)
            continue
    
        ## 7 送受信データ比較
        time.sleep(sleep_act_time)
        bret = diff_file()
    
        ## 差分ファイルからテスト結果出力
        fout = open(g_test_result_file, 'a')
        # bret==True なら送受信データ一致
        if bret == True:
            # テスト結果 OK
            message = g_test_ok
            print message ,
            sys.stdout.flush()
            fout.write(message + '\n')
        else:
            errorFlag = False
            # テスト結果 NG
            message = g_test_ng
            print message ,
            sys.stdout.flush()
            fout.write(message + '\n')
            # 受信データをテスト結果ファイルへコピー
            fin2 = open(g_diff_recv_file, 'r')
            while(1):
                s2 = fin2.readline()
                if len(s2) == 0:
                    break
                fout.write(s2)
            fin2.close()
        fout.close()
    
        time.sleep(sleep_for_time)
    
    ## 6 ポート切断
    bool_ret = disconnect_ports()

    return errorFlag
    
##
# @if jp
# @brief ActivateとDeactivateだけを繰り返す。
#
# 接続せずに Activate と Deactivate を繰り返す。
#
# @else
# @brief Only activation and deactivation are repeated. 
#
# Activate and Deactivate are repeated without connecting it. 
#
# @endif
def test_activate_deactivate_2(message,arg0,arg1,arg2):
    
    errorFlag = True
    
    for i in range(loop_count):
    
        ## 1 コネクタープロファイル設定
        make_connecter_profile(arg0,arg1,arg2)
    
        ## 4 アクティベート
        bool_ret = activate_components(sleep_recv_act_time)
        if bool_ret!=True:
            errorFlag = False
    
        time.sleep(sleep_act_time)
    
        ## 5 ディアクティベート
        bool_ret = deactivate_components()
        if bool_ret!=True:
            errorFlag = False
    
        ## テスト結果出力
        fout = open(g_test_result_file, 'a')
        if errorFlag:
            message = g_test_ok
        else :
            message = g_test_ng
        print message ,
        sys.stdout.flush()
        fout.write(message)
        fout.close()
    
        time.sleep(sleep_for_time)

    return errorFlag
    
##
# @if jp
# @brief 接続、activate、deactivate、切断を繰り返す。 
#
# データのチェックは行われない。
# periodic,newの接続テストでは、ポリシーがSKIP,NEWの場合、
# データの確認は行わない。
#
#
# @else
# @brief The connection, activate, deactivate, and cutting are repeated. 
#
# Data is not checked. 
#
# @endif
def test_connection_5(message,arg0,arg1,arg2):
    
    errorFlag = True
    for i in range(loop_count):
    
        ## 2 受信データファイル削除
        delete_recv_file()
    
        ## 1 コネクタープロファイル設定
        make_connecter_profile(arg0,arg1,arg2)
    
        ## 3 ポート接続
        bool_ret = connect_ports()
        if bool_ret!=True:
            errorFlag = False

        ## 4 アクティベート
        bool_ret = activate_components(sleep_recv_act_time)
        if bool_ret!=True:
            errorFlag = False
    
        time.sleep(sleep_act_time)
    
        ## 5 ディアクティベート
        bool_ret = deactivate_components()
        if bool_ret!=True:
            errorFlag = False
    
        ## 6 ポート切断
        bool_ret = disconnect_ports()
        if bool_ret!=True:
            errorFlag = False
    
        ## 受信ファイル有無判定
        if os.path.isfile(g_diff_recv_file) == False:
            errorFlag = False
            fout = open(g_test_result_file, 'a')
            message = g_test_ng
            fout.write(message + '\n')
            fout.close()
            print message,
            sys.stdout.flush()
            time.sleep(sleep_for_time)
            continue
    
        ## ファイルサイズをチェックする
        time.sleep(sleep_act_time)
        fout = open(g_test_result_file, 'a')
        fsize=os.path.getsize(g_diff_recv_file)
        if fsize > 0:
            message = g_test_ok
            print message ,
            sys.stdout.flush()
            fout.write(message)
        else:
            errorFlag = False
            # テスト結果 NG
            message = g_test_ng
            print message ,
            sys.stdout.flush()
            fout.write(message)

        fout.close()
    
        time.sleep(sleep_for_time)

    return errorFlag
##
# @if jp
# @brief 接続、activate、deactivate、切断を繰り返す。 
#
#
# @else
# @brief The connection, activate, deactivate, and cutting are repeated. 
#
#
# @endif
def test_connection(message,arg0,arg1,arg2):
    
    errorFlag = True
    for i in range(loop_count):
    
        ## 2 受信データファイル削除
        delete_recv_file()
    
        ## 1 コネクタープロファイル設定
        make_connecter_profile(arg0,arg1,arg2)
    
        ## 3 ポート接続
        bool_ret = connect_ports()
        if bool_ret!=True:
            errorFlag = False
    
        ## 4 アクティベート
        bool_ret = activate_components(sleep_recv_act_time)
        if bool_ret!=True:
            errorFlag = False
    
        time.sleep(sleep_act_time)
    
        ## 5 ディアクティベート
        bool_ret = deactivate_components()
        if bool_ret!=True:
            errorFlag = False
    
        ## 6 ポート切断
        bool_ret = disconnect_ports()
        if bool_ret!=True:
            errorFlag = False

    
        ## 受信ファイル有無判定
        if os.path.isfile(g_diff_recv_file) == False:
            errorFlag = False
            fout = open(g_test_result_file, 'a')
            message = g_test_ng
            fout.write(message + '\n')
            fout.close()
            print message,
            sys.stdout.flush()
            time.sleep(sleep_for_time)
            continue
    
        ## 7 送受信データ比較
        time.sleep(sleep_act_time)
        bret = diff_file()
    
        ## 差分ファイルからテスト結果出力
        fout = open(g_test_result_file, 'a')
        # bret==True なら送受信データ一致
        if bret == True:
            # テスト結果 OK
            message = g_test_ok
            print message ,
            sys.stdout.flush()
            fout.write(message + '\n')
        else:
            errorFlag = False
            # テスト結果 NG
            message = g_test_ng
            print message ,
            sys.stdout.flush()
            fout.write(message + '\n')
            # 受信データをテスト結果ファイルへコピー
            fin2 = open(g_diff_recv_file, 'r')
            while(1):
                s2 = fin2.readline()
                if len(s2) == 0:
                    break
                fout.write(s2)
            fin2.close()
        fout.close()
    
        time.sleep(sleep_for_time)

    return errorFlag
    
    
##
# @if jp
# @brief 接続、activate、切断、deactivateを繰り返す。 
#
#
# @else
# @brief 
#
#
# @endif
def test_connection_2(message,arg0,arg1,arg2):
    
    errorFlag = True
    
    for i in range(loop_count):
    
        ## 2 受信データファイル削除
        delete_recv_file()
    
        ## 1 コネクタープロファイル設定
        make_connecter_profile("flush", "", 0)
    
        ## 3 ポート接続
        bool_ret = connect_ports()
        if bool_ret!=True:
            errorFlag = False
    
        ## 4 アクティベート
        bool_ret = activate_components(sleep_recv_act_time)
        if bool_ret!=True:
            errorFlag = False
    
        time.sleep(sleep_act_time)
    
        ## 6 ポート切断
        bool_ret = disconnect_ports()
        if bool_ret!=True:
            errorFlag = False
    
        ## 5 ディアクティベート
        bool_ret = deactivate_components()
        if bool_ret!=True:
            errorFlag = False
    
        ## 受信ファイル有無判定
        if os.path.isfile(g_diff_recv_file) == False:
            errorFlag = False
            fout = open(g_test_result_file, 'a')
            message = g_test_ng
            fout.write(message + '\n')
            fout.close()
            print message,
            sys.stdout.flush()
            time.sleep(sleep_for_time)
            continue
    
        ## 7 送受信データ比較
        time.sleep(sleep_act_time)
        bret = diff_file()
    
        ## 差分ファイルからテスト結果出力
        fout = open(g_test_result_file, 'a')
        # bret==True なら送受信データ一致
        if bret == True:
            # テスト結果 OK
            message = g_test_ok
            print message ,
            sys.stdout.flush()
            fout.write(message + '\n')
        else:
            errorFlag = False
            # テスト結果 NG
            message = g_test_ng
            print message ,
            sys.stdout.flush()
            fout.write(message + '\n')
            # 受信データをテスト結果ファイルへコピー
            fin2 = open(g_diff_recv_file, 'r')
            while(1):
                s2 = fin2.readline()
                if len(s2) == 0:
                    break
                fout.write(s2)
            fin2.close()
        fout.close()
        ## 差分ファイルからテスト結果出力
        time.sleep(sleep_for_time)

    return errorFlag
    
    
##
# @if jp
# @brief activate、接続、deactivate、切断を繰り返す。 
#
#  データのチェックは行われない。 
#  注意：Activateを先に行っている為、受信データは途中からの内容になります。
#  periodic,newの接続テストでは、ポリシーがSKIP,NEWの場合、
#  データの確認は行わない。
#
# @else
# @brief 
#
#
# @endif
def test_connection_3(message,arg0,arg1,arg2):
    
    errorFlag = True

    for i in range(loop_count):
    
        ## 2 受信データファイル削除
        delete_recv_file()
    
        ## 1 コネクタープロファイル設定
        make_connecter_profile(arg0,arg1,arg2)
    
        ## 4 アクティベート
        bool_ret = activate_components(sleep_recv_act_time)
        if bool_ret!=True:
            errorFlag = False
    
        ## 3 ポート接続
        bool_ret = connect_ports()
        if bool_ret!=True:
            errorFlag = False
    
        time.sleep(sleep_act_time)
    
        ## 5 ディアクティベート
        bool_ret = deactivate_components()
        if bool_ret!=True:
            errorFlag = False
    
        ## 6 ポート切断
        bool_ret = disconnect_ports()
        if bool_ret!=True:
            errorFlag = False
    
        ## 受信ファイル有無判定
        if os.path.isfile(g_diff_recv_file) == False:
            errorFlag = False
            fout = open(g_test_result_file, 'a')
            message = g_test_ng
            fout.write(message + '\n')
            fout.close()
            print message,
            sys.stdout.flush()
            time.sleep(sleep_for_time)
            continue
    
        ## 7 送受信データ比較
        time.sleep(sleep_act_time)
        fout = open(g_test_result_file, 'a')
        fsize=os.path.getsize(g_diff_recv_file)
        if fsize > 0:
            message = g_test_ok
            print message ,
            sys.stdout.flush()
            fout.write(message)
        else:
            errorFlag = False
            # テスト結果 NG
            message = g_test_ng
            print message ,
            sys.stdout.flush()
            fout.write(message)

        fout.close()
    
        time.sleep(sleep_for_time)
    
    return errorFlag

    
##
# @if jp
# @brief activate、接続、切断、deactivateを繰り返す。 
#
#  データのチェックは行われない。 
#  注意：Activateを先に行っている為、受信データは途中からの内容になります。
#  periodic,newの接続テストでは、ポリシーがSKIP,NEWの場合、
#  データの確認は行わない。
#
# @else
# @brief 
#
#
# @endif
def test_connection_4(message,arg0,arg1,arg2):
    
    errorFlag = True
    for i in range(loop_count):
    
        ## 2 受信データファイル削除
        delete_recv_file()
    
        ## 1 コネクタープロファイル設定
        make_connecter_profile(arg0,arg1,arg2)
    
        ## 4 アクティベート
        bool_ret = activate_components(sleep_recv_act_time)
        if bool_ret!=True:
            errorFlag = False
    
        ## 3 ポート接続
        bool_ret = connect_ports()
        if bool_ret!=True:
            errorFlag = False
    
        time.sleep(sleep_act_time)
    
        ## 6 ポート切断
        bool_ret = disconnect_ports()
        if bool_ret!=True:
            errorFlag = False
    
        ## 5 ディアクティベート
        bool_ret = deactivate_components()
        if bool_ret!=True:
            errorFlag = False
    
        ## 受信ファイル有無判定
        if os.path.isfile(g_diff_recv_file) == False:
            errorFlag = False
            fout = open(g_test_result_file, 'a')
            message = g_test_ng
            fout.write(message + '\n')
            fout.close()
            print message,
            sys.stdout.flush()
            time.sleep(sleep_for_time)
            continue
    
        ## 7 送受信データ比較
        time.sleep(sleep_act_time)
        fout = open(g_test_result_file, 'a')
        fsize=os.path.getsize(g_diff_recv_file)
        if fsize > 0:
            message = g_test_ok
            print message ,
            sys.stdout.flush()
            fout.write(message)
        else:
            errorFlag = False
            # テスト結果 NG
            message = g_test_ng
            print message ,
            sys.stdout.flush()
            fout.write(message)

        fout.close()
    
        time.sleep(sleep_for_time)

    return errorFlag
    
##
# @if jp
# @brief テストテーブル
# @else
# @brief Test table
# @endif
test_table = [
    [ test_connect_disconnect,
      "Connect(out->in, flush) -> Disconnect",
      "flush", "", 0 ],
    [ test_connect_disconnect,
      "Connect(in->out, flush) -> Disconnect",
      "flush", "", 1 ],
    [ test_activate_deactivate,
      "Connecting(out->in, flush), Activate -> send/recv -> Deactivate",
      "flush", "", 0 ],
    [ test_activate_deactivate,
      "Connecting(in->out, flush), Activate -> send/recv -> Deactivate",
      "flush", "", 1 ],
    [ test_activate_deactivate_2,
      "Not Connect(out->in, flush), Activate -> Deactivate",
      "flush", "", 0 ],
    [ test_connection,
      "Connect(out->in, flush) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "flush", "", 0 ],
    [ test_connection,
      "Connect(in->out, flush) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "flush", "", 1 ],
    [ test_connection_2,
      "Connect(out->in, flush) -> Activate -> send/recv -> Disconnect -> Deactivate",
      "flush", "", 0 ],
    [ test_connection_2,
      "Connect(in->out, flush) -> Activate -> send/recv -> Disconnect -> Deactivate",
      "flush", "", 1 ],
    [ test_connection_3,
      "Activate -> Connect(out->in, flush) -> send/recv -> Deactivate -> Disconnect",
      "flush", "", 0 ],
    [ test_connection_3,
      "Activate -> Connect(in->out, flush) -> send/recv -> Deactivate -> Disconnect",
      "flush", "", 1 ],
    [ test_connection_4,
      "Activate -> Connect(out->in, flush) -> send/recv -> Disconnect -> Deactivate",
      "flush", "", 0 ],
    [ test_connection_4,
      "Activate -> Connect(in->out, flush) -> send/recv -> Disconnect -> Deactivate",
      "flush", "", 1 ],
    [ test_connect_disconnect,
      "Connect(out->in, new,ALL) -> Disconnect",
      "new", "ALL", 0 ],
    [ test_connect_disconnect,
      "Connect(out->in, new,FIFO) -> Disconnect",
      "new", "FIFO", 0 ],
    [ test_connect_disconnect,
      "Connect(out->in, new,NEW) -> Disconnect",
      "new", "NEW", 0 ],
    [ test_connect_disconnect,
      "Connect(out->in, new,SKIP) -> Disconnect",
      "new", "SKIP", 0 ],
    [ test_connect_disconnect,
      "Connect(in->out, new,ALL) -> Disconnect",
      "new", "ALL", 1 ],
    [ test_connect_disconnect,
      "Connect(in->out, new,FIFO) -> Disconnect",
      "new", "FIFO", 1 ],
    [ test_connect_disconnect,
      "Connect(in->out, new,NEW) -> Disconnect",
      "new", "NEW", 1 ],
    [ test_connect_disconnect,
      "Connect(in->out, new,SKIP) -> Disconnect",
      "new", "SKIP", 1 ],
    [ test_activate_deactivate,
      "Connecting(out->in, new,ALL), Activate -> send/recv -> Deactivate",
      "new", "ALL", 0 ],
    [ test_activate_deactivate,
      "Connecting(out->in, new,FIFO), Activate -> send/recv -> Deactivate",
      "new", "FIFO", 0 ],
    [ test_activate_deactivate,
      "Connecting(out->in, new,NEW), Activate -> send/recv -> Deactivate",
      "new", "NEW", 0 ],
    [ test_activate_deactivate_2,
      "Connecting(out->in, new,SKIP), Activate -> send/recv -> Deactivate",
      "new", "SKIP", 0 ],
    [ test_activate_deactivate,
      "Connecting(in->out, new,ALL), Activate -> send/recv -> Deactivate",
      "new", "ALL", 1 ],
    [ test_activate_deactivate,
      "Connecting(in->out, new,FIFO), Activate -> send/recv -> Deactivate",
      "new", "FIFO", 1 ],
    [ test_activate_deactivate_2,
      "Connecting(in->out, new,NEW), Activate -> send/recv -> Deactivate",
      "new", "NEW", 1 ],
    [ test_activate_deactivate_2,
      "Connecting(in->out, new,SKIP), Activate -> send/recv -> Deactivate",
      "new", "SKIP", 1 ],
    [ test_connection,
      "Connect(out->in, new,ALL) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "new", "ALL", 0 ],
    [ test_connection,
      "Connect(out->in, new,FIFO) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "new", "FIFO", 0 ],
    [ test_connection_5,
      "Connect(out->in, new,NEW) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "new", "NEW", 0 ],
    [ test_connection_5,
      "Connect(out->in, new,SKIP) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "new", "SKIP", 0 ],
    [ test_connection,
      "Connect(in->out, new,ALL) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "new", "ALL", 1 ],
    [ test_connection,
      "Connect(in->out, new,FIFO) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "new", "FIFO", 1 ],
    [ test_connection_5,
      "Connect(in->out, new,NEW) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "new", "NEW", 1 ],
    [ test_connection_5,
      "Connect(in->out, new,SKIP) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "new", "SKIP", 1 ],
    [ test_connect_disconnect,
      "Connect(out->in, periodic,ALL) -> Disconnect",
      "periodic", "ALL", 0 ],
    [ test_connect_disconnect,
      "Connect(out->in, periodic,FIFO) -> Disconnect",
      "periodic", "FIFO", 0 ],
    [ test_connect_disconnect,
      "Connect(out->in, periodic,NEW) -> Disconnect",
      "periodic", "NEW", 0 ],
    [ test_connect_disconnect,
      "Connect(out->in, periodic,SKIP) -> Disconnect",
      "periodic", "SKIP", 0 ],
    [ test_connect_disconnect,
      "Connect(in->out, periodic,ALL) -> Disconnect",
      "periodic", "ALL", 1 ],
    [ test_connect_disconnect,
      "Connect(in->out, periodic,FIFO) -> Disconnect",
      "periodic", "FIFO", 1 ],
    [ test_connect_disconnect,
      "Connect(in->out, periodic,NEW) -> Disconnect",
      "periodic", "NEW", 1 ],
    [ test_connect_disconnect,
      "Connect(in->out, periodic,SKIP) -> Disconnect",
      "periodic", "SKIP", 1 ],
    [ test_activate_deactivate,
      "Connecting(out->in, periodic,ALL), Activate -> send/recv -> Deactivate",
      "periodic", "ALL", 0 ],
    [ test_activate_deactivate,
      "Connecting(out->in, periodic,FIFO), Activate -> send/recv -> Deactivate",
      "periodic", "FIFO", 0 ],
    [ test_activate_deactivate_2,
      "Connecting(out->in, periodic,NEW), Activate -> send/recv -> Deactivate",
      "periodic", "NEW", 0 ],
    [ test_activate_deactivate_2,
      "Connecting(out->in, periodic,SKIP), Activate -> send/recv -> Deactivate",
      "periodic", "SKIP", 0 ],
    [ test_activate_deactivate,
      "Connecting(in->out, periodic,ALL), Activate -> send/recv -> Deactivate",
      "periodic", "ALL", 1 ],
    [ test_activate_deactivate,
      "Connecting(in->out, periodic,FIFO), Activate -> send/recv -> Deactivate",
      "periodic", "FIFO", 1 ],
    [ test_activate_deactivate_2,
      "Connecting(in->out, periodic,NEW), Activate -> send/recv -> Deactivate",
      "periodic", "NEW", 1 ],
    [ test_activate_deactivate_2,
      "Connecting(in->out, periodic,SKIP), Activate -> send/recv -> Deactivate",
      "periodic", "SKIP", 1 ],
    [ test_connection,
      "Connect(out->in, periodic,ALL) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "periodic", "ALL", 0 ],
    [ test_connection,
      "Connect(out->in, periodic,FIFO) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "periodic", "FIFO", 0 ],
    [ test_connection_5,
      "Connect(out->in, periodic,NEW) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "periodic", "NEW", 0 ],
    [ test_connection_5,
      "Connect(out->in, periodic,SKIP) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "periodic", "SKIP", 0 ],
    [ test_connection,
      "Connect(in->out, periodic,ALL) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "periodic", "ALL", 1 ],
    [ test_connection,
      "Connect(in->out, periodic,FIFO) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "periodic", "FIFO", 1 ],
    [ test_connection_5,
      "Connect(in->out, periodic,NEW) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "periodic", "NEW", 1 ],
    [ test_connection_5,
      "Connect(in->out, periodic,SKIP) -> Activate -> send/recv -> Deactivate -> Disconnect",
      "periodic", "SKIP", 1 ],

]
##
# @if jp
# @brief main
# @else
# @brief main
# @endif
def main():
    
    initGlobal()
    ok_counter = 0
    ng_counter = 0

    exit = 0
    result_message = ""
    for element in test_table:
        components_entry()
        
        message = g_mess_header + g_test_case 
        message = message + element[1]
        message = message + g_mess_footer
        fout = open(g_test_result_file, 'a')
        fout.write('\n' + message)
        fout.close()
        print "" 
        print message
        ret = element[0](message,element[2],element[3],element[4])
        components_exit()
        if ret == False:
            ng_counter = ng_counter + 1
            exit = 1
            result_message = "!!!FAILURES!!!"
            break;
        else :
            ok_counter = ok_counter + 1
        

    print ""
    print result_message 
    print "Test Results: " 
    print "Run:",ok_counter+ng_counter,"   Failures:", ng_counter
    return exit 
if __name__ == "__main__":
  main()
