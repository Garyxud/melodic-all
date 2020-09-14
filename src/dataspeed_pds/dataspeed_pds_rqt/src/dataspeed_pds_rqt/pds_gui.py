#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Dataspeed Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright notice,
#       this list of conditions and the following disclaimer in the documentation
#       and/or other materials provided with the distribution.
#     * Neither the name of Dataspeed Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from this
#       software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import rospy
import rospkg
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGraphicsScene, QLabel, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsTextItem
from python_qt_binding.QtCore import Qt, Signal, QTimer, QDateTime, QDate, QTime, QPoint, QEvent, QObject, pyqtSignal
from python_qt_binding.QtGui import QPixmap, QFont, QPen, QColor, QBrush, QImage
    
from cmath import *
from math import *
from rospy.rostime import Time
from numpy import isfinite

from dataspeed_pds_msgs.msg import Status, Channel, Relay, Mode, Script

# In order to style the buttons I had to make them labels instead,
# but in order to make them clickable I had to use the below code.
def clickable(widget):
    class Filter(QObject):
        clicked = pyqtSignal()
        def eventFilter(self, obj, event):
        
            if obj == widget:
                if event.type() == QEvent.MouseButtonRelease:
                    if obj.rect().contains(event.pos()):
                        self.clicked.emit()
                        # The developer can opt for .emit(obj) to get the object within the slot.
                        return True
            
            return False
    
    filter = Filter(widget)
    widget.installEventFilter(filter)
    return filter.clicked

class PowerDistributionGui(Plugin):
    gui_update_timer = QTimer()
    
    # Channel Status
    current_status = None
    # Channel Current in amps
    current_amps = None
    
    # Switching between tabs and full GUI
    is_currently_tab = False
    widget_count = 0
    current_tab_idx = -1
    raw_data_tab_idx = 5
        
    def __init__(self, context):
        super(PowerDistributionGui, self).__init__(context)

        # Qt setup
        self.context_ = context
        self.init_gui('full')
        
        # ROS setup
        self.subscribe_topics()
        self.advertise_topics()
        self.last_update = 0
        
    def init_gui(self, gui_type):
        if gui_type == 'full':
            self.spawn_full_gui()

        self.bind_callbacks()
        self.reset_gui_timer()

    def update_channel_buttons(self):
        for index in range(0, 24):
            channelStatusName = "NONE"
            buttonStyle = ""

            if self.current_status.chan[index].status == Channel.ON:
                buttonStyle += 'background-color: rgb(78, 154, 6);' # Green
                channelStatusName = "ON"
            elif self.current_status.chan[index].status == Channel.NO_LOAD:
                buttonStyle += 'background-color: rgb(78, 154, 6);' # Green
                channelStatusName = "NO LOAD"
            elif self.current_status.chan[index].status == Channel.OVER_CURRENT:
                buttonStyle += 'background-color: rgb(252, 175, 62);' # Orange
                channelStatusName = "OVER CURRENT"
            elif self.current_status.chan[index].status == Channel.OFF:
                buttonStyle += 'background-color: rgb(130, 130, 130);' # Gray
                channelStatusName = "OFF"
            elif self.current_status.chan[index].status == Channel.BAD_RELAY:
                buttonStyle += 'background-color: rgb(229, 90, 90);' # Red
                channelStatusName = "BAD RELAY"
            elif self.current_status.chan[index].status == Channel.BAD_FUSE:
                buttonStyle += 'background-color: rgb(229, 90, 90);' # Red
                channelStatusName = "BAD FUSE"

            buttonText  = '<html><head/><body><p align="center"><span style=" font-size:12pt;">'
            buttonText += '#{} {}'.format(index + 1, channelStatusName)
            buttonText += '</span></p><p align="center"><span style=" font-size:28pt;">'
            buttonText += '{0:.2f} A'.format(self.current_status.chan[index].current)
            buttonText += '</span></p><p align="center"><span style=" font-size:8pt;">'
            buttonText += '</span></p></body></html>'
            getattr(self._widget, "buttonChannel{:0>2d}".format(index + 1)).setText(buttonText)

            buttonStyle += 'color: rgb(0, 0, 0);'
            buttonStyle += 'border-radius: 12px;'
            buttonStyle += 'padding: 12px;'
            getattr(self._widget, "buttonChannel{:0>2d}".format(index + 1)).setStyleSheet(buttonStyle)

    def update_inverter_buttons(self):
        # Update text on Master Inverter Button
        buttonTextMaster  = '<html><head/><body><p align="center"><span style=" font-size:20pt;">Toggle Master</span></p><p align="center"><span style=" font-size:28pt;">'
        buttonTextMaster += 'ON' if self.current_status.master.inverter.status else 'OFF'
        buttonTextMaster += '</span></p></body></html>'
        self._widget.buttonInverterMaster.setText(buttonTextMaster)

        # Update colors on Master Inverter Button
        buttonStyleMaster  = 'color: rgb(0, 0, 0);'
        buttonStyleMaster += 'border-radius: 12px;'
        buttonStyleMaster += 'padding: 12px;'
        buttonStyleMaster += 'background-color: rgb(78, 154, 6);' if self.current_status.master.inverter.status else 'background-color: rgb(229, 90, 90);'
        self._widget.buttonInverterMaster.setStyleSheet(buttonStyleMaster)

        # Update text on Slave Inverter Button
        buttonTextSlave  = '<html><head/><body><p align="center"><span style=" font-size:20pt;">Toggle Slave</span></p><p align="center"><span style=" font-size:28pt;">'
        buttonTextSlave += 'ON' if self.current_status.slave[0].inverter.status else 'OFF'
        buttonTextSlave += '</span></p></body></html>'
        self._widget.buttonInverterSlave.setText(buttonTextSlave)

        # Update colors on Slave Inverter Button
        buttonStyleSlave  = 'color: rgb(0, 0, 0);'
        buttonStyleSlave += 'border-radius: 12px;'
        buttonStyleSlave += 'padding: 12px;'
        buttonStyleSlave += 'background-color: rgb(78, 154, 6);' if self.current_status.slave[0].inverter.status else 'background-color: rgb(229, 90, 90);'
        self._widget.buttonInverterSlave.setStyleSheet(buttonStyleSlave)

    def update_mode(self):
        self._widget.radioButtonAuto.setChecked(self.current_status.mode.mode == Mode.AUTO)
        self._widget.radioButtonManual.setChecked(self.current_status.mode.mode == Mode.MANUAL)
        self._widget.radioButtonValet.setChecked(self.current_status.mode.mode == Mode.VALET)

    def click_mode_auto(self):
        out_msg = Mode()
        out_msg.mode = Mode.AUTO
        self.pub_cmd_mode.publish(out_msg)

    def click_mode_manual(self):
        out_msg = Mode()
        out_msg.mode = Mode.MANUAL
        self.pub_cmd_mode.publish(out_msg)

    def click_mode_valet(self):
        out_msg = Mode()
        out_msg.mode = Mode.VALET
        self.pub_cmd_mode.publish(out_msg)

    def recv_status(self, msg):
        self.current_status = msg
        self.last_update = time.time()

    def subscribe_topics(self):
        sub_status = rospy.Subscriber('/pds/status', Status, self.recv_status)
    
    def advertise_topics(self):
        self.pub_cmd_request = rospy.Publisher('/pds/relay', Relay, queue_size=1)
        self.pub_cmd_mode = rospy.Publisher('/pds/mode', Mode, queue_size=1)
        self.pub_cmd_script = rospy.Publisher('/pds/script', Script, queue_size=1)
        
    def reset_gui_timer(self):
        self.gui_update_timer = QTimer(self._widget)
        self.gui_update_timer.setInterval(100)
        self.gui_update_timer.setSingleShot(False)
        self.gui_update_timer.timeout.connect(lambda: self.update_gui_cb())
        self.gui_update_timer.start()
        
    def click_script_stop(self):
        out_msg = Script()
        out_msg.script = Script.NONE
        self.pub_cmd_script.publish(out_msg)

    def click_script_startup(self):
        out_msg = Script()
        out_msg.script = Script.STARTUP
        self.pub_cmd_script.publish(out_msg)

    def click_script_shutdown(self):
        out_msg = Script()
        out_msg.script = Script.SHUTDOWN
        self.pub_cmd_script.publish(out_msg)

    def click_inverter_master(self):
        out_msg = Relay()
        # Inverter 1 is Channel 48
        out_msg.channel = 48
        out_msg.request = Relay.TOGGLE
        self.pub_cmd_request.publish(out_msg)

    def click_inverter_slave(self):
        out_msg = Relay()
        # Inverter 2 is Channel 49
        out_msg.channel = 49
        out_msg.request = Relay.TOGGLE
        self.pub_cmd_request.publish(out_msg)

    def update_gui_cb(self):
        # Reposition the hovering data label.
        self._widget.labelNoData.move(
            self._widget.width()/2 - self._widget.labelNoData.width()/2,
            self._widget.height()/2 - self._widget.labelNoData.height()/2)

        if time.time() - self.last_update > 3:
            # Clear status info if we haven't gotten an update in 3 seoncds.
            self.current_status = None
        if self.current_status is None:
            self._widget.labelNoData.show()
            return
        else:
            self._widget.labelNoData.hide()
            self.update_channel_buttons()
            self.update_inverter_buttons()
            self.update_mode()

    def get_size(self, widget):
        print(str(widget.width()) + "," + str(widget.height()))

    def spawn_full_gui(self):
        super(PowerDistributionGui, self).__init__(self.context_)
        # Give QObjects reasonable names
        self.setObjectName('PowerDistributionGui')

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('dataspeed_pds_rqt'), 'resource', 'PowerDistributionGui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('PowerDistributionGui' + str(self.widget_count))
        self.widget_count += 1
        # Add widget to the user interface
        self.context_.add_widget(self._widget)
        
    def spawn_tab_gui(self):
        super(PowerDistributionGui, self).__init__(self.context_)
        # Give QObjects reasonable names
        self.setObjectName('PowerDistributionGui')

        # Create QWidget
        self._widget = QWidget()
        
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('dataspeed_pds_rqt'), 'resource', 'PowerDistributionGuiTabs.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('PowerDistributionGui' + str(self.widget_count))
        self.widget_count += 1
        # Add widget to the user interface
        self.context_.add_widget(self._widget)
        
    def create_click_channel(self, number):
        def click_channel():
            out_msg = Relay()
            # Indexes 0-23 represent Channels 1-24
            out_msg.channel = number - 1
            out_msg.request = Relay.TOGGLE
            self.pub_cmd_request.publish(out_msg)
        return click_channel

    def bind_callbacks(self):
        clickable(self._widget.buttonInverterMaster).connect(self.click_inverter_master)
        clickable(self._widget.buttonInverterSlave).connect(self.click_inverter_slave)
        self._widget.radioButtonAuto.clicked.connect(self.click_mode_auto)
        self._widget.radioButtonManual.clicked.connect(self.click_mode_manual)
        self._widget.radioButtonValet.clicked.connect(self.click_mode_valet)
        self._widget.buttonScriptStop.clicked.connect(self.click_script_stop)
        self._widget.buttonScriptStartup.clicked.connect(self.click_script_startup)
        self._widget.buttonScriptShutdown.clicked.connect(self.click_script_shutdown)

        for index in range(1, 24 + 1):
            clickable(getattr(self._widget, "buttonChannel{:0>2d}".format(index))).connect(self.create_click_channel(index))

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for dataspeed_pds_rqt plugin')
        #group.add_argument('node_name', nargs='*', default=[], help='Node(s) to open automatically')
    
    def shutdown_plugin(self):
        pass
