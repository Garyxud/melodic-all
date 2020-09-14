#!/usr/bin/env python
# coding:utf-8

import serial
from threading import Lock

class SeedCommand:
  def __init__(self):
    pass

  def open_port(self,port='/dev/ttyACM0',baud=115200):
    self.ser=serial.Serial(port, baud, timeout=0.1)

  #COM Port Open
  def open_com(self):
    self.ser.write("Z0\r")
    self.ser.write("O\r")

  #Com Port Close
  def close_com(self):
    self.ser.write("C\r")
    self.ser.close()

  #Binary data to Hex String data and decide number of byte
  def int2str(self,value,byte=1):
    if value < 0: value = 0xFFFFFF + 1 + value
    return str(hex(value).upper()[2:][-2:].rjust(2**byte,"0"))

  #Binary data to Hex data and decide number of byte
  def int2hex(self,value,byte=1):
    if value < 0:value = 0xFFFFFF + 1 + value
    return hex(value).upper()[2:][-2:].rjust(2**byte,"0")

  #SEED CAN data send
  def write_buffer(self,send_data):
    can_data = ''.join(send_data) + '\r'
    self.ser.write(can_data)
    #print "send\t:%s" % cand_data

####################################################################################
  def write_serial_command(self,id_num,d3,d4,d5,d6,d7,d8):
    send_data = 12*[0]
    send_data[0] ='t'

    if d3 == 0x53: send_data[1] = self.int2str(0x00,0)
    else: send_data[1] = self.int2str(0x03,0)

    send_data[2] = self.int2str(id_num)
    send_data[3] = self.int2str(8,0)
    send_data[4] = self.int2str(0xF0 + id_num)
    send_data[5] = self.int2str(0x00)
    send_data[6] = self.int2str(d3)
    send_data[7] = self.int2str(d4)
    send_data[8] = self.int2str(d5)
    send_data[9] = self.int2str(d6)
    send_data[10] = self.int2str(d7)
    send_data[11] = self.int2str(d8)

    self.write_buffer(send_data)

  def read_serial_command(self):
    ret_str = ''
    while(self.ser.inWaiting() < 1):
      pass
    while(self.ser.read(1) != 't'):
      pass
    ret_str = 't'
    # Get Data Length 
    ret = self.ser.read(4)
    data_len = int(ret[-1],16)
    ret_str += ret
    # Get All data
    ret = self.ser.read(data_len*2)
    ret_str += ret
    #print "Receive Data \t:%s" % ret_str
    return ret_str

#---------  Get inormation command(0x40~0x4F)  ---------
  def get_position(self,id_num):
    self.ser.flushInput()
    self.ser.flushOutput()
    cmd = 0
    speed = 0
    position = 0

    self.write_serial_command(id_num,0x42,id_num,0x00,0x00,0x00,0x00)
    data = self.read_serial_command()
    cmd = int(data[9]+data[10],16)
    if(cmd != 0x42): return [False,0,0]
    else:
      speed = int(data[11]+data[12]+data[13]+data[14],16)
      position = int(data[15]+data[16]+data[17]+data[18]+data[19]+data[20],16)

      if position > 0xFFFFFF/2: position = position - 0xFFFFFF
      elif position == 0xFFFFFF: position = 0
      else : position = position

      return [True,speed,position]

#---------  actuate motor command(0x50~0x5F)  ---------
  def on_servo(self,id_num,data):
    self.write_serial_command(id_num,0x50,id_num,data,0x00,0x00,0x00)

  def stop_motor(self,id_num):
    self.write_serial_command(id_num,0x51,0x00,0x00,0x00,0x00,0x00)

  def run_script(self,id_num,s_num):
    if s_num > 0x00 and s_num < 0x0F :
      self.write_serial_command(id_num,0x5F,id_num,s_num,0x00,0x00,0x00)

  def wait_for_script_end(self,_number):
    number_of_end = 0
    signal = 0
    if( number_of_end < _number):
      while(signal != 0xFF):
        response = self.read_serial_command()
        signal = int(response[13]+response[14],16)
      number_of_end += 1
      
  def actuate_continuous_absolute_position(self,id_num,time,pos):
    data = 5*[0]

    if pos >= 0xFFFFFF/2: pos = 0xFFFFFF/2
    elif pos <= -0xFFFFFF/2: pos = -0xFFFFFF/2

    data[0] = time >> 8
    data[1] = time  
    data[2] = pos >> 16
    data[3] = pos >> 8
    data[4] = pos 

    self.write_serial_command(id_num,0x64,data[0],data[1],data[2],data[3],data[4])
