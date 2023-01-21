#!/usr/bin/env python3

import serial
import os
from time import sleep

class PacketHandler:
  def __init__(self, _port_name, _baud_rate):
    self.port_name = _port_name
    self.baud_rate = _baud_rate
    self._ser = serial.Serial(self.port_name, self.baud_rate, timeout=0.001)
    self._ser.reset_input_buffer()
    self._ser.reset_output_buffer()
    
    self.Wheel_Pos = [0.0, 0.0] # r, l
    self.Wheel_RPM = [0, 0] # r, l
    self.is_Lift = [False, False] # lift1, lift0
    self.Lift_err = [False, False] # lift1_err, lift0_err
    self.Wheel_err = [False, False] # Wheel_r_err, Wheel_l_err

  def get_port_state(self):
    return self._ser.is_open

  def close_port(self):
    print("Close Port")
    self._ser.close()

  def get_Wheel_pos(self):
    return self.Wheel_Pos
    
  def get_Wheel_RPM(self):
    return self.Wheel_RPM

  def get_is_Lift(self):
    return self.is_Lift

  def get_Lift_err(self):
    return self.Lift_err

  def get_Wheel_err(self):
    return self.Wheel_err

  def checksum(self, initial, data):
      _d = initial
      for i, temp in enumerate(data):
          _d = _d ^ temp
      _d = _d + 1
      if _d == 256:
        _d = 0
      return bytes([_d])

  def read_packet(self):
    if self.get_port_state() == True:
      whole_packet = self._ser.readline().split(b'\\')[0]
      if len(whole_packet) == 16:
        if bytes([whole_packet[0]]) == b'\xff' and bytes([whole_packet[1]]) == b'\xff':
          if self.checksum(whole_packet[2], whole_packet[3:15]) == whole_packet[15].to_bytes(1,byteorder='big'):
            self.Wheel_Pos[0] = (int.from_bytes(whole_packet[2:6], byteorder='big',signed=True))/4096 # wheel_l
            self.Wheel_Pos[1] = (int.from_bytes(whole_packet[6:10], byteorder='big',signed=True))/4096 # wheel_r
            self.Wheel_RPM[0] = int.from_bytes(whole_packet[10:12], byteorder='little',signed=True) # l rpm
            self.Wheel_RPM[1] = int.from_bytes(whole_packet[12:14], byteorder='little',signed=True) # r rpm
            print("--------------------")
            for i, _temp in enumerate(whole_packet):
              print(_temp)
            print("--------------------")  
            # print(whole_packet)
            # print(whole_packet[2:6])
            # print(whole_packet[6:10])
            print((self.Wheel_RPM))
            if len(bin(whole_packet[14])) > 6:
              if int(bin(whole_packet[14])[5]) == 1: # lift 1
                self.is_Lift[0] = True
              else: self.is_Lift[0] = False
            else: self.is_Lift[0] = False
            if len(bin(whole_packet[14])) > 7:
              if int(bin(whole_packet[14])[6]) == 1: # lift 0
                self.is_Lift[1] = True
              else: self.is_Lift[1] = False
            else: self.is_Lift[1] = False
            if len(bin(whole_packet[14])) > 8:
              
              if int(bin(whole_packet[14])[7]) == 1: # lift 1 err
                self.Lift_err[0] = True
              else: self.Lift_err[0] = False      
            else: self.Lift_err[0] = False      
            if len(bin(whole_packet[14])) > 9:
              if int(bin(whole_packet[14])[8]) == 1: # lift 0 err
                self.Lift_err[1] = True
              else: self.Lift_err[1] = False           
            else: self.Lift_err[1] = False        

            if len(bin(whole_packet[14])) > 10:            
              if int(bin(whole_packet[14])[9]) == 1: # wheel_r err
                self.Wheel_err[0] = True
              else: self.Wheel_err[0] = False        
            else: self.Wheel_err[0] = False       

            if len(bin(whole_packet[14])) > 11:  
              if int(bin(whole_packet[14])[10]) == 1: # wheel_l err
                self.Wheel_err[1] = True
              else: self.Wheel_err[1] = False    
            else: self.Wheel_err[1] = False    
          else:
            print("error packet1")
        else:
          print("error packet2")

  def write_motor(self, r_rpm, l_rpm, lift):
    header_1 = b'\xff'
    header_2 = b'\xff'
    byte_r_rpm = r_rpm.to_bytes(2, 'little', signed=True)
    byte_l_rpm = l_rpm.to_bytes(2, 'little', signed=True)
    if lift:
      byte_lift = b'\x03' # up
    else:
      byte_lift = b'\x00' # down
    _check = byte_r_rpm + byte_l_rpm + byte_lift
    _checksum = self.checksum(_check[0], _check[1:5])
    send_data = header_1 + header_2 + _check + _checksum
    self._ser.write(send_data)
    
    
