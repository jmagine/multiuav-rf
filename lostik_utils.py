'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Jan 02 2020
                                      SARC

  File Name  : lostik_utils.py
  Description: LoStik controller for packaged usage
---*-----------------------------------------------------------------------*'''

import serial
import threading
import time

class LS_Controller(threading.Thread):
  def __init__(self, port, baudrate=57600):
    super(LS_Controller, self).__init__()
    self.buf = []
    self.buf_pointer = 0
    self.frame_count = 0
    self.daemon = True
    self.end_thread = False

    self.ser = serial.Serial(port, baudrate=baudrate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
    self.ser.reset_input_buffer()
    self.ser.reset_output_buffer()
    self.name = self.ser.name

    print("[%s] connected" % (self.name))
    self.start()

  def send_cmd(self, cmd):
    if cmd == 'end_thread':
      print("[%s] stopping lostik controller" % (self.name))
      self.end_thread = True

  def write_serial(self, cmd, block=False):
    val = ('%s\r\n' % (cmd)).encode('utf-8')
    end = ""

    self.ser.write(val)
    self.ser.flush()

    if block:
      res = []
      while len(res) == 0:
        self.read_serial()
        res = self.proc_buf()
        
      end = "\t--- ser rx: %s" % (res[0])
    print("[%s] ser tx: %-32s%s" % (self.name, val, end))

  def read_serial(self):
    self.buf += [chr(c) for c in self.ser.read(self.ser.in_waiting)]

  def proc_buf(self):
    try:
      startline = 0
      endline = 0
      lines = []

      if len(self.buf) > self.buf_pointer:
        for i, c in enumerate(self.buf):
          if c == '\n' and self.buf[i - 1] == '\r':
            endline = i
            lines.append("".join(self.buf[startline:endline-1]))
            startline = endline + 1
            self.frame_count += 1
        
        self.buf = self.buf[endline+1:]
    except ValueError:
      pass

    self.buf_pointer = len(self.buf)
    return lines

  def run(self):
    self.write_serial("sys get ver", block=True)
    self.write_serial("radio get mod", block=True)
    self.write_serial("radio get freq", block=True)
    self.write_serial("radio get sf", block=True)
    self.write_serial("mac pause", block=True)
    self.write_serial("radio set pwr 10", block=True)
    self.write_serial("sys set pindig GPIO11 0", block=True)

    while not self.end_thread:
      self.read_serial()
      lines = self.proc_buf()
      
      if len(lines) > 0:
        #print(self.frame_count, lines)

        for line in lines:
          print(line)
        time.sleep(0.01)

    self.ser.close()
    print("[%s] closed serial port" % (self.name))