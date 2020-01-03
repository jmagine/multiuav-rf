'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Jan 02 2020
                                      SARC

  File Name  : lostik_utils.py
  Description: LoStik controller for packaged usage
---*-----------------------------------------------------------------------*'''

import serial
import sys
import threading
import time

N_BYTES = 255

class LS_Controller(threading.Thread):
  def __init__(self, port, baudrate=57600, prt=True):
    super(LS_Controller, self).__init__()
    self.buf = []
    self.buf_pointer = 0
    self.tx_count = 0
    self.rx_count = 0
    self.daemon = True
    self.end_thread = False
    self.print = prt

    self.ser = serial.Serial(port, baudrate=baudrate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
    self.ser.reset_input_buffer()
    self.ser.reset_output_buffer()
    self.name = self.ser.name

    print("[%s] connected" % (self.name))

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
    if self.print: print("[%s] ser tx: %-32s%s" % (self.name, val, end))

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
            self.rx_count += 1
        if endline != 0:
          self.buf = self.buf[endline+1:]
    except ValueError:
      pass

    self.buf_pointer = len(self.buf)
    return lines

  def tx(self, msg, block=True):
    self.write_serial("radio tx %.255x" % (int(msg)), block=block)

  def run(self):
    while not self.end_thread:
      self.read_serial()
      lines = self.proc_buf()
      
      if len(lines) > 0:
        #print(self.frame_count, lines)
        for line in lines:
          if self.print: print(line)
          if line == 'radio_tx_ok':
            self.tx_count += 1

      time.sleep(0.001)

    self.ser.close()
    print("[%s] closed serial port" % (self.name))

  def print_diagnostics(self, t_start, t_end):
    if t_start == t_end:
      return

    print("[diag]\ttx c: %-4d tx_byte: %-6d tx t: %.4f tx_kbps: %.4f" % (self.tx_count, 
                       self.tx_count * N_BYTES, 
                       t_end - t_start, 
                       self.tx_count * N_BYTES / ((t_end - t_start) * 1000)))
    sys.stdout.flush()