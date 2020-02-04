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

N_BYTES = 64

class LS_Controller(threading.Thread):
  def __init__(self, port, baudrate=57600, prt=True):
    super(LS_Controller, self).__init__()

    #TODO hashtable for last seen freqs
    
    #threading/control params
    self.daemon = True
    self.end_thread = False

    #for serial cmds to/from LoStik
    self.ser = serial.Serial(port, baudrate=baudrate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
    self.ser.reset_input_buffer()
    self.ser.reset_output_buffer()
    self.name = self.ser.name

    #custom serial buffer
    self.ser_rx = []
    self.buf = []
    self.tx_count = 0
    self.rx_count = 0
    self.wait_rx = threading.Event()
    self.wait_tx = threading.Event()

    #other
    self.print = prt

    print("[%s] connected" % (self.name))

  #send cmd from main thread to this one
  def send_cmd(self, cmd):
    if cmd == 'end_thread':
      print("[%s] stopping lostik controller" % (self.name))
      self.end_thread = True

  #write a command to RN2903 through serial link
  def write_serial(self, cmd, block=False):
    val = ('%s\r\n' % (cmd)).encode('utf-8')
    end = ""

    self.ser.write(val)
    self.ser.flush()

    if block:
      while len(self.ser_rx) == 0:
        self.read_serial()
        time.sleep(0.01)
        
      end = "\t--- ser rx: %s" % (self.ser_rx[0])
      del self.ser_rx[0]

    if self.print:
      print("[%s] ser tx: %-32s%s" % (self.name, cmd, end))

  #read from serial if there are new things to read
  def read_serial(self):

    #t_start = time.time()
    #if nothing to read, just continue
    if self.ser.in_waiting == 0:
      return

    self.buf += [chr(c) for c in self.ser.read(self.ser.in_waiting)]

    #piece outputs together into lines
    try:
      startline = 0
      endline = 0
      lines = []

      for i, c in enumerate(self.buf):
        if c == '\n' and self.buf[i - 1] == '\r':
          endline = i
          lines.append("".join(self.buf[startline:endline-1]))
          startline = endline + 1
          #self.rx_count += 1
      if endline != 0:
        self.buf = self.buf[endline+1:]
    except ValueError:
      #print("value error")
      pass

    if lines:
      self.ser_rx.extend(lines)
    
    #print("[read_serial] profile: %.4f" % (time.time() - t_start))

  #transmit a message through radio
  def tx(self, msg, block=True):
    
    t_start = time.time()

    self.write_serial("radio tx %.64x" % (int(msg)), block=False)
    self.tx_count += 1

    while block:
      self.read_serial()
      if len(self.ser_rx) > 0:
        line = self.ser_rx[0]

        if line == 'radio_tx_ok':
          block = False

        del self.ser_rx[0]

    print("[tx] profile: %.4f" % (time.time() - t_start))

  #wait until message is received
  def rx(self, block=True):
    self.write_serial("radio rx 0", block=False)

    while block:
      self.read_serial()
      if len(self.ser_rx) > 0:
        line = self.ser_rx[0]

        if line == 'busy' or line == 'radio_err':
          del self.ser_rx[0]
          self.write_serial("radio rx 0", block=False)
          continue

        if line == 'ok':
          del self.ser_rx[0]
          continue

        self.rx_count += 1
        print("[rx] %s" % (line))
        block = False
        del self.ser_rx[0]

    #TODO update hashtable if correct packet rxed

  #run thread, command loop
  def run(self):
    while not self.end_thread:
      time.sleep(1)
      
      '''
      self.read_serial()
      
      if len(self.ser_rx) > 0:
        line = self.ser_rx[0]
        if self.print: 
          print(line)

        if line == 'radio_tx_ok':
          self.wait_tx.set()
          print("tx %d" % (self.tx_count))
          self.tx_count += 1

        elif line == 'err' or line == 'radio_err':
          self.wait_tx.set()
          self.wait_rx.set()
          #pass
        elif line == 'busy' or line == 'invalid_param':
          pass
        elif len(line.split(" ")) > 1:
          if line.split(" ")[0] == 'radio_rx':
            if self.print:
              print("[RX] %s" % (line.split(" ")[-1]))
            print("rx %d" % (self.rx_count))
            #TODO do something with the payload, put it somewhere
            self.rx_count += 1
            self.wait_rx.set()

        del self.ser_rx[0]
        '''

    self.ser.close()
    print("[%s] closed serial port" % (self.name))

  #print tx link characteristics
  def print_diagnostics(self, t_start, t_end, t_last=None, tx_count_last=0, rx_count_last=0):
    if t_start == t_end:
      return

    if t_last is None:
      t_last = t_start
      tx_count_last = 0
      rx_count_last = 0

    print("[diag] t: %.4f tx c: %-4d tx_kbps: %.4f tx_kbps_total: %.4f rx_c: %-4d rx_kbps: %.4f rx_kbps_total: %.4f " % (t_end - t_start, 
                       self.tx_count, 
                       (self.tx_count - tx_count_last) * N_BYTES / ((t_end - t_last) * 1000),
                       self.tx_count * N_BYTES / ((t_end - t_start) * 1000),

                       self.rx_count,
                       (self.rx_count - rx_count_last) * N_BYTES / ((t_end - t_last) * 1000),
                       self.rx_count * N_BYTES / ((t_end - t_start) * 1000))
                       )
    sys.stdout.flush()