'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Jan 02 2020
                                      SARC

  File Name  : lostik_tx.py
  Description: LoStik transmitter module
---*-----------------------------------------------------------------------*'''

import time
import serial
import threading

import lostik_utils

#FREQ = 920e6
freqs = [902e6 + 1e6 * i for i in range(27)]
PORT = '/dev/ttyUSB0'
BAUD = 57600

try:
  lsc = lostik_utils.LS_Controller(PORT, BAUD)
  #lsc.start()
  lsc.join()
except KeyboardInterrupt:
  print("[main] Ctrl+C rx, stopping controller")
  lsc.send_cmd("end_thread")
  lsc.join()