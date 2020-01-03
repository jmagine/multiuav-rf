'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Jan 02 2020
                                      SARC

  File Name  : lostik_rx.py
  Description: LoStik receiver module
---*-----------------------------------------------------------------------*'''

import time
import serial
import threading

import lostik_utils

#FREQ = 920e6
#FREQS = [902e6 + 1e6 * i for i in range(27)]
#PORT = '/dev/ttyUSB0'
PORT = 'COM3'
BAUD = 57600

try:
  lsc = lostik_utils.LS_Controller(PORT, BAUD, prt=True)
  lsc.start()
  lsc.write_serial("sys get ver", block=True)

  lsc.write_serial("radio set sf sf7", block=True)
  lsc.write_serial("radio set bw 500", block=True)
  lsc.write_serial("mac pause", block=True)
  lsc.write_serial("radio set pwr 10", block=True)
  lsc.write_serial("sys set pindig GPIO11 0", block=True)

  lsc.write_serial("radio get mod", block=True)
  lsc.write_serial("radio get freq", block=True)
  lsc.write_serial("radio get sf", block=True)
  lsc.write_serial("radio get bw", block=True)
  lsc.write_serial("radio rx 0", block=True)
  while True:
    lsc.write_serial("radio rx 0", block=True)
    time.sleep(0.01)

except KeyboardInterrupt:
  print("[main] Ctrl+C rx, stopping controller")
  lsc.send_cmd("end_thread")
  lsc.join()