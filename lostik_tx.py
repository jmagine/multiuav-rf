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
#FREQS = [902e6 + 1e6 * i for i in range(27)]

#PORT = '/dev/ttyUSB0'
PORT = 'COM4'
BAUD = 57600
BIG_PACKET = time.time()

tx_start = time.time()

try:
  lsc = lostik_utils.LS_Controller(PORT, baudrate=BAUD, prt=False)
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
  #lsc.write_serial("radio rx 0", block=True)
  tx_start = time.time()
  while True:
    lsc.tx(BIG_PACKET, block=False)
    time.sleep(0.01)
    
    if lsc.tx_count % 100 == 0:
      tx_curr = time.time()
      lsc.print_diagnostics(tx_start, tx_curr)

except KeyboardInterrupt:
  print("[main] Ctrl+C rx, stopping controller")

  #print diagnostics
  tx_end = time.time()
  lsc.send_cmd("end_thread")
  tx_count = lsc.tx_count
  lsc.print_diagnostics(tx_start, tx_end)
  lsc.join()
