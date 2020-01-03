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

PORT = '/dev/ttyUSB0'
#PORT = 'COM4'

BAUD = 57600

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

  tx_start = time.time()
  while True:
    lsc.tx(time.time(), block=False)
    time.sleep(0.001)
    
    if lsc.tx_count % 100 == 0:
      tx_end = time.time()
      if tx_end - tx_start == 0: continue

      tx_count = lsc.tx_count
      print("[main] tx count: %d" % (tx_count))
      print("[main] tx bytes: %d" % (tx_count * 8))
      print("[main] tx time: %f" % (tx_end - tx_start))
      print("[main] tx kbps: %f" % (tx_count * 8 / ((tx_end - tx_start) * 1000)))

except KeyboardInterrupt:
  print("[main] Ctrl+C rx, stopping controller")

  #print diagnostics
  tx_end = time.time()
  lsc.send_cmd("end_thread")
  tx_count = lsc.tx_count
  lsc.join()

  print("[main] tx count: %d" % (tx_count))
  print("[main] tx bytes: %d" % (tx_count * 8))
  print("[main] tx time: %f" % (tx_end - tx_start))
  print("[main] tx kbps: %f" % (tx_count * 8 / ((tx_end - tx_start) * 1000)))