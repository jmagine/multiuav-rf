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
PORT = 'COM7'
BAUD = 57600

t_start = time.time()

try:
  lsc = lostik_utils.LS_Controller(PORT, baudrate=BAUD, prt=False)
  lsc.write_serial("sys get ver", block=True)

  lsc.write_serial("radio set sf sf7", block=True)
  lsc.write_serial("radio set bw 500", block=True)

  #lsc.write_serial("radio set sf sf12", block=True)
  #lsc.write_serial("radio set bw 500", block=True)
  lsc.write_serial("mac pause", block=True)
  lsc.write_serial("radio set pwr 2", block=True)
  lsc.write_serial("sys set pindig GPIO11 0", block=True)

  lsc.write_serial("radio get mod", block=True)
  lsc.write_serial("radio get freq", block=True)
  lsc.write_serial("radio get sf", block=True)
  lsc.write_serial("radio get bw", block=True)

  lsc.start()

  t_start = time.time()
  t_last = time.time()
  tx_count_last = 0
  rx_count_last = 0
  while True:
    lsc.tx(lsc.tx_count, block=True)
    lsc.rx(block=True)

    t_curr = time.time()
    if t_curr > t_last + 5:
      lsc.print_diagnostics(t_start, t_curr, t_last, tx_count_last, rx_count_last)
      t_last = time.time()
      tx_count_last = lsc.tx_count
      rx_count_last = lsc.rx_count

except KeyboardInterrupt:
  print("[main] Ctrl+C rx, stopping controller")

  #print diagnostics
  t_end = time.time()
  lsc.send_cmd("end_thread")
  lsc.print_diagnostics(t_start, t_end)
  lsc.join()