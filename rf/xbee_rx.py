'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Dec 05 2019
                                      SARC

  File Name  : xbee_rx.py
  Description: Xbee receiver
---*-----------------------------------------------------------------------*'''

from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.devices import XBeeDevice

import matplotlib.pyplot as plt
import sys
import time

import xbee_utils

PORT = "COM6"
BAUD_RATE = 921600
SC = "7FFF"

device = XBeeDevice(PORT, BAUD_RATE)
c = xbee_utils.XBee_Controller(device, PORT)
try:
  #open connection and write params
  c.device.open()
  #self.set_channel(SC)

  #find remote device
  c.setup_connection()

  #accept user input
  cmd = ''
  while cmd != 'q':
    cmd = input("[cmd] [c] change channel [q] quit --- :")
    if len(cmd) > 0 and cmd[0] == 'q':
      break
    elif len(cmd) > 0 and cmd[0] == 'c':
      arg = cmd.strip('\n').split(' ')[1]
      print("[cmd] change channel: %s" % (arg))
      c.remote_device = None
      c.set_channel(arg)
      c.setup_connection()
except KeyboardInterrupt:
  print("[cmd] Ctrl+C received. Stopping")
finally:
  c.close_device()