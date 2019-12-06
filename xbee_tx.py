'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Dec 05 2019
                                      SARC

  File Name  : xbee_tx.py
  Description: Xbee transmitter
---*-----------------------------------------------------------------------*'''

from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.devices import XBeeDevice

import matplotlib.pyplot as plt
import sys
import time

MSG = ''
MSG = MSG.join(['a' for i in range(92)])

MAX_LEN = 92
PORT = 'COM5'
BAUD_RATE = 921600
REMOTE_NODE_ID = 'xb1'
NUM_TX = 100

remote_device = None

def callback_dev_discovered(remote):
  global remote_device
  print("device discovered: %s" % (remote))
  remote_device = remote

def callback_disc_finished(status):
  if status == NetworkDiscoveryStatus.SUCCESS:
    print("Discovery process finished successfully.")
  else:
    print("There was an error discovering devices: %s" % (status.description))

print(len(MSG))
print(MSG)
device = XBeeDevice(PORT, BAUD_RATE)

try:
  device.open()
  xbee_network = device.get_network()
  xbee_network.set_discovery_timeout(25) #1000 sec timeout
  xbee_network.clear()

  xbee_network.add_device_discovered_callback(callback_dev_discovered)
  xbee_network.add_discovery_process_finished_callback(callback_disc_finished)
  xbee_network.start_discovery_process()

  print("[%s] discovering devices" % (PORT))

  while remote_device is None:
    time.sleep(0.1)

  times = []
  print("[%s] tx start" % (PORT))

  start = time.time()
  last = time.time()
  for i in range(NUM_TX):
    #print('[%d] TX' % (i))
    #device.send_data_async(remote_device, MSG)
    device.send_data(remote_device, MSG)
    curr = time.time()
    times.append(curr - last)
    last = curr

  end = time.time()
  print("successful. t: %.3f" % (end - start))

  plt.plot(range(len(times)), times)
  plt.show()

except KeyboardInterrupt:
  print("[%s] closing device" % (PORT))
  if device is not None and device.is_open():
    device.close()
finally:
  print("[%s] closing device" % (PORT))
  if device is not None and device.is_open():
    device.close()