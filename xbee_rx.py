'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Dec 05 2019
                                      SARC

  File Name  : xbee_rx.py
  Description: XBee receiver
---*-----------------------------------------------------------------------*'''

from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.devices import XBeeDevice
import sys
import time

MSG = ''
MSG = MSG.join(['a' for i in range(92)])

MAX_LEN = 92
PORT = 'COM6'
BAUD_RATE = 921600
REMOTE_NODE_ID = 'xb0'
NUM_TX = 10

def callback_dev_discovered(remote):
  print("device discovered: %s" % (remote))

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
  xbee_network.set_discovery_timeout(15) #15 sec timeout
  xbee_network.clear()

  xbee_network.add_device_discovered_callback(callback_dev_discovered)
  xbee_network.add_discovery_process_finished_callback(callback_disc_finished)
  xbee_network.start_discovery_process()

  print("[%s] discovering devices" % (PORT))

  while xbee_network.is_discovery_running():
    time.sleep(0.1)

  #remote_device = xbee_network.discover_device(REMOTE_NODE_ID)

  #if remote_device is None:
  #  print('could not find remote device')
  #  sys.exit(1)

  #for i in range(NUM_TX):
  #  print('[%d] TX' % (i))
  #  device.send_data_async(remote_device, MSG)

finally:
  if device is not None and device.is_open():
    device.close()