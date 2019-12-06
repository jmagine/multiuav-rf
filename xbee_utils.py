'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Dec 05 2019
                                      SARC

  File Name  : xbee_utils.py
  Description: Xbee test suite
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
#CHANNEL = 11
SC = "7FFF"
#plt.ion()
#plt.show()

class XBee_Controller():
  def __init__(self, device):
    self.device = device
    self.remote_device = None
    self.times = []

  def callback_dev_discovered(self, remote):
    print("[%s] device discovered: %s" % (PORT, remote))
    self.remote_device = remote

  def callback_disc_finished(self, status):
    if status == NetworkDiscoveryStatus.SUCCESS:
      print("[%s] Discovery process finished." % (PORT))

      if self.remote_device is None:
        print("[%s] No remote device found. Trying again..." % (PORT))
        self.device.get_network().start_discovery_process()
    else:
      print("[%s] There was an error discovering devices: %s" % (PORT, status.description))
  
  def print_channel(self):
    channel = int.from_bytes(self.device.get_parameter("CH"), byteorder='big', signed=False)
    channel_hex = bytes(self.device.get_parameter("CH")).hex()
    print("[%s] Channel: %d %s" % (PORT, channel, channel_hex))
  
  def set_channel(self, sc):
    print("[%s] Writing SC: %s" % (PORT, sc))
    self.device.set_parameter("SC", bytearray.fromhex(sc))

    #wait for channel to change
    channel = int.from_bytes(self.device.get_parameter("CH"), byteorder='big', signed=False)
    print("[%s] Waiting for channel selection" % (PORT))
    while channel == 0:
      self.print_channel()
      time.sleep(1)
      channel = int.from_bytes(self.device.get_parameter("CH"), byteorder='big', signed=False)
    

  def setup_connection(self):
    #confiure network
    xbee_network = self.device.get_network()
    xbee_network.set_discovery_timeout(25) #1000 sec timeout
    xbee_network.clear()

    xbee_network.add_device_discovered_callback(self.callback_dev_discovered)
    xbee_network.add_discovery_process_finished_callback(self.callback_disc_finished)
    xbee_network.start_discovery_process()

    #wait for device to be found
    print("[%s] discovering devices" % (PORT))
    while self.remote_device is None:
      channel = int.from_bytes(self.device.get_parameter("CH"), byteorder='big', signed=False)
      print("[%s] Channel: %d %s" % (PORT, channel, bytes(self.device.get_parameter("CH")).hex()))
      time.sleep(1)

    #once device is found, stop search
    if xbee_network.is_discovery_running():
      xbee_network.stop_discovery_process()
  
  def close_device(self):
    print("[%s] closing device" % (PORT))
    if self.device is not None and self.device.is_open():
      self.device.close()

  def run(self):
    try:
      #open connection and write params
      self.device.open()
      #self.set_channel(SC)

      #find remote device
      self.setup_connection()

      #accept user input
      cmd = ""
      while cmd != 'q':
        cmd = input("[cmd] [t] run tx [c] change channel [q] quit --- :")
        if len(cmd) > 0 and cmd[0] == 'q':
          break
        elif len(cmd) > 0 and cmd[0] == 't':
          self.tx()
          plt.plot(range(len(self.times)), self.times)
          #plt.show(block=False)
          plt.draw()
          plt.pause(0.001)
        elif len(cmd) > 0 and cmd[0] == 'c':
          arg = cmd.strip('\n').split(' ')[1]
          print("[cmd] change channel: %s" % (arg))
          self.remote_device = None
          self.set_channel(arg)
          self.setup_connection()
    except KeyboardInterrupt:
      print("[cmd] Ctrl+C received. Stopping")
    finally:
      self.close_device()
      print("[%s] device closed" % (PORT))

  def tx(self):
    self.times = []
    print("[%s] tx start" % (PORT))

    start = time.time()
    last = time.time()
    for i in range(NUM_TX):
      #print('[%d] TX' % (i))
      #device.send_data_async(self.remote_device, MSG)
      self.device.send_data(self.remote_device, MSG)
      #self.device.send_data(self.remote_device, str(time.time()))
      curr = time.time()
      self.times.append(curr - last)
      time.sleep(0.1)
      last = time.time()
      #time.sleep(0.01)

    end = time.time()
    print("successful. t: %.3f" % (end - start))

device = XBeeDevice(PORT, BAUD_RATE)
c = XBee_Controller(device)
c.run()