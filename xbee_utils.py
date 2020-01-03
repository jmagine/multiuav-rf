'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Dec 05 2019
                                      SARC

  File Name  : xbee_utils.py
  Description: Xbee controller for packaged usage
---*-----------------------------------------------------------------------*'''

from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.devices import XBeeDevice

import matplotlib.pyplot as plt
import sys
import time

class XBee_Controller():
  def __init__(self, device, port):
    self.device = device
    self.remote_device = None
    self.times = []
    self.port = port
    self.rx_frame = 0

  def callback_dev_discovered(self, remote):
    print("[%s] device discovered: %s" % (self.port, remote))
    self.remote_device = remote

  def callback_disc_finished(self, status):
    if status == NetworkDiscoveryStatus.SUCCESS:
      print("[%s] Discovery process finished." % (self.port))

      if self.remote_device is None:
        print("[%s] No remote device found. Trying again..." % (self.port))
        self.device.get_network().start_discovery_process()
    else:
      print("[%s] There was an error discovering devices: %s" % (self.port, status.description))
  
  def callback_msg_rx(self, msg):
    rx_data = msg.data.decode(encoding="utf-8")
    print("[%s] rx: frame: %d len: %d [%s] %s" % (self.port, self.rx_frame, len(rx_data), msg.remote_device, rx_data))
    self.rx_frame += 1

  def print_channel(self):
    channel = int.from_bytes(self.device.get_parameter("CH"), byteorder='big', signed=False)
    channel_hex = bytes(self.device.get_parameter("CH")).hex()
    print("[%s] Channel: %d %s" % (self.port, channel, channel_hex))
  
  def set_channel(self, sc):
    print("[%s] Writing SC: %s" % (self.port, sc))
    self.device.get_network().clear()
    self.device.set_parameter("SC", bytearray.fromhex(sc))

    #wait for channel to change
    channel = int.from_bytes(self.device.get_parameter("CH"), byteorder='big', signed=False)
    print("[%s] Waiting for channel selection" % (self.port))
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
    print("[%s] discovering devices" % (self.port))
    while self.remote_device is None:
      channel = int.from_bytes(self.device.get_parameter("CH"), byteorder='big', signed=False)
      print("[%s] Channel: %d %s" % (self.port, channel, bytes(self.device.get_parameter("CH")).hex()))
      time.sleep(1)

    #once device is found, stop search
    if xbee_network.is_discovery_running():
      xbee_network.stop_discovery_process()

    self.device.add_data_received_callback(self.callback_msg_rx)
  
  def close_device(self):
    print("[%s] closing device" % (self.port))
    if self.device is not None and self.device.is_open():
      self.device.close()

  def tx(self, data, num_tx=100):
    self.times = []
    print("[%s] tx start - num_tx: %d msg: %s" % (self.port, num_tx, str(data)))

    start = time.time()
    last = time.time()
    for i in range(num_tx):
      #print('[%d] TX' % (i))
      self.device.send_data(self.remote_device, str(data))
      curr = time.time()
      self.times.append(curr - last)
      self.print_channel()
      last = time.time()

    end = time.time()
    print("tx completed. t: %.3f" % (end - start))