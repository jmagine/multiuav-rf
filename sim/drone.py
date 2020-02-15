'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Feb 15 2020
                                      SARC

  File Name  : drone.py
  Description: Drone module for simulation
---*-----------------------------------------------------------------------*'''

import numpy as np

#TODO use comm model
#TODO use physics
#TODO implement PID control

class drone():
  def __init__(self):
    self.pos = np.zeros((2))
    self.vel = np.zeros((2))
    self.acc = np.zeros((2))

    self.rx_freq = 902e6

  

