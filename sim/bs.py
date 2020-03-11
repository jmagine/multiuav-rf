'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Mar 05 2020
                                      SARC

  File Name  : bs.py
  Description: Base station module for simulation
---*-----------------------------------------------------------------------*'''

import numpy as np

class base_station():
  def __init__(self, p):
    self.pos = np.array(p)