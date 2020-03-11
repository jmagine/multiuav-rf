'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Mar 05 2020
                                      SARC

  File Name  : poi.py
  Description: Point of interest module for simulation
---*-----------------------------------------------------------------------*'''

import numpy as np

class poi():
  def __init__(self, p, t_start, t_window):
    self.pos = np.array(p)
    self.t_start = t_start
    self.t_window = t_window
    self.data = 100