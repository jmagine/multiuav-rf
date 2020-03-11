'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Feb 15 2020
                                      SARC

  File Name  : drone.py
  Description: Drone movement module
---*-----------------------------------------------------------------------*'''

import numpy as np

#TODO use comm model? maybe in env
#TODO use physics? 

class drone():
  def __init__(self, idx, p, v_max):
    self.idx = idx
    self.pos = np.array(p)
    self.pos_ref = np.array(p)
    self.vel = 0
    self.vel_max = v_max
    self.ttt = 0
  
  def callback(self):
    pass

  #updates pos_ref and the corresponding velocity vector
  def set_pos_ref(self, p):
    self.pos_ref = np.array(p)

    #compute direction vector if pos_ref changes
    vel = self.pos_ref - self.pos

    mag = np.linalg.norm(vel)
    self.vel = vel * self.vel_max / mag

    #ticks to target
    self.ttt = mag // self.vel_max

  def tick(self):
    #move according to max velocity model
    self.ttt -= 1

    if self.ttt < 0:
      self.pos = self.pos_ref
    else:
      self.pos += self.vel
    
    #print("[d][%d] p: %s" % (self.idx, self.pos))
