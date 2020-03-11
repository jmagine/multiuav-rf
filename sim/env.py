'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Feb 15 2020
                                      SARC

  File Name  : env.py
  Description: Environment module for simulation
---*-----------------------------------------------------------------------*'''

import numpy as np
import matplotlib.pyplot as plt
import random
from scipy.spatial import Voronoi, voronoi_plot_2d

import drone
import bs
import poi

#TODO keep track of where broadcasts are occuring
#TODO radio propagation model
#TODO point of interest model

N_DRONES = 8
N_POI = 10

class env():
  def __init__(self, n_drones=8, reg_bounds=[]):
    self.reg_bounds = []
    self.drn = []
    self.poi = []
    self.poi_active = []
    self.tx = {}
    self.bs = []

    self.t = 0
    #self.net = 

  def setup(self):
    #generate or load in situation, including drone positions, pois, freqs
    for i in range(N_DRONES):
      x = random.uniform(-1, 1)
      y = random.uniform(-1, 1)
      self.drn.append(drone.drone(i, [x, y], [x, y]))

    for i in range(N_POI):
      self.poi.append(poi.poi([random.uniform(-1, 1), random.uniform(-1, 1)], random.randint(0, 50), 50))

    self.bs = [bs.base_station([0,0])] #set up for multiple base stations in future work

    #sort pois by start time
    self.poi.sort(key=lambda x: x.t_start)

  def tick(self):
    #update positions of all drones
    for d in self.drn:
      d.tick()

    #evaluate capacity of network

    #evaluate data sent through network

    #check for poi expiration #TODO actually maybe don't need to do this

    #check for any new pois
    while len(self.poi) > 0 and self.t >= self.poi[0].t_start:
      print(self.poi[0].t_start, self.t)
      self.poi_active.append(self.poi[0])
      del self.poi[0] #TODO replace with something more efficient than this if slow

      #run assignment and network topology generation on new poi set
      vor = Voronoi([d.pos for d in self.drn] + [b.pos for b in self.bs])
      voronoi_plot_2d(vor)
      plt.show()
    #print(self.poi)
    #print(self.poi_active)
    self.t += 1

  #evaluate cost function on a current snapshot of situation
  def eval_snap(self):
    pass

  #draw and save a figure of the current situation
  def plot(self):
    pass

  #print diagnostics for current situation
  def print(self):
    print("[env] drones: %d" % (len(self.drn)))
    print("[env] poi_active: %d" % (len(self.poi_active)))
    print("[env] tx: %d" % (len(self.tx.keys())))

  

