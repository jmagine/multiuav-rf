'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Feb 15 2020
                                      SARC

  File Name  : env.py
  Description: Environment module for simulation
---*-----------------------------------------------------------------------*'''

import numpy as np
import math
import matplotlib.pyplot as plt
import random
from scipy.spatial import Voronoi, voronoi_plot_2d, KDTree
import time
import networkx as nx

import drone
import bs
import poi

#TODO keep track of where broadcasts are occuring
#TODO radio propagation model
#TODO point of interest model

N_DRONES = 32
N_POI = 100
P_BOUNDS = [[-1, 1], [-1, 1]]
VIEW_BORDER = 0.1

BANDWIDTH = 1.0

plt.ion()

class env():
  def __init__(self, n_drones=8, p_bounds=P_BOUNDS):
    self.p_bounds = p_bounds
    self.drn = []
    self.poi = []
    self.poi_active = []
    self.tx = {}
    self.bs = []
    self.g = nx.Graph()

    self.plot = True

    self.fig_vor = plt.figure(figsize=(8, 8), dpi=100)
    self.ax_vor = self.fig_vor.add_subplot(1,1,1)
    self.ax_vor.axis('equal')
    self.ax_vor.set_xlim([self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER])
    self.ax_vor.set_ylim([self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER])

    self.t = 0
    #self.net = 

  def setup(self):
    self.bs = [bs.base_station([0,0])] #set up for multiple base stations in future work
    for i, b in enumerate(self.bs):
      self.g.add_node(i, p=b.pos)

    #generate or load in situation, including drone positions, pois, freqs
    for i in range(N_DRONES):
      x = random.uniform(self.p_bounds[0][0], self.p_bounds[0][1])
      y = random.uniform(self.p_bounds[1][0], self.p_bounds[1][1])
      self.drn.append(drone.drone(i, [x, y], 1))
      self.g.add_node(len(self.bs) + i, p=self.drn[i].pos)

    for i in range(N_POI):
      self.poi.append(poi.poi([random.uniform(self.p_bounds[0][0], self.p_bounds[0][1]), random.uniform(self.p_bounds[1][0], self.p_bounds[1][1])], random.randint(0, 500), 500))

    #sort pois by start time
    self.poi.sort(key=lambda x: x.t_start)

  def plot_vor(self, vor):
    self.fig_vor.clf()
    ax = self.fig_vor.gca()
    
    # Plot drone points
    ax.scatter([d.pos[0] for d in self.drn], [d.pos[1] for d in self.drn], marker='x', color='b')
    #print("initial",vor.filtered_points)

    # Plot ridge points
    #for region in vor.filtered_regions:
    #    vertices = vor.vertices[region, :]
    #    ax.plot(vertices[:, 0], vertices[:, 1], 'go')

    # Plot ridges
    for region in vor.filtered_regions:
        vertices = vor.vertices[region + [region[0]], :]
        ax.plot(vertices[:, 0], vertices[:, 1], linewidth=1, linestyle='-', color='k')

    # Plot Centroids
    for region in vor.filtered_regions:
      ax.scatter(self.centroids[:, 0], self.centroids[:, 1], marker='.', color='r')

  def tick(self):
    t_start = time.time()
    #update positions of all drones
    for d in self.drn:
      d.tick()

    #evaluate capacity of network

    #evaluate data sent through network

    #check for poi expiration if applicable

    #check for any new pois
    while len(self.poi) > 0 and self.t >= self.poi[0].t_start:
      #print(self.poi[0].t_start, self.t)
      self.poi_active.append(self.poi[0])
      del self.poi[0] #TODO replace with something more efficient than this if slow

      #run assignment and network topology generation on new poi set
    
    #every few ticks, update voronoi partitioning
    if self.t % 5 == 0:
      vor_points = [d.pos for d in self.drn] + [b.pos for b in self.bs]
      vor, self.centroids = self.cvt(vor_points)
      self.centroid_kdt = KDTree(self.centroids)
      #plot the voronoi partitioning

      for d in self.drn:
        dist, idx = self.centroid_kdt.query(d.pos)
        d.set_pos_ref(self.centroid_kdt.data[idx])

      if self.plot:
        self.plot_vor(vor)
        plt.show()
        plt.pause(0.0001)

      print('[tick] t: %.3f' % (time.time() - t_start))

    #print(self.poi)
    #print(self.poi_active)
    self.t += 1

  def cvt(self, vor_points):

    t_start = time.time()
    #mirror the points across bounds so the bounds become voronoi edges
    points_center = vor_points
    points_left = np.copy(points_center)
    points_right = np.copy(points_center)
    points_down = np.copy(points_center)
    points_up = np.copy(points_center)

    points_left[:, 0] = self.p_bounds[0][0] - (points_left[:, 0] - self.p_bounds[0][0])
    points_right[:, 0] = self.p_bounds[0][1] + (self.p_bounds[0][1] - points_right[:, 0])
    points_down[:, 1] = self.p_bounds[1][0] - (points_down[:, 1] - self.p_bounds[1][0])
    points_up[:, 1] = self.p_bounds[1][1] + (self.p_bounds[1][1] - points_up[:, 1])

    points = np.append(points_center,
                       np.append(np.append(points_left,
                                           points_right,
                                           axis=0),
                                 np.append(points_down,
                                           points_up,
                                           axis=0),
                                 axis=0),
                       axis=0)

    vor = Voronoi(points)
    # Filter regions and select corresponding points
    regions = []
    points_to_filter = [] # we'll need to gather points too
    ind = np.arange(np.array(points).shape[0])
    ind = np.expand_dims(ind, axis= 1)

    for i,region in enumerate(vor.regions): # enumerate the regions
      if not region: # nicer to skip the empty region altogether
        continue

      flag = True
      for index in region:
        if index == -1:
          flag = False
          break
        else:
          x = vor.vertices[index, 0]
          y = vor.vertices[index, 1]
          if not(self.p_bounds[0][0] - 0.01 <= x and x <= self.p_bounds[0][1] + 0.01 and
                self.p_bounds[1][0] - 0.01 <= y and y <= self.p_bounds[1][1] + 0.01):
            #print("out of bound")
            flag = False
            break
      if flag:
        regions.append(region)

        # find the point which lies inside
        points_to_filter.append(vor.points[vor.point_region == i][0,:])

    vor.filtered_points = np.array([vor.point_region[:vor.npoints//5]])
    vor.filtered_regions = regions
    
    centroids = []
    for region in vor.filtered_regions:

      vertices = vor.vertices[region + [region[0]], :]

      A = 0
      C_x = 0
      C_y = 0

      for i in range(0, len(vertices) - 1):
        s = (vertices[i, 0] * vertices[i + 1, 1] - vertices[i + 1, 0] * vertices[i, 1])
        A += s
        C_x += (vertices[i, 0] + vertices[i + 1, 0]) * s
        C_y += (vertices[i, 1] + vertices[i + 1, 1]) * s

      A *= 0.5
      C_x *= (1.0 / (6.0 * A))
      C_y *= (1.0 / (6.0 * A))
      centroids.append([C_x, C_y])
    
    print("t: %d cvt t: %.3f" % (self.t, time.time() - t_start)) #, centroids, len(vor.filtered_regions))
    return vor, np.array(centroids)

  #evaluate cost function on a current snapshot of situation
  #calculate capacities here
  def eval_snap(self):
    pass

  #print diagnostics for current situation
  def print(self):
    print("[env] drones: %d" % (len(self.drn)))
    print("[env] poi_active: %d" % (len(self.poi_active)))
    print("[env] tx: %d" % (len(self.tx.keys())))

  def capacity(self, freq, pos_tx, pos_rx, pw_tx=20, noise=-90):
    noise_total = noise

    pw_rx = power_fspl(pw_tx, freq, dist(pos_tx, pos_rx))

    #cross interference
    if tuple(freq) in self.tx:
      for pos_noise in self.tx[tuple(freq)]:
        noise_total += power_fspl(pw_tx, freq, dist(pos_noise, pos_rx))

    return BANDWIDTH * math.log2(1 + pw_rx / noise_total)

def dist(pos_0, pos_1):
  return np.linalg.norm(np.array(pos_0) - np.array(pos_1))

#calculate received signal power in dB
def power_fspl(pw_tx, freq, dist):
  if dist <= 0 or freq <= 0:
    return pw_tx

  loss = 20 * math.log10(4 * math.pi / 300 * freq * dist)
  return pw_tx - loss