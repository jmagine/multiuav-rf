'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Feb 15 2020
                                      SARC

  File Name  : env.py
  Description: Environment module for simulation
---*-----------------------------------------------------------------------*'''

import math
import numpy as np
import random
from scipy.spatial import Voronoi, voronoi_plot_2d, KDTree
import sys
import time

import drone

import utils
import vis

#TODO keep track of where broadcasts are occuring
#TODO radio propagation model
#TODO point of interest model

BANDWIDTH = 1.0

class env():
  def __init__(self, n_drones, p_bounds, M, F, v_max):
    self.p_bounds = p_bounds
    self.n_drones = n_drones
    self.M = M
    self.F = F
    self.v_max = v_max
    
    self.drn = []
    #self.poi = []
    #self.poi_active = []
    self.tx = {}
    self.bs = []
    self.t = 0

  def setup(self):
    #self.bs = [bs.base_station([0,0])] #set up for multiple base stations in future work

    #generate or load in situation, including drone positions, pois, freqs
    '''
    for i in range(self.n_drones):
      x = random.uniform(self.p_bounds[0][0], self.p_bounds[0][1])
      y = random.uniform(self.p_bounds[1][0], self.p_bounds[1][1])
      self.drn.append(drone.drone(i, [x, y], 1))
      #self.g.add_node(len(self.bs) + i, p=self.drn[i].pos)
    '''

    #for i in range(N_POI):
    #  self.poi.append(poi.poi([random.uniform(self.p_bounds[0][0], self.p_bounds[0][1]), random.uniform(self.p_bounds[1][0], self.p_bounds[1][1])], random.randint(0, 500), 500))

    #sort pois by start time
    #self.poi.sort(key=lambda x: x.t_start)

    random.seed(1)

    self.gt = np.zeros((self.n_drones, 2))
    '''
    self.gt[0][0] = -200
    self.gt[0][1] = -200

    self.gt[1][0] = 100
    self.gt[1][1] = -100

    self.gt[2][0] = -100
    self.gt[2][1] = 100

    self.gt[3][0] = 100
    self.gt[3][1] = 100
    '''

    #'''
    for i in range(self.n_drones):
      #self.gt[i][0] = random.uniform(self.p_bounds[0][0], self.p_bounds[0][1])
      #self.gt[i][1] = random.uniform(self.p_bounds[1][0], self.p_bounds[1][1])
      self.gt[i][0] = np.clip(random.gauss(0, 150), self.p_bounds[0][0], self.p_bounds[0][1])
      self.gt[i][1] = np.clip(random.gauss(0, 150), self.p_bounds[1][0], self.p_bounds[1][1])

    #for k in range(self.n_drones):
    #  print("\\addplot[color=green,mark=square] coordinates{(%.2f,%.2f)};" % (self.gt[k][0], self.gt[k][1]))


    #'''
    #drone trajectory init
    self.init_q = np.zeros((self.n_drones, self.M, 2))
    self.init_p = np.zeros((self.n_drones, self.M))
    
    '''
    self.init_q[0][0][0] = 450
    self.init_q[0][0][1] = 450
    self.init_q[1][0][0] = -450
    self.init_q[1][0][1] = 450
    self.init_q[2][0][0] = 450
    self.init_q[2][0][1] = -450
    self.init_q[3][0][0] = -450
    self.init_q[3][0][1] = -450
    '''

    for i in range(self.n_drones):
      self.init_q[i][0][0] = random.uniform(self.p_bounds[0][0], self.p_bounds[0][1])
      self.init_q[i][0][1] = random.uniform(self.p_bounds[1][0], self.p_bounds[1][1])

      dist = utils.dist(self.gt[i], self.init_q[i][0])

      x_step = (self.gt[i][0] - self.init_q[i][0][0]) * self.v_max / dist
      y_step = (self.gt[i][1] - self.init_q[i][0][1]) * self.v_max / dist
      for n in range(self.M):
        if n < dist / self.v_max:
          self.init_q[i][n][0] = self.init_q[i][0][0] + x_step * n
          self.init_q[i][n][1] = self.init_q[i][0][1] + y_step * n
        else:
          self.init_q[i][n][0] = self.gt[i][0]
          self.init_q[i][n][1] = self.gt[i][1]

    #drone power init
    for n in range(self.M):
      for k in range(self.n_drones):
        self.init_p[k][n] = 100

        '''
        dist = utils.dist(self.init_q[k][n], self.gt[k])

        if dist > 0:
          self.init_p[k][n] = min(1, 1.0 / dist)
        else:
          self.init_p[k][n] = 1
        '''

    #print(self.init_p, self.init_q)
    print(self.gt)

  def tick(self):
    t_start = time.time()
    #update positions of all drones
    for d in self.drn:
      d.tick()

    #every few ticks, print heartbeat
    if self.t % 5 == 0:
      print('[tick] t: %.3f' % (time.time() - t_start))

    self.t += 1

    '''
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

      plotter.plot_vor(self.drn, self.centroids, vor)

      print('[tick] t: %.3f' % (time.time() - t_start))

    #print(self.poi)
    #print(self.poi_active)
    '''

  #TODO move to utils
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

  #print diagnostics for current situation
  def print(self):
    print("[env] drones: %d" % (len(self.drn)))

  def capacity(self, freq, pos_tx, pos_rx, pw_tx=20, noise=-90):
    noise_total = noise

    pw_rx = utils.power_fspl(pw_tx, freq, utils.dist(pos_tx, pos_rx))

    #cross interference
    if tuple(freq) in self.tx:
      for pos_noise in self.tx[tuple(freq)]:
        noise_total += utils.power_fspl(pw_tx, freq, utils.dist(pos_noise, pos_rx))

    return BANDWIDTH * math.log2(1 + pw_rx / noise_total)