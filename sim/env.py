'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Feb 15 2020
                                      SARC

  File Name  : env.py
  Description: Environment module for simulation
---*-----------------------------------------------------------------------*'''

import math
import matplotlib.pyplot as plt
import mosek.fusion as mf
import networkx as nx
import numpy as np
import random
from scipy.spatial import Voronoi, voronoi_plot_2d, KDTree
import sys
import time

import drone
import bs
import poi

#TODO keep track of where broadcasts are occuring
#TODO radio propagation model
#TODO point of interest model

N_DRONES = 4
M = 10
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

    self.plot_vor_bool = False
    self.plot_traj_bool = True

    if self.plot_vor_bool:
      self.fig_vor = plt.figure(figsize=(8, 8), dpi=100)
      self.ax_vor = self.fig_vor.add_subplot(1,1,1)
      self.ax_vor.axis('equal')
      self.ax_vor.set_xlim([self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER])
      self.ax_vor.set_ylim([self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER])

    if self.plot_traj_bool:
      self.fig_traj = plt.figure(figsize=(8, 8), dpi=100)
      self.ax_traj = self.fig_traj.add_subplot(1,1,1)
      self.ax_traj.axis('equal')
      self.ax_traj.set_xlim([self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER])
      self.ax_traj.set_ylim([self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER])

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

    self.gt = np.zeros((N_DRONES, 2))
    for i in range(N_DRONES):
      self.gt[i][0] = random.uniform(self.p_bounds[0][0], self.p_bounds[0][1])
      self.gt[i][1] = random.uniform(self.p_bounds[1][0], self.p_bounds[1][1])

    F = [1 for x in range(N_DRONES)]
    #TODO move this somewhere more appropriate later on
    #self.ma_sca(len(self.drn), M, self.gt, F)
    self.shen_sca(len(self.drn), M, self.gt)

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

  def plot_traj(self, q, gt):
    self.fig_traj.clf()
    ax = self.fig_traj.gca()
    
    # Plot drone points
    for k in range(len(q)):
      #print(q[k, :, 0], q[k, :, 1])
      ax.plot(q[k, :, 0], q[k, :, 1], marker='x', ms=3, color='C%d' % (k))
      ax.scatter(gt[k][0], gt[k][1], marker='.',s=64, color='C%d' % (k))

    ax.set_xlim([self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER])
    ax.set_ylim([self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER])

    #for k in range(len(gt)):
    #  ax.scatter(gt[k])
    ax.set_aspect('equal', 'box')
    #self.ax_traj.set_xlim([self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER])
    #self.ax_traj.set_ylim([self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER])

    plt.show()
    plt.pause(0.0001)

    #print(self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER, self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER)
    #print(self.ax_traj.get_ylim(), self.ax_traj.get_xlim())

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

      if self.plot_vor_bool:
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

  def shen_sca(self, K, M, GT):

    p = np.zeros((K, M))
    q = np.zeros((K, M, 2))
    a = np.zeros((K, M))
    d = np.zeros((K, K, M))
    I = np.zeros((K, M))

    for i in range(N_DRONES):
      q[i][0][0] = random.uniform(self.p_bounds[0][0], self.p_bounds[0][1])
      q[i][0][1] = random.uniform(self.p_bounds[1][0], self.p_bounds[1][1])

      x_step = (self.gt[i][0] - q[i][0][0]) / M
      y_step = (self.gt[i][1] - q[i][0][1]) / M
      for n in range(M):
        q[i][n][0] = q[i][0][0] + x_step * n
        q[i][n][1] = q[i][0][1] + y_step * n

    for n in range(M):
      for k in range(K):
        p[k][n] = 1

    gt = GT
    gamma = 0.01

    eps = 0.01 #iteration tolerance
    print('[shen] init SCA: k: %d m: %d gt: %s' % (K, M, GT))
    
    
    #init trajectory q and power p foor all k and n
    #store 2 sets of q and p for previous and current iteration
    #TODO

    #determine initial R given p and q
    for n in range(M):
      for k in range(K):
        a[k][n] = math.sqrt(p[k][n])

        I[k][n] = 0
        for j in range(K):
          d[j][k][n] = dist(q[j][n], gt[k])**2 + 0.1
          I[k][n] += gamma * a[j][n]**2 / d[j][k][n]
        I[k][n] -= gamma * a[k][n]**2 / d[k][k][n]
    shen_r = shen_rpq(a, q, d, I, gt, gamma)
    print("[shen] R: %.2f" % (shen_r))

    for k in range(K):
        for n in range(M):
          print("[shen] q:[%.2f %.2f] a:%.2f" % (q[k][n][0], q[k][n][1], a[k][n]))
        print()

    itr = 0
    while True:
      var_a, var_q = optimize_shen(a, q, p, d, I, gt, gamma)
      
      #update a, q, p
      for idx in range(K * M):
        a[idx // M][idx % M] = var_a.level()[idx]
        p[idx // M][idx % M] = a[idx // M][idx % M]**2

        q[idx // M][idx % M][0] = var_q.level()[idx * 2]
        q[idx // M][idx % M][1] = var_q.level()[idx * 2 + 1]
      
      #update a, d, I with p, q
      for n in range(M):
        for k in range(K):
          a[k][n] = math.sqrt(p[k][n])
          
          I[k][n] = 0
          for j in range(K):
            d[j][k][n] = dist(q[j][n], gt[k])**2 + 0.1
            I[k][n] += gamma * a[j][n]**2 / d[j][k][n]
          I[k][n] -= gamma * a[k][n]**2 / d[k][k][n]

      #update R by eq 4 using shen_rpq
      shen_r_new = shen_rpq(a, q, d, I, gt, gamma)
      print("[shen] R: %.2f" % (shen_r_new))
      
      self.plot_traj(q, gt)
      for k in range(K):
        for n in range(M):
          print("[shen] q:[%.2f %.2f] a:%.2f" % (q[k][n][0], q[k][n][1], a[k][n]))

      #continue if prev r was 0
      if shen_r == 0: 
        shen_r = shen_r_new
        continue

      #termination condition
      '''
      if ((shen_r_new - shen_r) / (shen_r)) < eps:
        print("term")
        time.sleep(30)
        break
      '''

      #update iteration
      shen_r = shen_r_new

      itr += 1
      if itr % 5 == 0:
        pass

    p = np.multiply(a, a)
    return p, q

  def ma_sca(self, K, M, GT, F):

    p = np.zeros((K, M))
    q = np.zeros((K, M, 2))
    a = np.zeros((K, M))
    d = np.zeros((K, K, M))
    I = np.zeros((K, M))

    for n in range(M):
      for k in range(K):
        p[k][n] = 1
      
      #q[0][n][0] = -0.1 * n
      #q[0][n][1] = -0.1 * n

      #q[1][n][0] = 0.1 * n
      #q[1][n][1] = -0.1 * n

      #q[2][n][0] = -0.1 * n
      #q[2][n][1] = 0.1 * n

      #q[3][n][0] = 0.1 * n
      #q[3][n][1] = 0.1 * n

      q[0][n][0] = -0.1
      q[0][n][1] = -0.1
      q[1][n][0] = 0.1
      q[1][n][1] = -0.1

      q[2][n][0] = -0.1
      q[2][n][1] = 0.1
      q[3][n][0] = 0.1
      q[3][n][1] = 0.1

      '''
      q[2][n][0] = 0.7
      q[2][n][1] = 0.3
      q[3][n][0] = 0.7
      q[3][n][1] = 0.7
      '''

    gt = GT
    gamma = 0.1

    eps = 0.01 #iteration tolerance
    print('[shen] init SCA: k: %d m: %d gt: %s' % (K, M, GT))
    
    
    #init trajectory q and power p foor all k and n
    #store 2 sets of q and p for previous and current iteration
    #TODO

    #determine initial R given p and q
    for n in range(M):
      for k in range(K):
        a[k][n] = math.sqrt(p[k][n])

        I[k][n] = 0
        for j in range(K):
          d[j][k][n] = dist(q[j][n], gt[k])**2 + 0.0001
          I[k][n] += gamma * a[j][n]**2 / d[j][k][n]
        I[k][n] -= gamma * a[k][n]**2 / d[k][k][n]
    shen_r = shen_rpq(a, q, d, I, gt, gamma)
    print("[shen] R: %.2f" % (shen_r))

    for k in range(K):
        for n in range(M):
          print("[shen] q:[%.2f %.2f] a:%.2f" % (q[k][n][0], q[k][n][1], a[k][n]))
        print()

    itr = 0
    while True:
      var_a, var_q = optimize_ma(a, q, p, d, I, gt, gamma)
      
      #update a, q, p
      for idx in range(K * M):
        a[idx // M][idx % M] = var_a.level()[idx]
        p[idx // M][idx % M] = a[idx // M][idx % M]**2

        q[idx // M][idx % M][0] = var_q.level()[idx * 2]
        q[idx // M][idx % M][1] = var_q.level()[idx * 2 + 1]
      
      #update a, d, I with p, q
      for n in range(M):
        for k in range(K):
          a[k][n] = math.sqrt(p[k][n])
          
          I[k][n] = 0
          for j in range(K):
            d[j][k][n] = dist(q[j][n], gt[k])**2 + 0.0001
            if F[j] == F[k]:
              I[k][n] += gamma * a[j][n]**2 / d[j][k][n]
          I[k][n] -= gamma * a[k][n]**2 / d[k][k][n]

      #update R by eq 4 using shen_rpq
      shen_r_new = shen_rpq(a, q, d, I, gt, gamma)
      print("[shen] R: %.2f" % (shen_r_new))
      
      self.plot_traj(q, gt)

      #'''
      for k in range(K):
        for n in range(M):
          print("[shen] q:[%.2f %.2f] a:%.2f" % (q[k][n][0], q[k][n][1], a[k][n]))
      #'''

      #continue if prev r was 0
      if shen_r == 0: 
        shen_r = shen_r_new
        continue

      #termination condition
      '''
      if ((shen_r_new - shen_r) / (shen_r)) < eps:
        print("term")
        time.sleep(30)
        break
      '''

      #update iteration
      shen_r = shen_r_new

      itr += 1
      if itr % 5 == 0:
        pass

    p = np.multiply(a, a)
    return p, q

def optimize_shen(a, q, p, d, I, gt, gamma):
  d_min = 0.05
  v_max = 0.1
  #dist_pad = 100

  K, M = a.shape
  #update a, q by solving complex problem 23
  m = mf.Model('shen_sca')
  var_a = m.variable('a', [K, M], mf.Domain.inRange(0.01, 5))
  var_q = m.variable('q', [K, M, 2], mf.Domain.inRange(-1.0, 1.0))
  var_dist = m.variable('dist_expr', [K, K, M], mf.Domain.greaterThan(0.0))
  var_inner = m.variable('inner_div', [K, K, M])
  var_t = m.variable('t', [K, M, K])
  #log_inner = m.variable('log_inner')
  #m.constraint('min_sep', mf.Expr.hstack(mf.Expr()), mf.Domain.inQCone())
  #m.constraint('level_speed', mf.Expr.hstack(mf.Expr()), mf.Domain.inQCone())
  #m.constraint('min_sep', mf.Expr.hstack(mf.Expr()), mf.Domain.inQCone())

  #fixed starting loc
  for k in range(K):
    m.constraint(mf.Expr.sub(var_q.index(k, 0, 0), q[k][0][0]), mf.Domain.equalsTo(0.0))
    m.constraint(mf.Expr.sub(var_q.index(k, 0, 1), q[k][0][1]), mf.Domain.equalsTo(0.0))
  
  #define expression for 20c in Shen 2020
  inner_1 = np.empty((K, M), dtype=object)
  inner_2 = np.empty((K, M), dtype=object)
  inner_3 = np.empty((K, K, M), dtype=object)
  #for each n, k, create R_k(a[n],q[n],a_r[n],q_r[n])

  obj_expr = mf.Expr.constTerm(0)
  for n in range(M):
    for k in range(K):
      inner_1[k][n] = mf.Expr.zeros(1)
      inner_2[k][n] = mf.Expr.zeros(1)

      t_1_idx = 0
      for j in range(K): 
        #var_dist >= sqrt(x_dist^2 + y_dist^2)
        m.constraint('dist_%d_%d_%d' % (j, k, n), mf.Expr.vstack([
            var_dist.index(j, k, n), 
            mf.Expr.sub(var_q.index(j, n, 0), gt[k][0]), 
            mf.Expr.sub(var_q.index(j, n, 1), gt[k][1])]), mf.Domain.inQCone())

        #inner_1[k][n] = mf.Expr.add(inner_1[k][n], mf.Expr.mul(-1.0 * gamma * p[j][n] / d[j][k][n]**2, var_dist.index(j, k, n)))
        inner_1[k][n] = mf.Expr.add(inner_1[k][n], mf.Expr.sub(mf.Expr.mul(2 * a[j][n] / d[j][k][n], var_a.index(j, n)), mf.Expr.mul(a[j][n]**2 / d[j][k][n]**2, var_dist.index(j, k, n))))

        #
        if j != k:
          #m.constraint(mf.Expr.vstack([mf.Expr.constTerm(0.5), var_power.index(j, k, n), var_a.index(j, n)]), mf.Domain.inRotatedQCone())
          #inner_3 = 
          inner_3[j][k][n] = mf.Expr.add(d[j][k][n], mf.Expr.dot(2 * (q[j][n] - gt[k]), mf.Expr.hstack([mf.Expr.sub(var_q.index(j, n, 0), q[j][n][0]), mf.Expr.sub(var_q.index(j, n, 1), q[j][n][1])])))

          #inner_2[k][n] = summation of var_inner(j, k, n)
          #inner_2[k][n] = mf.Expr.add(inner_2[k][n], var_inner.index(j, k, n))
          m.constraint('inner_%d_%d_%d' % (j, k, n), mf.Expr.vstack([mf.Expr.mul(0.5, var_inner.index(j, k, n)), inner_3[j][k][n], var_a.index(j, n)]), mf.Domain.inRotatedQCone())

          #t[1] >= gamma / (1 + I[k][n]) * inner_2[k][n]
          m.constraint('t1_%d_%d_%d' % (j, k, n), mf.Expr.sub(var_t.index(k, n, 1 + t_1_idx), mf.Expr.mul(gamma / (1 + I[k][n]), var_inner.index(j, k, n))), mf.Domain.greaterThan(0.0))
          t_1_idx += 1
        '''
        #t[2] >= -log(1 + I[k][n])
        m.constraint('t2_%d_%d_%d' % (j, k, n), mf.Expr.hstack(mf.Expr.constTerm(1 + I[k][n]), 1, var_t.index(k, n, 2)), mf.Domain.inPExpCone())
        
        #t[3] >= I[k][n] / (1 + I[k][n])
        m.constraint('t3_%d_%d_%d' % (j, k, n), mf.Expr.sub(var_t.index(k, n, 3), mf.Expr.constTerm(I[k][n] / (1 + I[k][n]))), mf.Domain.greaterThan(0.0))
        '''

        
        '''
        #min sep
        if j > k:
          dist_jk = q[k][n] - q[j][n]
          dist_jk = np.abs(dist_jk)
          #print(dist_jk)
          m.constraint('min_sep_%d_%d_%d' % (j, k, n), mf.Expr.vstack([mf.Expr.constTerm((dist_jk).T), 
              mf.Expr.vstack([mf.Expr.sub(var_q.index(k, n, 0), var_q.index(j, n, 0)), mf.Expr.sub(var_q.index(k, n, 1), var_q.index(j, n, 1))]), 
              mf.Expr.constTerm(q[k][n][0] - q[j][n][0]), 
              mf.Expr.constTerm(q[k][n][1] - q[j][n][1]), 
              mf.Expr.constTerm(d_min)]), 
              mf.Domain.inRotatedQCone())
        '''
      #t0 <= log(1+inner_1)
      #m.constraint('t0_%d_%d' % (k, n), mf.Expr.hstack(
      #    mf.Expr.add(1, mf.Expr.mul(gamma, mf.Expr.sub(mf.Expr.mul(2 * a[k][n] / d[k][k][n], var_a.index(k, n)), mf.Expr.mul(a[k][n]**2 / d[k][k][n]**2, var_dist.index(k, k, n))))), 
      #    1, 
      #    var_t.index(k, n, 0)), mf.Domain.inPExpCone())
      m.constraint('t0_%d_%d' % (k, n), mf.Expr.hstack(
          mf.Expr.add(1, mf.Expr.mul(gamma, inner_1[k][n])), 
          1, 
          var_t.index(k, n, 0)), mf.Domain.inPExpCone())
      
      #flight speed
      if n < M - 1:
        m.constraint('speed_%d_%d' % (k, n), mf.Expr.vstack([mf.Expr.constTerm(v_max), mf.Expr.sub(var_q.index(k, n + 1, 0), var_q.index(k, n, 0)), mf.Expr.sub(var_q.index(k, n + 1, 1), var_q.index(k, n, 1))]), mf.Domain.inQCone())
  
      #t_sign = []

      #for k in range(K):
      #  for n in range(M):
      #t_sign = t_sign + [1] + [-1 for x in range(K - 1)]
      obj_expr = mf.Expr.add(obj_expr, mf.Expr.dot(var_t.slice([k, n, 0], [k + 1, n + 1, K]), [1] + [-1 for x in range(K - 1)]))

  #print(t_sign)
  #obj_expr = mf.Expr.add(obj_expr, mf.Expr.dot(var_t.level(), t_sign))
  m.objective('obj_rate', mf.ObjectiveSense.Maximize, obj_expr)

  #m.setLogHandler(sys.stdout)
  #m.writeTask('shen_sca.opf')
  m.solve()

  #print(m.getProblemStatus())
  print("[opt] %f" % (m.primalObjValue()))
  #print(var_a.level(), var_q.level())
  return var_a, var_q

def shen_rpq(a, q, d, I, gt, gamma):
  K, M = I.shape

  r_temp = np.zeros((K))
  r_total = np.zeros((M))
  r_total_k = np.zeros((K))

  #nth time slot
  for n in range(M):
    
    r_total[n] = 0
    #kth uav-gt pair
    for k in range(K):
      #interference over all other uavs for kth pair
      r_temp[k] = I[k][n] + gamma * a[k][n]**2 / d[k][k][n]
      
      #calculate rate for kth pair at time n
      r_total[n] += math.log(1 + gamma * a[k][n]**2 / d[k][k][n]) - math.log(1 + I[k][n])
      r_total_k[k] += math.log(1 + gamma * a[k][n]**2 / d[k][k][n]) - math.log(1 + I[k][n])
      #r_total[n] += math.log(1 + r_temp[k]) - math.log(1 + I[k][n])
      #r_total_k[k] += math.log(1 + r_temp[k]) - math.log(1 + I[k][n])

  for k in range(K):
    print("[rpq] %d: %.2f" % (k, r_total_k[k]))
  return sum(r_total)

def optimize_ma(a, q, p, d, I, gt, gamma):
  d_min = 0.05
  v_max = 0.1
  #dist_pad = 100

  K, M = a.shape
  #update a, q by solving complex problem 23
  m = mf.Model('shen_sca')
  var_a = m.variable('a', [K, M], mf.Domain.inRange(0.001, 5))
  var_q = m.variable('q', [K, M, 2], mf.Domain.inRange(-1.0, 1.0))
  var_dist = m.variable('dist_expr', [K, K, M])
  var_inner = m.variable('inner_div', [K, K, M])
  var_t = m.variable('t', [K, M, K])
  #log_inner = m.variable('log_inner')
  #m.constraint('min_sep', mf.Expr.hstack(mf.Expr()), mf.Domain.inQCone())
  #m.constraint('level_speed', mf.Expr.hstack(mf.Expr()), mf.Domain.inQCone())
  #m.constraint('min_sep', mf.Expr.hstack(mf.Expr()), mf.Domain.inQCone())

  #fixed starting loc
  for k in range(K):
    m.constraint(mf.Expr.sub(var_q.index(k, 0, 0), q[k][0][0]), mf.Domain.equalsTo(0.0))
    m.constraint(mf.Expr.sub(var_q.index(k, 0, 1), q[k][0][1]), mf.Domain.equalsTo(0.0))
  
  #define expression for 20c in Shen 2020
  inner_1 = np.empty((K, M), dtype=object)
  inner_2 = np.empty((K, M), dtype=object)
  inner_3 = np.empty((K, K, M), dtype=object)
  #for each n, k, create R_k(a[n],q[n],a_r[n],q_r[n])

  obj_expr = mf.Expr.constTerm(0)
  for n in range(M):
    for k in range(K):
      inner_1[k][n] = mf.Expr.zeros(1)
      inner_2[k][n] = mf.Expr.zeros(1)

      t_1_idx = 0
      for j in range(K): 
        #dist_expr = q.index(j, n, 0) - gt[k][0]
        m.constraint('dist_%d_%d_%d' % (j, k, n), mf.Expr.vstack([mf.Expr.constTerm(0.5), var_dist.index(j, k, n), mf.Expr.sub(var_q.index(j, n, 0), gt[k][0]), mf.Expr.sub(var_q.index(j, n, 1), gt[k][1])]), mf.Domain.inRotatedQCone())
        #var_dist = mf.Expr.add(
        #    mf.Expr.sub(var_q.index(j, n, 0), gt[k][0]) * mf.Expr.sub(var_q.index(j, n, 0), gt[k][0]), 
        #    mf.Expr.sub(var_q.index(j, n, 1), gt[k][1]) * mf.Expr.sub(var_q.index(j, n, 1), gt[k][1]))

        inner_1[k][n] = mf.Expr.add(inner_1[k][n], mf.Expr.sub(mf.Expr.mul(2 * a[j][n] / d[j][k][n], var_a.index(j, n)), mf.Expr.mul(p[j][n] / d[j][k][n]**2, var_dist.index(j, k, n))))
        #constraint on inner_1... can bound some temp variable to be >= ||q[j][n] - gt[k]||^2
        #m.constraint(mf.Expr.)

        #t0 <= log(1+inner_1)
        m.constraint('t0_%d_%d_%d' % (j, k, n), mf.Expr.hstack(mf.Expr.add(1, inner_1[k][n]), 1, var_t.index(k, n, 0)), mf.Domain.inPExpCone())

        #
        if j != k:
          #m.constraint(mf.Expr.vstack([mf.Expr.constTerm(0.5), var_power.index(j, k, n), var_a.index(j, n)]), mf.Domain.inRotatedQCone())
          #inner_3 = 
          inner_3[j][k][n] = mf.Expr.add(d[j][k][n], mf.Expr.dot(2 * (q[j][n] - gt[k]), mf.Expr.hstack([mf.Expr.sub(var_q.index(j, n, 0), q[j][n][0]), mf.Expr.sub(var_q.index(j, n, 1), q[j][n][1])])))

          #inner_2[k][n] = summation of var_inner(j, k, n)
          #inner_2[k][n] = mf.Expr.add(inner_2[k][n], var_inner.index(j, k, n))
          m.constraint('inner_%d_%d_%d' % (j, k, n), mf.Expr.vstack([mf.Expr.mul(0.5, var_inner.index(j, k, n)), inner_3[j][k][n], var_a.index(j, n)]), mf.Domain.inRotatedQCone())

          #t[1] >= gamma / (1 + I[k][n]) * inner_2[k][n]
          m.constraint('t1_%d_%d_%d' % (j, k, n), mf.Expr.sub(var_t.index(k, n, 1 + t_1_idx), mf.Expr.mul(gamma / (1 + I[k][n]), var_inner.index(j, k, n))), mf.Domain.greaterThan(0.0))
          t_1_idx += 1
        '''
        #t[2] >= -log(1 + I[k][n])
        m.constraint('t2_%d_%d_%d' % (j, k, n), mf.Expr.hstack(mf.Expr.constTerm(1 + I[k][n]), 1, var_t.index(k, n, 2)), mf.Domain.inPExpCone())
        
        #t[3] >= I[k][n] / (1 + I[k][n])
        m.constraint('t3_%d_%d_%d' % (j, k, n), mf.Expr.sub(var_t.index(k, n, 3), mf.Expr.constTerm(I[k][n] / (1 + I[k][n]))), mf.Domain.greaterThan(0.0))
        '''

        '''
        #min sep
        if j > k:
          dist_jk = q[k][n] - q[j][n]
          dist_jk = np.abs(dist_jk)
          #print(dist_jk)
          m.constraint('min_sep_%d_%d_%d' % (j, k, n), mf.Expr.vstack([mf.Expr.constTerm((dist_jk).T), 
              mf.Expr.vstack([mf.Expr.sub(var_q.index(k, n, 0), var_q.index(j, n, 0)), mf.Expr.sub(var_q.index(k, n, 1), var_q.index(j, n, 1))]), 
              mf.Expr.constTerm(q[k][n][0] - q[j][n][0]), 
              mf.Expr.constTerm(q[k][n][1] - q[j][n][1]), 
              mf.Expr.constTerm(d_min)]), 
              mf.Domain.inRotatedQCone())
        '''

      #flight speed
      if n < M - 1:
        m.constraint('speed_%d_%d' % (k, n), mf.Expr.vstack([mf.Expr.constTerm(v_max), mf.Expr.sub(var_q.index(k, n + 1, 0), var_q.index(k, n, 0)), mf.Expr.sub(var_q.index(k, n + 1, 1), var_q.index(k, n, 1))]), mf.Domain.inQCone())
      #obj_expr = mf.Expr(math.log(1 + gamma * mf.Expr.sum(inner_1)) - gamma / (1 + I[k][n]) * mf_Expr.sum(inner_2) - math.log(1 + I[k][n]) + I[k][n] / (1 + I[k][n]))
      #obj_expr = mf.Expr(math.log(1 + gamma * mf.Expr.sum(inner_1)))
  
      #t_sign = []

      #for k in range(K):
      #  for n in range(M):
      #t_sign = t_sign + [1] + [-1 for x in range(K - 1)]
      obj_expr = mf.Expr.add(obj_expr, mf.Expr.dot(var_t.slice([k, n, 0], [k + 1, n + 1, K]), [1] + [-1 for x in range(K - 1)]))

  #print(t_sign)
  #obj_expr = mf.Expr.add(obj_expr, mf.Expr.dot(var_t.level(), t_sign))
  m.objective('obj_rate', mf.ObjectiveSense.Maximize, obj_expr)

  m.setLogHandler(sys.stdout)
  m.writeTask('shen_sca.opf')
  m.solve()

  #print(m.getProblemStatus())
  print("[opt] %f" % (m.primalObjValue()))
  #print(var_a.level(), var_q.level())
  return var_a, var_q

def dist(pos_0, pos_1):
  return np.linalg.norm(np.array(pos_0) - np.array(pos_1))

#calculate received signal power in dB
def power_fspl(pw_tx, freq, dist):
  if dist <= 0 or freq <= 0:
    return pw_tx

  loss = 20 * math.log10(4 * math.pi / 300 * freq * dist)
  return pw_tx - loss