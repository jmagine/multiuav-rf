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
import time

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

    self.gt = np.zeros((N_DRONES, 2))
    for i in range(N_DRONES):
      #gen random x and y pos for each ground terminal
      self.gt[i][0] = random.uniform(self.p_bounds[0][0], self.p_bounds[0][1])
      self.gt[i][1] = random.uniform(self.p_bounds[1][0], self.p_bounds[1][1])

    #TODO move this somewhere more appropriate later on
    self.shen_sca(len(self.drn), 10, self.gt)

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

  def shen_sca(self, K, M, GT):

    p = np.zeros((2, K, M))
    q = np.zeros((2, K, M, 2))
    a = np.zeros((2, K, M))
    d = np.zeros((K, K, M))
    I = np.zeros((K, M))
    gt = GT
    gamma = 1

    eps = 0.1 #iteration tolerance
    
    
    #init trajectory q and power p foor all k and n
    #store 2 sets of q and p for previous and current iteration
    #TODO

    #determine initial R given p and q
    for n in range(M):
      for k in range(K):
        a[0][k][n] = math.sqrt(p[0][k][n])

        I[k][n] = 0
        for j in range(K):
          d[j][k][n] = dist(q[0][j][n], gt[k])**2 + 0.0001
          I[k][n] += gamma * a[0][j][n]**2 / d[j][k][n]
        I[k][n] -= gamma * a[0][k][n]**2 / d[k][k][n]
    shen_r = shen_rpq(a, q, d, I, gt, gamma)

    while True:
      #update a, d, I with p, q
      for n in range(M):
        for k in range(K):
          a[0][k][n] = math.sqrt(p[0][k][n])
          
          I[k][n] = 0
          for j in range(K):
            d[j][k][n] = dist(q[0][j][n], gt[k])**2 + 0.0001
            I[k][n] += gamma * a[0][j][n]**2 / d[j][k][n]
          I[k][n] -= gamma * a[0][k][n]**2 / d[k][k][n]

      #update a, q by solving complex problem 23
      m = mf.Model('shen_sca')
      var_a = m.variable('a', [K, M], mf.Domain.greaterThan(0.0))
      var_q = m.variable('q', [K, M, 2], mf.Domain.greaterThan(0.0))
      var_dist = m.variable('dist_expr', [K, K, M], mf.Domain.greaterThan(0.0))
      var_power = m.variable('power_expr', [K, K, M], mf.Domain.greaterThan(0.0))
      var_t = m.variable(4)
      #log_inner = m.variable('log_inner')
      #m.constraint('min_sep', mf.Expr.hstack(mf.Expr()), mf.Domain.inQCone())
      #m.constraint('level_speed', mf.Expr.hstack(mf.Expr()), mf.Domain.inQCone())
      #m.constraint('min_sep', mf.Expr.hstack(mf.Expr()), mf.Domain.inQCone())
      #m.constraint('power', mf.Expr(), mf.Domain.)

      #define expression for 20c in Shen 2020
      inner_1 = np.empty((K, M), dtype=object)
      inner_2 = np.empty((K, M), dtype=object)
      #for each n, k, create R_k(a[n],q[n],a_r[n],q_r[n])
      for n in range(M):
          for k in range(K):
            
            inner_1[k][n] = mf.Expr.zeros(1)
            inner_2[k][n] = mf.Expr.zeros(1)
            for j in range(1, K): 
              #dist_expr = q.index(j, n, 0) - gt[k][0]
              m.constraint(mf.Expr.vstack([mf.Expr.constTerm(0.5), var_dist.index(j, k, n), mf.Expr.sub(var_q.index(j, n, 0), gt[k][0]), mf.Expr.sub(var_q.index(j, n, 1), gt[k][1])]), mf.Domain.inRotatedQCone())
              #var_dist = mf.Expr.add(
              #    mf.Expr.sub(var_q.index(j, n, 0), gt[k][0]) * mf.Expr.sub(var_q.index(j, n, 0), gt[k][0]), 
              #    mf.Expr.sub(var_q.index(j, n, 1), gt[k][1]) * mf.Expr.sub(var_q.index(j, n, 1), gt[k][1]))

              inner_1[k][n] = mf.Expr.add(inner_1[k][n], mf.Expr.sub(mf.Expr.mul(2 * a[0][j][n] / d[j][k][n], var_a.index(j, n)), mf.Expr.mul(p[0][j][n] / d[j][k][n]**2, var_dist.index(j, k, n))))
              #constraint on inner_1... can bound some temp variable to be >= ||q[j][n] - gt[k]||^2
              #m.constraint(mf.Expr.)

              #constraint on first log t0
              m.constraint(mf.Expr.hstack(mf.Expr.add(1, inner_1[k][n]), 1, var_t.index(0)), mf.Domain.inPExpCone())

              if j != k:
                m.constraint(mf.Expr.vstack([mf.Expr.constTerm(0.5), var_power.index(j, k, n), var_a.index(j, n)]), mf.Domain.inRotatedQCone())
                inner_2[k][n] = mf.Expr.add(inner_2[k][n], var_power.index(j, k, n) / (mf.Expr.add(d[j][k][n], mf.Expr.dot(2 * (q[0][j][n] - gt[k]), mf.Expr.vstack([mf.Expr.sub(var_q.index(j, n, 0), q[0][j][n][0]), mf.Expr.sub(var_q.index(j, n, 1), q[0][j][n][1])])))))

              #t[1] >= gamma / (1 + I) * inner_2[k][n]
              m.constraint(mf.Expr.sub(var_t.index(1), mf.Expr.mul(gamma / (1 + I[k][n]), inner_2[k][n])), mf.Domain.greaterThan(0.0))

              #-log(1 + I[k][n])
              m.constraint(mf.Expr.hstack(mf.Expr(1 + I[k][n]), 1, var_t.index(2)), mf.Domain.inPExpCone())
              m.constraint(mf.Expr.sub(var_t.index(3), mf.Expr(I[k][n]) / mf.Expr.sum(1, I[k][n])), mf.Domain.greaterThan(0.0))

      #obj_expr = mf.Expr(math.log(1 + gamma * mf.Expr.sum(inner_1)) - gamma / (1 + I[k][n]) * mf_Expr.sum(inner_2) - math.log(1 + I[k][n]) + I[k][n] / (1 + I[k][n]))
      #obj_expr = mf.Expr(math.log(1 + gamma * mf.Expr.sum(inner_1)))
      obj_expr = mf.Expr.dot(var_t, [1, -1, -1, 1])

      #m.constraint(mf.Expr.hstack(log_inner, 1, t.index(0), mf.Domain.inPExpCone()))
      #m.constraint(mf.Expr.vstack(inner_1), Domain.inQCone())
      m.objective('obj_rate', mf.ObjectiveSense.Maximize, obj_expr)
      m.solve()

      #update R by eq 4 using shen_rpq
      shen_r_new = shen_rpq(a, q, d, I, gt, gamma)
      print("R: %.2f" % (shen_r_new))

      #continue if prev r was 0
      if shen_r == 0: 
        shen_r = shen_r_new
        continue

      #termination condition
      if ((shen_r_new - shen_r) / (shen_r)) < eps:
        break

      #update iteration
      shen_r = shen_r_new
      a[0] = a[1]
      q[0] = q[1]

    p = np.multiply(a[1], a[1])
    return p, q

def shen_rpq(a, q, d, I, gt, gamma):
  K, M = I.shape

  r_temp = np.zeros((K))
  r_total = np.zeros((M))

  #nth time slot
  for n in range(M):
    
    r_total[n] = 0
    #kth uav-gt pair
    for k in range(K):
      #interference over all other uavs for kth pair
      r_temp[k] = I[k][n] + gamma * a[0][k][n]**2 / d[k][k][n]
      
      #calculate rate for kth pair at time n
      r_total[n] += math.log10(1 + r_temp[k]) - math.log10(1 + I[k][n])

  return sum(r_total)

def dist(pos_0, pos_1):
  return np.linalg.norm(np.array(pos_0) - np.array(pos_1))

#calculate received signal power in dB
def power_fspl(pw_tx, freq, dist):
  if dist <= 0 or freq <= 0:
    return pw_tx

  loss = 20 * math.log10(4 * math.pi / 300 * freq * dist)
  return pw_tx - loss