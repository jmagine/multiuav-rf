import numpy as np 
import random 
import math
import networkx as nx
import mosek.fusion as mf
import sys
import time

import utils
import vis

class ma_sca():
  def __init__(self, K, M, GT, F, v_max=0.2, h_min=0.2, graph_max_edge_dist=0.5, Q=None, P=None, plotter=None):

    #init shen params
    self.d = np.zeros((K, K, M))
    self.I = np.zeros((K, M))
    self.gt = GT
    self.gamma = 10**4
    self.eps = 0.01 #itr tolerance

    if Q is not None:
      self.q = Q
    else:
      self.q = np.zeros((K, M, 2))

    if P is not None:
      self.p = P
    else:
      self.p = np.zeros((K, M))

    if plotter is not None:
      self.plotter = plotter

    '''
    for k in range(K):
      print("\\addplot[color=blue,mark=o] coordinates{", end='')
      for n in range(M):
        print("(%.2f,%.2f)" % (self.q[k][n][0], self.q[k][n][1]), end='')
      print("};")
    print()
    '''

    '''
    for k in range(K):
      print("\\addplot[color=red,mark=o] coordinates{", end='')
      for n in range(M):
        print("(%d,%.2f)" % (n, 10 * math.log(self.p[k][n], 10)), end='')
      print("};")
    print()
    '''

    self.a = np.sqrt(self.p)

    #init graph for coloring based on UAV trajectory ending locations
    self.g_gt = nx.Graph()
    
    #freq init
    self.F = F

    self.v_max = v_max
    self.h_min = h_min

    #visualize initial trajectory
    self.plotter.plot_traj(self.q, self.gt)

    #update graph based on trajectory endpoints
    self.max_color_dist = graph_max_edge_dist
    self.update_graph(self.q[:,M - 1,:])
    self.f = self.color_graph(self.F)

    #self.f = self.naive_freq(2.484)

    print('[sca] init complete: K: %d M: %d eps: %f F: %s' % (K, M, self.eps, self.F))

    #determine initial d, I, R given p, q, f
    for f in self.f:
      uav_idxs = self.f[f]

      for n in range(M):
        for k in uav_idxs:
          self.a[k][n] = math.sqrt(self.p[k][n])

          self.I[k][n] = 0
          for j in uav_idxs:
            self.d[j][k][n] = utils.dist2(self.q[j][n], self.gt[k], self.h_min)**2
            self.I[k][n] += self.gamma * self.a[j][n]**2 / self.d[j][k][n]
          self.I[k][n] -= self.gamma * self.a[k][n]**2 / self.d[k][k][n]

    self.rate= utils.calc_rates(self.a, self.q, self.f, self.d, self.I, self.gt, self.gamma)
    self.rate = np.sum(self.rate) * 1.44
    print("[sca] init R: %f" % (self.rate))

  def sca(self):
    K, M = self.a.shape
    eps = 0.00001

    while True:
      #split sca into smaller shen optimizations
      for f in self.f:

        #print(f, self.f)

        print("[sca] F: %.3f" % (f))
        uav_idxs = self.f[f]

        if len(uav_idxs) == 0:
          continue

        itr = 0
        while True:
          #num_uavs = len(uav_idxs)
          #idx_convert = []

          a_f, q_f, p_f, d_f, I_f, gt_f = slice_freq(uav_idxs, self.a, self.q, self.p, self.d, self.I, self.gt)
          var_a, var_q, obj_val = optimize_shen(a_f, q_f, p_f, d_f, I_f, gt_f, self.gamma, self.v_max, self.h_min)
          
          for idx in range(len(uav_idxs)):
            for n in range(M):
              obj_val = obj_val - math.log(1 + I_f[idx][n], 2) + I_f[idx][n] / (1 + I_f[idx][n])
          #unpack a, q from optimizer
          a_temp = np.zeros((len(uav_idxs), M))
          p_temp = np.zeros((len(uav_idxs), M))
          q_temp = np.zeros((len(uav_idxs), M, 2))

          for idx in range(len(uav_idxs) * M):
            a_temp[idx // M][idx % M] = var_a.level()[idx]
            p_temp[idx // M][idx % M] = a_temp[idx // M][idx % M]**2

            q_temp[idx // M][idx % M][0] = var_q.level()[idx * 2]
            q_temp[idx // M][idx % M][1] = var_q.level()[idx * 2 + 1]

          self.a[uav_idxs] = a_temp
          self.p[uav_idxs] = p_temp
          self.q[uav_idxs] = q_temp

          #print(var_a.level())
          #print(a_temp.shape)
          #print(a_temp)
          
          '''
          for uav in uav_idxs:
            print("[sca] uav: %d p: %.4f q: %.2f %.2f" % (uav, self.p[uav][M-1], self.q[uav][M-1][0], self.q[uav][M-1][1]))
          '''
          '''
          var_a, var_q = optimize_shen(self.a, self.q, self.p, self.d, self.I, self.gt, self.gamma)
          #update a, q, p
          for idx in range(K * M):
            self.a[idx // M][idx % M] = var_a.level()[idx]
            self.p[idx // M][idx % M] = self.a[idx // M][idx % M]**2

            self.q[idx // M][idx % M][0] = var_q.level()[idx * 2]
            self.q[idx // M][idx % M][1] = var_q.level()[idx * 2 + 1]
          '''

        

          #self.f = self.naive_freq(2.484)

          #update d, I
          for n in range(M):
            for k in uav_idxs:
              self.a[k][n] = math.sqrt(self.p[k][n])

              self.I[k][n] = 0
              for j in uav_idxs:
                self.d[j][k][n] = utils.dist2(self.q[j][n], self.gt[k], self.h_min)**2
                self.I[k][n] += self.gamma * self.a[j][n]**2 / self.d[j][k][n]
              self.I[k][n] -= self.gamma * self.a[k][n]**2 / self.d[k][k][n]

          #update R by eq 4 using shen_rpq
          rate_new = utils.calc_rates(self.a, self.q, self.f, self.d, self.I, self.gt, self.gamma)
          rate_new = np.sum(rate_new) * 1.44
          #print("[sca] R: %.2f opt: %.2f"  % (rate_new, obj_val))
          
          #visualize and print
          self.plotter.plot_traj(self.q, self.gt)
          '''
          for k in range(K):
            for n in range(M):
              if n % 5 == 0:
                print("[shen] q:[%.2f %.2f] a:%.2f" % (self.q[k][n][0], self.q[k][n][1], self.a[k][n]))
            print()
          '''

          '''
          #continue if prev r was 0
          if self.rate <= 0: 
            self.rate = rate_new
            continue
          '''

          #termination condition
          if (rate_new - self.rate) / (self.rate) < eps:
            #self.rate = rate_new
            #print("[sca] R: %f opt: %f" % (self.rate, obj_val))
            break

          #update iteration
          self.rate = rate_new

          itr += 1
          if itr % 5 == 0:
            pass

      for k in range(K):
        print("[sca] uav: %d p: %.4f q: [%.2f %.2f]" % (k, self.p[k][M-1], self.q[k][M-1][0], self.q[k][M-1][1]))
      print("[sca] R: %f --------------------" % (rate_new))
      #print()

      #update freq graph based on trajectory endpoints
      self.update_graph(self.q[:,M - 1,:])
      f_new = self.color_graph(self.F)
      if self.f == f_new:
        print("[sca] finished")
        print("[sca] R: %f" % (self.rate))
        break
      self.f = f_new
      
      #update d, I
      for f in self.f:
        uav_idxs = self.f[f]

        for n in range(M):
          for k in uav_idxs:
            self.a[k][n] = math.sqrt(self.p[k][n])

            self.I[k][n] = 0
            for j in uav_idxs:
              self.d[j][k][n] = utils.dist2(self.q[j][n], self.gt[k], self.h_min)**2
              self.I[k][n] += self.gamma * self.a[j][n]**2 / self.d[j][k][n]
            self.I[k][n] -= self.gamma * self.a[k][n]**2 / self.d[k][k][n]

    #print ending diagnostics
    rate_final = utils.calc_rates(self.a, self.q, self.f, self.d, self.I, self.gt, self.gamma)
    rate_final = np.sum(rate_final) * 1.44
    print("[sca] R FINAL: %f --------------------" % (rate_final))
    '''
    print("[sca] p FINAL:")
    for k in range(K):
      print("\\addplot[color=blue,mark=o] coordinates{", end='')
      for n in range(M):
        print("(%d,%.2f)" % (n, 10 * math.log(self.p[k][n], 10)), end='')
      print("};")
    print()
    '''
    '''
    for k in range(K):
      print("\\addplot[color=blue,mark=o] coordinates{", end='')
      for n in range(M):
        print("(%.2f,%.2f)" % (self.q[k][n][0], self.q[k][n][1]), end='')
      print("};")
    print()
    
    '''

    self.p = np.multiply(self.a, self.a)
    time.sleep(60)
    return self.p, self.q

  #update graph given new set of target positions
  def update_graph(self, target_pos):
    #print("update_graph: \n", target_pos)
    t_start = time.time()

    self.g_gt.clear()

    for i in range(len(target_pos)):
      self.g_gt.add_node(i, p=target_pos[i])
    
    for v1 in self.g_gt:
      pos1 = self.g_gt.nodes[v1]['p']
      for v2 in self.g_gt:
        pos2 = self.g_gt.nodes[v2]['p']
      
        if v1 == v2 or self.g_gt.has_edge(v1, v2):
          continue
        
        #nodes are close enough that they should be assigned sep freqs if possible
        if utils.dist(pos1, pos2) <= self.max_color_dist:
          self.g_gt.add_edge(v1, v2, weight=utils.dist(pos1, pos2))
    
    t_end = time.time()
    #print("[upd graph] time:", t_end - t_start)

  def color_graph(self, F):

    t_start = time.time()

    F = list(sorted(F, reverse=True))
    #print("[color] freqs:", F)

    range_factor = 20000.0

    #freq assignments f
    f = [0]

    while 0 in f:
      #reinit everything
      g = self.g_gt.copy()
      max_vert_idx = len(g.nodes())

      g_freqs = []

      #print("[color] range factor:", range_factor)

      f = [0 for i in range(g.number_of_nodes())]

      #starting from highest freq, max freq reuse
      for curr_freq in F:
        #determine max dist threshold for current freq
        dist_thresh = range_factor / curr_freq**2

        #fetch full graph state
        edges = g.edges(data="weight")
        edges = list(sorted(edges, key=lambda x: x[2]))
        #edges = list(sorted(edges, reverse=True, key=lambda x: x[2]))
        verts = g.nodes()

        #print(edges)
        #print(verts)

        '''
        #remove edges with larger weight than threshold
        while edge_ptr < len(edges) and edges[edge_ptr][2] > dist_thresh:
          g.remove_edge(edges[edge_ptr][0], edges[edge_ptr][1])
          edge_ptr += 1
        '''

        #create a copy of graph for use on this freq
        g_freq = g.copy()

        #remove edges for other freqs
        for edge in edges:
          if edge[2] > dist_thresh:
            g_freq.remove_edge(edge[0], edge[1])
        g_freqs.append(g_freq)

        #find connected components in graph
        visited = [0 for x in range(max_vert_idx)]
        cc_id = []
        curr_id = 0

        #print(verts)

        for v in verts:
          #if not g.has_node(v):
          #  continue

          #if previously unvisited, recursively visit
          num_visited = visit_node(g_freq, v, visited, cc_id, curr_id)

          if num_visited > 0:
            curr_id += 1
        
        #print(cc_id)
        #for each cc, assign vert with lowest degree the current freq
        for cc in cc_id:
          g_sub = g_freq.copy()

          while len(cc) > 0:
            lowest_degree = 1000
            lowest_vertex = -1
            for v in cc:
              if g_sub.degree(v) < lowest_degree:
                lowest_degree = g_sub.degree(v)
                lowest_vertex = v

            if lowest_vertex != -1:
              f[lowest_vertex] = curr_freq

              #remove the vertex's neighbors from consideration for this freq
              neighbors = list(g_sub.adj[lowest_vertex])
              for neighbor in neighbors:
                g_sub.remove_node(neighbor)
                cc.remove(neighbor)

              #remove the vertex and its edges from consideration completely
              g_sub.remove_node(lowest_vertex)
              g.remove_node(lowest_vertex)
              cc.remove(lowest_vertex)

              #print(g.nodes())

      
      #change range factor
      range_factor *= 0.95

    #print(f)
    t_end = time.time()
    
    freq_assignment_dict = {}
    for freq in F:
      freq_assignment_dict[freq] = []

    for i, freq in enumerate(f):
      freq_assignment_dict[freq].append(i)

    #print("[color] time:", t_end - t_start)
    print("[color] range_fac:", range_factor)
    print("[color] assign:", freq_assignment_dict)

    self.plotter.plot_graph(g_freqs[0], self.gt, f)
    return freq_assignment_dict

  #assign all vehicles to a single freq
  def naive_freq(self, freq):
    freq_assignments = {}
    uav_idxs = list(range(len(self.g_gt.nodes())))
    freq_assignments[freq] = uav_idxs
    return freq_assignments


#apply curr_id to all nodes in connected component  
def visit_node(g, i, visited, cc_id, curr_id):
  num_visited = 0

  if visited[i] != 1:
    visited[i] = 1
    if curr_id >= len(cc_id):
      cc_id.append([i])
    else:
      cc_id[curr_id].append(i)

    num_visited += 1

    #print(i)
    for v in g.adj[i]:
     num_visited += visit_node(g, v, visited, cc_id, curr_id)

  return num_visited

def slice_freq(idxs, a, q, p, d, I, gt):
  
  #print(idxs)
  idxs = np.array(idxs)

  a_f = a[idxs]
  q_f = q[idxs]
  p_f = p[idxs]
  I_f = I[idxs]
  gt_f = gt[idxs]

  d_f = d[idxs[:, None], idxs]
  #print(d_f.shape)

  return a_f, q_f, p_f, d_f, I_f, gt_f

def shen_sca(e, K, M, GT):

  p = np.zeros((K, M))
  q = np.zeros((K, M, 2))
  a = np.zeros((K, M))
  d = np.zeros((K, K, M))
  I = np.zeros((K, M))
  gt = GT
  gamma = 0.01
  eps = 0.01 #itr tolerance

  #drone trajectory init
  for i in range(K):
    q[i][0][0] = random.uniform(e.p_bounds[0][0], e.p_bounds[0][1])
    q[i][0][1] = random.uniform(e.p_bounds[1][0], e.p_bounds[1][1])

    x_step = (e.gt[i][0] - q[i][0][0]) / M
    y_step = (e.gt[i][1] - q[i][0][1]) / M
    for n in range(M):
      q[i][n][0] = q[i][0][0] + x_step * n
      q[i][n][1] = q[i][0][1] + y_step * n
  
  #drone power init
  for n in range(M):
    for k in range(K):
      p[k][n] = 1

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
        d[j][k][n] = utils.dist(q[j][n], gt[k])**2
        I[k][n] += gamma * a[j][n]**2 / d[j][k][n]
      I[k][n] -= gamma * a[k][n]**2 / d[k][k][n]
  shen_r = utils.shen_rpq(a, q, d, I, gt, gamma)
  print("[shen] R: %.2f" % (shen_r))

  #visualize and print
  e.plot_traj(q, gt)
  for k in range(K):
      for n in range(M):
        if n % 5 == 0:
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
          d[j][k][n] = utils.dist(q[j][n], gt[k])**2
          I[k][n] += gamma * a[j][n]**2 / d[j][k][n]
        I[k][n] -= gamma * a[k][n]**2 / d[k][k][n]

    #update R by eq 4 using shen_rpq
    shen_r_new = utils.shen_rpq(a, q, d, I, gt, gamma)
    print("[shen] R: %.2f" % (shen_r_new))
    
    #visualize and print
    e.plot_traj(q, gt)
    for k in range(K):
      for n in range(M):
        if n % 5 == 0:
          print("[shen] q:[%.2f %.2f] a:%.2f" % (q[k][n][0], q[k][n][1], a[k][n]))
      print()

    #continue if prev r was 0
    if shen_r <= 0: 
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

def optimize_shen(a, q, p, d, I, gt, gamma, v_max=0.2, h_min=0.2):
  d_min = 0.0

  K, M = a.shape

  #print(K,M)
  #update a, q by solving complex problem 23
  m = mf.Model('shen_sca')
  var_a = m.variable('a', [K, M], mf.Domain.inRange(0.01, 31.6))
  var_q = m.variable('q', [K, M, 2], mf.Domain.inRange(-500, 500))
  var_dist = m.variable('dist_expr', [K, K, M], mf.Domain.greaterThan(0.0))
  var_inner = m.variable('inner_div', [K, K, M])
  var_t = m.variable('t', [K, M, K])

  #fixed starting loc
  for k in range(K):
    m.constraint(mf.Expr.sub(var_q.index(k, 0, 0), q[k][0][0]), mf.Domain.equalsTo(0.0))
    m.constraint(mf.Expr.sub(var_q.index(k, 0, 1), q[k][0][1]), mf.Domain.equalsTo(0.0))
  
  #define expression for 20c in Shen 2020
  inner_1 = np.empty((K, M), dtype=object)
  inner_2 = np.empty((K, K, M), dtype=object)

  #for each n, k, create R_k(a[n],q[n],a_r[n],q_r[n]) and add the expressions together for objective
  obj_expr = mf.Expr.constTerm(0)
  for n in range(M):
    for k in range(K):

      inner_1[k][n] = mf.Expr.zeros(1)
      t_1_idx = 0
      for j in range(K): 

        #t[0] computations
        #var_dist >= sqrt(x_dist^2 + y_dist^2 +z_dist^2)
        m.constraint('dist_%d_%d_%d' % (j, k, n), mf.Expr.vstack([
            mf.Expr.constTerm(0.5), 
            var_dist.index(j, k, n), 
            mf.Expr.sub(var_q.index(j, n, 0), gt[k][0]), 
            mf.Expr.sub(var_q.index(j, n, 1), gt[k][1]),
            mf.Expr.constTerm(h_min)]), mf.Domain.inRotatedQCone())

        #inner_1[k][n] = mf.Expr.add(inner_1[k][n], mf.Expr.mul(-1.0 * gamma * p[j][n] / d[j][k][n]**2, var_dist.index(j, k, n)))
        #inner_1[k][n] = mf.Expr.add(inner_1[k][n], mf.Expr.sub(mf.Expr.mul(2 * a[j][n] / d[j][k][n], var_a.index(j, n)), mf.Expr.mul(a[j][n]**2 / d[j][k][n]**2, var_dist.index(j, k, n))))

        #t[1] computations
        if j != k:
          dist_temp = np.zeros((3))
          dist_temp[0] = q[j][n][0] - gt[k][0]
          dist_temp[1] = q[j][n][1] - gt[k][1]
          dist_temp[2] = h_min
          inner_2[j][k][n] = mf.Expr.add(d[j][k][n], mf.Expr.dot(2 * (dist_temp), mf.Expr.hstack([mf.Expr.sub(var_q.index(j, n, 0), q[j][n][0]), mf.Expr.sub(var_q.index(j, n, 1), q[j][n][1]), mf.Expr.constTerm(0)])))
          m.constraint('inner_%d_%d_%d' % (j, k, n), mf.Expr.vstack([mf.Expr.mul(0.5, var_inner.index(j, k, n)), inner_2[j][k][n], var_a.index(j, n)]), mf.Domain.inRotatedQCone())

          #t[1+j] >= gamma / (1 + I[k][n]) * inner_2[k][n]
          m.constraint('t1_%d_%d_%d' % (j, k, n), mf.Expr.sub(var_t.index(k, n, 1 + t_1_idx), mf.Expr.mul(gamma / (1 + I[k][n]), var_inner.index(j, k, n))), mf.Domain.greaterThan(0.0))
          t_1_idx += 1

        '''
        #t[2] >= -log(1 + I[k][n])
        m.constraint('t2_%d_%d_%d' % (j, k, n), mf.Expr.hstack(mf.Expr.constTerm(1 + I[k][n]), 1, var_t.index(k, n, 2)), mf.Domain.inPExpCone())
        
        #t[3] >= I[k][n] / (1 + I[k][n])
        m.constraint('t3_%d_%d_%d' % (j, k, n), mf.Expr.sub(var_t.index(k, n, 3), mf.Expr.constTerm(I[k][n] / (1 + I[k][n]))), mf.Domain.greaterThan(0.0))
        '''
        
        '''
        #min sep constraint
        if j > k:
          dist_jk = q[k][n] - q[j][n]
          dist_jk = np.abs(dist_jk)
          #print(j, k, n, dist_jk)
          #print(mf.Expr.dot(dist_jk, mf.Expr.vstack([mf.Expr.sub(var_q.index(k, n, 0), var_q.index(j, n, 0)), mf.Expr.sub(var_q.index(k, n, 1), var_q.index(j, n, 1))])))
          #print(dist_jk)
          m.constraint('min_sep_%d_%d_%d' % (j, k, n), mf.Expr.vstack([
              mf.Expr.constTerm(1), 
              mf.Expr.add(mf.Expr.mul(dist_jk[0], var_q.index(k, n, 0)))
              #mf.Expr.dot(dist_jk, mf.Expr.vstack([mf.Expr.sub(var_q.index(k, n, 0), var_q.index(j, n, 0)), mf.Expr.sub(var_q.index(k, n, 1), var_q.index(j, n, 1))])), 
              mf.Expr.constTerm(q[k][n][0] - q[j][n][0]), 
              mf.Expr.constTerm(q[k][n][1] - q[j][n][1]), 
              mf.Expr.constTerm(d_min)]), 
              mf.Domain.inRotatedQCone())
        '''

      #t[0] <= log(1 + gamma * inner_1)
      inner_1[k][n] = mf.Expr.sub(mf.Expr.mul(2 * a[k][n] / d[k][k][n], var_a.index(k, n)), mf.Expr.mul(a[k][n]**2 / d[k][k][n]**2, var_dist.index(k, k, n)))
      m.constraint('t0_%d_%d' % (k, n), mf.Expr.hstack(
          mf.Expr.add(1, mf.Expr.mul(gamma, inner_1[k][n])), 
          1, 
          var_t.index(k, n, 0)), mf.Domain.inPExpCone())
      
      #flight speed constraint
      if n < M - 1:
        m.constraint('speed_%d_%d' % (k, n), mf.Expr.vstack([mf.Expr.constTerm(v_max), mf.Expr.sub(var_q.index(k, n + 1, 0), var_q.index(k, n, 0)), mf.Expr.sub(var_q.index(k, n + 1, 1), var_q.index(k, n, 1))]), mf.Domain.inQCone())
  
      obj_expr = mf.Expr.add(obj_expr, mf.Expr.dot(var_t.slice([k, n, 0], [k + 1, n + 1, K]), [0.693147] + [-1 for x in range(K - 1)]))

  m.objective('obj_rate', mf.ObjectiveSense.Maximize, obj_expr)
  m.setSolverParam("intpntCoTolRelGap", 1.0e-5)
  #m.setSolverParam("intpntCoTolRelGap", 1.0e-8)

  #m.setLogHandler(sys.stdout)
  #m.writeTask('shen_sca.opf')
  m.solve()

  #print(m.getProblemStatus())
  #print("[opt] %f" % (m.primalObjValue()))
  #print(var_a.level(), var_q.level())
  return var_a, var_q, m.primalObjValue()

