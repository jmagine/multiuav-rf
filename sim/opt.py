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
  def __init__(self, K, M, GT, F, graph_max_edge_dist=0.5, Q=None, P=None, plotter=None):

    #init shen params
    self.d = np.zeros((K, K, M))
    self.I = np.zeros((K, M))
    self.gt = GT
    self.gamma = 0.01
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

    self.a = np.sqrt(self.p)

    #init graph for coloring based on UAV trajectory ending locations
    self.g_gt = nx.Graph()
    
    #freq init
    self.f = F

    #update graph based on trajectory endpoints
    self.max_color_dist = graph_max_edge_dist
    self.update_graph(self.q[:,M - 1,:])
    self.freq_assignments = self.color_graph(self.f)

    print('[shen] init SCA: k: %d m: %d gt: %s' % (K, M, GT))

    #determine initial R given p and q
    for n in range(M):
      for k in range(K):
        self.a[k][n] = math.sqrt(self.p[k][n])

        self.I[k][n] = 0
        for j in range(K):
          self.d[j][k][n] = utils.dist(self.q[j][n], self.gt[k])**2 + 0.1
          self.I[k][n] += self.gamma * self.a[j][n]**2 / self.d[j][k][n]
        self.I[k][n] -= self.gamma * self.a[k][n]**2 / self.d[k][k][n]
    self.shen_r = utils.shen_rpq(self.a, self.q, self.d, self.I, self.gt, self.gamma)
    print("[shen] R: %.2f" % (self.shen_r))

  def sca(self):
    K, M = self.a.shape
    itr = 0
    while True:
      var_a, var_q = optimize_shen(self.a, self.q, self.p, self.d, self.I, self.gt, self.gamma)
      
      #update a, q, p
      for idx in range(K * M):
        self.a[idx // M][idx % M] = var_a.level()[idx]
        self.p[idx // M][idx % M] = self.a[idx // M][idx % M]**2

        self.q[idx // M][idx % M][0] = var_q.level()[idx * 2]
        self.q[idx // M][idx % M][1] = var_q.level()[idx * 2 + 1]
      
      #update freq graph based on trajectory endpoints
      self.update_graph(self.q[:,M - 1,:])
      self.freq_assignments = self.color_graph(self.f)

      #update a, d, I with p, q, F
      #TODO update with F
      for n in range(M):
        for k in range(K):
          self.a[k][n] = math.sqrt(self.p[k][n])
          
          self.I[k][n] = 0
          for j in range(K):
            self.d[j][k][n] = utils.dist(self.q[j][n], self.gt[k])**2 + 0.1
            self.I[k][n] += self.gamma * self.a[j][n]**2 / self.d[j][k][n]
          self.I[k][n] -= self.gamma * self.a[k][n]**2 / self.d[k][k][n]

      #update R by eq 4 using shen_rpq
      shen_r_new = utils.shen_rpq(self.a, self.q, self.d, self.I, self.gt, self.gamma)
      #print("[shen] R: %.2f" % (shen_r_new))
      
      #visualize and print
      self.plotter.plot_traj(self.q, self.gt)
      '''
      for k in range(K):
        for n in range(M):
          if n % 5 == 0:
            print("[shen] q:[%.2f %.2f] a:%.2f" % (self.q[k][n][0], self.q[k][n][1], self.a[k][n]))
        print()
      '''

      #continue if prev r was 0
      if self.shen_r <= 0: 
        self.shen_r = shen_r_new
        continue

      #termination condition
      '''
      if ((shen_r_new - shen_r) / (shen_r)) < eps:
        print("term")
        time.sleep(30)
        break
      '''

      #update iteration
      self.shen_r = shen_r_new

      itr += 1
      if itr % 5 == 0:
        pass

    self.p = np.multiply(self.a, self.a)
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
    print("[upd graph] time:", t_end - t_start)

  def color_graph(self, freqs):

    t_start = time.time()

    freqs = list(sorted(freqs, reverse=True))
    print("[color] freqs:", freqs)

    range_factor = 10.0
    freq_assignments = [0]

    while 0 in freq_assignments:
      #make copy of graph
      g = self.g_gt.copy()
      max_vert_idx = len(g.nodes())

      #graphs
      g_freqs = []

      print("[color] range factor:", range_factor)

      #edge_ptr = 0
      freq_assignments = [0 for i in range(g.number_of_nodes())]
      #starting from lowest freq
      for f in freqs:
        #determine max dist threshold for current freq
        dist_thresh = range_factor / f**2

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
        
        print(cc_id)
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
              freq_assignments[lowest_vertex] = f

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

        #print(freq_assignments)
      
      #change range factor
      range_factor *= 0.95

    print(freq_assignments)
    t_end = time.time()
    print("[color] time:", t_end - t_start)

    self.plotter.plot_graph(g_freqs[0], self.gt, freq_assignments)
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
        d[j][k][n] = utils.dist(q[j][n], gt[k])**2 + 0.1
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
          d[j][k][n] = utils.dist(q[j][n], gt[k])**2 + 0.1
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

def optimize_shen(a, q, p, d, I, gt, gamma):
  d_min = 0.0
  v_max = 0.2
  #dist_pad = 100

  K, M = a.shape
  #update a, q by solving complex problem 23
  m = mf.Model('shen_sca')
  var_a = m.variable('a', [K, M], mf.Domain.inRange(0.01, 5))
  var_q = m.variable('q', [K, M, 2], mf.Domain.inRange(-1.0, 1.0))
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
        #var_dist >= sqrt(x_dist^2 + y_dist^2)
        m.constraint('dist_%d_%d_%d' % (j, k, n), mf.Expr.vstack([
            var_dist.index(j, k, n), 
            mf.Expr.sub(var_q.index(j, n, 0), gt[k][0]), 
            mf.Expr.sub(var_q.index(j, n, 1), gt[k][1])]), mf.Domain.inQCone())

        #inner_1[k][n] = mf.Expr.add(inner_1[k][n], mf.Expr.mul(-1.0 * gamma * p[j][n] / d[j][k][n]**2, var_dist.index(j, k, n)))
        inner_1[k][n] = mf.Expr.add(inner_1[k][n], mf.Expr.sub(mf.Expr.mul(2 * a[j][n] / d[j][k][n], var_a.index(j, n)), mf.Expr.mul(a[j][n]**2 / d[j][k][n]**2, var_dist.index(j, k, n))))

        #t[1] computations
        if j != k:
          inner_2[j][k][n] = mf.Expr.add(d[j][k][n], mf.Expr.dot(2 * (q[j][n] - gt[k]), mf.Expr.hstack([mf.Expr.sub(var_q.index(j, n, 0), q[j][n][0]), mf.Expr.sub(var_q.index(j, n, 1), q[j][n][1])])))
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
      m.constraint('t0_%d_%d' % (k, n), mf.Expr.hstack(
          mf.Expr.add(1, mf.Expr.mul(gamma, inner_1[k][n])), 
          1, 
          var_t.index(k, n, 0)), mf.Domain.inPExpCone())
      
      #flight speed constraint
      if n < M - 1:
        m.constraint('speed_%d_%d' % (k, n), mf.Expr.vstack([mf.Expr.constTerm(v_max), mf.Expr.sub(var_q.index(k, n + 1, 0), var_q.index(k, n, 0)), mf.Expr.sub(var_q.index(k, n + 1, 1), var_q.index(k, n, 1))]), mf.Domain.inQCone())
  
      obj_expr = mf.Expr.add(obj_expr, mf.Expr.dot(var_t.slice([k, n, 0], [k + 1, n + 1, K]), [1] + [-1 for x in range(K - 1)]))

  m.objective('obj_rate', mf.ObjectiveSense.Maximize, obj_expr)

  #m.setLogHandler(sys.stdout)
  #m.writeTask('shen_sca.opf')
  m.solve()

  #print(m.getProblemStatus())
  print("[opt] %f" % (m.primalObjValue()))
  #print(var_a.level(), var_q.level())
  return var_a, var_q

