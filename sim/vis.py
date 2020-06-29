import matplotlib.pyplot as plt

VIEW_BORDER = 0.1

plt.ion()

class plotter():
  def __init__(self, pos_bounds, plot_vor, plot_traj, plot_graph):
    self.p_bounds = pos_bounds
    self.plot_vor_bool = plot_vor
    self.plot_traj_bool = plot_traj
    self.plot_graph_bool = plot_graph

    if self.plot_vor_bool:
      self.fig_vor = plt.figure(figsize=(16, 16), dpi=100)
      self.ax_vor = self.fig_vor.add_subplot(1,1,1)
      self.ax_vor.axis('equal')
      self.ax_vor.set_xlim([self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER])
      self.ax_vor.set_ylim([self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER])

    self.fig_opt = plt.figure(figsize=(8,4), dpi=100)

    #freq selection
    if self.plot_graph_bool:
      #self.fig_graph = plt.figure(figsize=(4, 4), dpi=100)
      self.ax_graph = self.fig_opt.add_subplot(1,2,1)
      self.ax_graph.set_xlim([self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER])
      self.ax_graph.set_ylim([self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER])
      self.ax_graph.tick_params(
        axis='both',
        which='both',
        top=False,
        bottom=False,
        left=False,
        right=False,
        labeltop=False,
        labelbottom=False,
        labelleft=False,
        labelright=False
      )

    #trajectories
    if self.plot_traj_bool:
      #self.fig_traj = plt.figure(figsize=(4, 4), dpi=100)
      self.ax_traj = self.fig_opt.add_subplot(1,2,2)
      self.ax_traj.set_xlim([self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER])
      self.ax_traj.set_ylim([self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER])
      self.ax_traj.tick_params(
        axis='both',
        which='both',
        top=False,
        bottom=False,
        left=False,
        right=False,
        labeltop=False,
        labelbottom=False,
        labelleft=False,
        labelright=False
      )

  def plot_vor(self, drones, centroids, vor):
    self.fig_vor.clf()
    ax = self.fig_vor.gca()
    
    # Plot drone points
    ax.scatter([d.pos[0] for d in drones], [d.pos[1] for d in drones], marker='x', color='b')
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
      ax.scatter(centroids[:, 0], centroids[:, 1], marker='.', color='r')

    plt.show()
    plt.pause(0.01)

  def plot_traj(self, q, gt):

    if not self.plot_traj_bool:
      return
    #self.fig_traj.clf()
    #ax = self.fig_traj.gca()
    self.ax_traj.cla()
    ax = self.ax_traj

    # Plot drone points
    for k in range(len(q)):
      #print(q[k, :, 0], q[k, :, 1])

      ax.plot(q[k, :, 0], q[k, :, 1], marker='.', ms=2, color='blue', linewidth=0.25)
      ax.scatter(gt[k][0], gt[k][1], marker='.',s=64, color='blue')

      #ax.plot(q[k, :, 0], q[k, :, 1], marker='.', ms=2, color='C%d' % (k % 8), linewidth=0.25)
      #ax.scatter(gt[k][0], gt[k][1], marker='.',s=64, color='C%d' % (k % 8))

    ax.set_xlim([self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER])
    ax.set_ylim([self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER])
    #for k in range(len(gt)):
    #  ax.scatter(gt[k])
    #ax.set_aspect('equal', 'box')
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    #self.ax_traj.set_xlim([self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER])
    #self.ax_traj.set_ylim([self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER])

    plt.show()
    plt.pause(0.01)

    #print(self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER, self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER)
    #print(self.ax_traj.get_ylim(), self.ax_traj.get_xlim())

  #plot gt positions, graph edges, freq assignments
  def plot_graph(self, g, gt, freqs):
    if not self.plot_graph_bool:
      return
    #self.fig_graph.clf()
    #ax = self.fig_graph.gca()
    self.ax_graph.cla()
    ax = self.ax_graph

    ax.set_xlim([self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER])
    ax.set_ylim([self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER])
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    #plot gt and target positions
    #unique_freqs = list(set(freqs))

    for k in range(len(gt)):
      size = 64
      if freqs[k] == 0.0:
        color = 'red'
      elif freqs[k] > 5.0:
        color = 'gold'
      elif freqs[k] > 2.4:
        color = 'green'
      else:
        color = 'blue'
      #size = 128
      ax.scatter(gt[k][0], gt[k][1], marker='x', s=5, c=color)
      ax.scatter(g.nodes(data='p')[k][0], g.nodes(data='p')[k][1], marker='.', s=size, c=color)

    #'''
    #plot graph edges
    for u, v in g.edges:
      u_pos = g.nodes(data='p')[u]
      v_pos = g.nodes(data='p')[v]

      x_pos = [u_pos[0], v_pos[0]]
      y_pos = [u_pos[1], v_pos[1]]
      ax.plot(x_pos, y_pos, c='black', linewidth=0.25)
      #print(g.nodes(data='p')[u], g.nodes(data='p')[v])
    #'''
    

    #TODO plot freq assignments

    plt.show()
    plt.pause(0.01)