import matplotlib.pyplot as plt

VIEW_BORDER = 0.1

plt.ion()

class plotter():
  def __init__(self, pos_bounds, plot_vor, plot_traj):
    self.p_bounds = pos_bounds
    self.plot_vor_bool = plot_vor
    self.plot_traj_bool = plot_traj

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
    plt.pause(0.01)

    #print(self.p_bounds[0][0] - VIEW_BORDER, self.p_bounds[0][1] + VIEW_BORDER, self.p_bounds[1][0] - VIEW_BORDER, self.p_bounds[1][1] + VIEW_BORDER)
    #print(self.ax_traj.get_ylim(), self.ax_traj.get_xlim())