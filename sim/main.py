'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Mar 08 2020
                                      SARC

  File Name  : main.py
  Description: Entry point for sim
---*-----------------------------------------------------------------------*'''

import env
import poi
import bs
import drone
import vis

import opt

#sim params
N_DRONES = 8

#variables for tuning
GRAPH_MAX_EDGE_DIST = 1.0
M = 20

P_BOUNDS = [[-1, 1], [-1, 1]]

PLOT_VOR = False
PLOT_TRAJ = True
PLOT_GRAPH = True

if __name__ == "__main__":
  e = env.env(n_drones=N_DRONES, p_bounds=P_BOUNDS, M=M)
  plotter = vis.plotter(pos_bounds=P_BOUNDS, plot_vor=PLOT_VOR, plot_traj=PLOT_TRAJ, plot_graph=PLOT_GRAPH)
  e.setup()
  o = opt.ma_sca(N_DRONES, M, e.gt, e.F, graph_max_edge_dist=GRAPH_MAX_EDGE_DIST, Q=e.init_q, P=e.init_p, plotter=plotter)
  o.sca()

  for i in range(1000):
    #e.tick()
    pass