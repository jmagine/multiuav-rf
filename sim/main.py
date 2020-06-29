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
GRAPH_MAX_EDGE_DIST = 10000.0
M = 30 #num time steps
B = 10 * 10**6 #bandwidth in hz
#B = 1.25 * 10**6 #FDMA

#F = [5.160, 5.180, 5.200, 5.220, 2.412, 2.437, 2.462, 2.484, 0.915]
#F = [5.18, 5.2, 5.22, 2.412, 2.437, 0.915]
#F = [5.22, 5.20, 5.18, 2.412, 0.915]
#F = [2.412, 2.437, 2.462, 2.484, 0.915]
#F = [2.412, 2.462, 0.915]
#F = [2.4, 0.9, 0.4]
#F = [2.4]

F = [5.2, 2.4, 0.9] 
#F = [5.220] #Fixed freq
#F = [5.16, 5.18, 5.20, 5.22, 5.24, 5.26, 5.28, 5.30] #FDMA

H_MIN = 100 #min height in meters
V_MAX = 50 #max velocity in meters

P_BOUNDS = [[-500, 500], [-500, 500]]

PLOT_VOR = False
PLOT_TRAJ = True
PLOT_GRAPH = True

if __name__ == "__main__":
  e = env.env(n_drones=N_DRONES, p_bounds=P_BOUNDS, M=M, F=F, v_max=V_MAX)
  plotter = vis.plotter(pos_bounds=P_BOUNDS, plot_vor=PLOT_VOR, plot_traj=PLOT_TRAJ, plot_graph=PLOT_GRAPH)
  e.setup()
  o = opt.ma_sca(N_DRONES, M, e.gt, B, e.F, v_max=V_MAX, h_min=H_MIN, graph_max_edge_dist=GRAPH_MAX_EDGE_DIST, Q=e.init_q, P=e.init_p, plotter=plotter)
  o.sca()

  for i in range(1000):
    #e.tick()
    pass