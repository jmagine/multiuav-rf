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

N_DRONES = 4
M = 20

P_BOUNDS = [[-1, 1], [-1, 1]]

PLOT_VOR = False
PLOT_TRAJ = True

if __name__ == "__main__":
  e = env.env(n_drones=N_DRONES, p_bounds=P_BOUNDS, M=M)
  plotter = vis.plotter(pos_bounds=P_BOUNDS, plot_vor=PLOT_VOR, plot_traj=PLOT_TRAJ)
  e.setup()
  o = opt.ma_sca(N_DRONES, M, e.gt, e.F, e.init_q, e.init_p, plotter=plotter)
  o.sca()

  for i in range(1000):
    #e.tick()
    pass