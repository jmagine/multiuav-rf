'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Oct 18 2018
                                      TODO

  File Name  : pid.py
  Description: TODO
---*-----------------------------------------------------------------------*'''

import time
import matplotlib.animation as anim
import matplotlib.pyplot as plt
import threading
import math
import numpy as np

'''[Global Vars]------------------------------------------------------------'''
ORIGIN_X = 0.0
ORIGIN_Y = 0.0
C_R = 10

#plt.autoscale(enable=True, axis="both")

fig = plt.figure()
ax = fig.add_subplot(2,1,1)
ax2 = fig.add_subplot(2,1,2)
scat = ax.scatter([], [])
ax.set_xlim([-1 * C_R - 1, C_R + 1])
ax.set_ylim([-1 * C_R - 1, C_R + 1])
scat.set_facecolors(['g', 'r'])
scat.set_sizes([31, 31])
prev_time = time.time()
vel = np.array([0.0, 0.0])

errors = [0, 1]
error_plot, = ax2.plot([i for i in range(len(errors))], errors, color="g")

class drone(): 
  def __init__(self, p, vel):
    self.pos = np.array(p)
    self.v = np.array(vel)
    self.prev_error = np.zeros((2))
    self.integral = np.zeros((2))
    self.dt = 0.01
    self.kp = 0.8 * 2.0
    self.ki = 0
    self.kd = 0
    
    #self.ki = 2.0 * self.kp / 2.0
    #self.kd = self.kp * 2.0 / 8.0
    
    #self.ki = 2 * self.kp / 1.0 
    #self.kd = self.kp * 0.01 / 8
  
  def callback(self):
    pass

  def run(self, ref_pos, vx=None, vy=None):
    self.pos += self.v

    #print(self.integral)

    if vx:
      self.v[0] = vx
    
    if vy:
      self.v[1] = vy

    #compute PID output
    error = ref_pos - self.pos
    
    self.integral = self.integral * 0.99 + error * self.dt
    '''
    for i in range(2):
      if self.integral[i] > 1:
        self.integral[i] = 1
      elif self.integral[i] < -1:
        self.integral[i] = -1
    '''
    #print(self.integral)

    derivative = (error - self.prev_error) / self.dt
    
    for i in range(2):
      if derivative[i] > 0.1:
        derivative[i] = 0.1
      elif derivative[i] < -0.1:
        derivative[i] = -0.1
    self.prev_error = error
    pid_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
    print(self.pos, pid_output, self.kp * error, self.ki * self.integral, self.kd * derivative)
    #print(error[0])
    #errors.append(error[0])

    return pid_output

d = drone([ORIGIN_X + C_R, ORIGIN_Y], [0.0, 0.0])

def dist(x1, y1, x2, y2):
  return ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))**(1/2)

def dist(p1, p2):
  assert len(p1) == len(p2)
  dims = len(p1)
  
  total = 0
  for i in range(dims):
    total += (p2[i] - p1[i]) * (p2[i] - p1[i])

  return (total)**(1/2)

#def pid_angle(x, y, ref_x, ref_y, d):
#  return math.atan(-1 * (C_R - dist(x, y, ORIGIN_X, ORIGIN_Y)) / d) + math.atan((y - ORIGIN_Y) / (x - ORIGIN_X)) + math.pi / 2

def ref(t):
  return np.array([ORIGIN_X + C_R * math.cos(t), ORIGIN_Y + C_R * math.sin(t)])

def update(i):
  global prev_time, vel
  #update reference point position
  curr_time = time.time()
  ref_point = ref(i / 25.0)
  #ref_x = ref_point[0]
  #ref_y = ref_point[1]
  out = d.run(ref_point)
  
  for i in range(2):
    if out[i] > 10 or out[i] < -10:
      out = out * 10 / out[i]

  #print(d.pos, out)

  d.v = out

  while time.time() - prev_time < d.dt:
    time.sleep(d.dt / 10)
  
  prev_time = time.time()
  #print the desired angle of drone
  #pid_ang = pid_angle(d.x, d.y, ref_point[0], ref_point[1], 0.05)
  #print(math.cos(pid_ang), math.sin(pid_ang))
  #d.run(math.cos(pid_ang), math.sin(pid_ang))
  
  scat.set_offsets([[ref_point[0], ref_point[1]], [d.pos[0], d.pos[1]]])

  errors.append(dist(ref_point, d.pos))
  error_plot.set_xdata([i for i in range(len(errors))])
  error_plot.set_ydata(errors)
  ax2.set_xlim([-1, len(errors) + 1])
  ax2.set_ylim([1, min(errors)])

def main():
  d = drone(ORIGIN_X + C_R, ORIGIN_Y, 1)
  

if __name__ == '__main__':
  #main()
  a = anim.FuncAnimation(fig, update, range(1000), interval=1, blit=False, repeat=False)
  plt.show()


