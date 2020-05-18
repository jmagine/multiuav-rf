

import numpy as np
import math

def calc_rates(a, q, f, d, I, gt, gamma):
  K, M = I.shape

  #r_temp = np.zeros((K))
  #r_total = np.zeros((M))
  #r_total_k = np.zeros((K))

  r = np.zeros((K, M))

  for freq in f:
    #uav_idxs = [idx for idx, fr in enumerate(f) if fr == freq]
    uav_idxs = f[freq]

    for n in range(M):
      for k in uav_idxs:
        inner_signal = 1
        inner_noise = 1

        inner_signal += gamma * a[k][n]**2 / d[k][k][n]
        for j in uav_idxs:
          #inner_signal += gamma * a[j][n]**2 / d[j][k][n]

          if j != k:
            inner_noise += gamma * a[j][n]**2 / d[j][k][n]

        r[k][n] = math.log(inner_signal, 2) - math.log(inner_noise, 2)

    '''
    #nth time slot
    for n in range(M):
      
      r_total[n] = 0
      #kth uav-gt pair
      for k in uav_idxs:
        #interference over all other uavs for kth pair
        r_temp[k] = I[k][n] + gamma * a[k][n]**2 / d[k][k][n]
        
        #calculate rate for kth pair at time n
        r_total[n] += math.log(1 + gamma * a[k][n]**2 / d[k][k][n]) - math.log(1 + I[k][n])
        r_total_k[k] += math.log(1 + gamma * a[k][n]**2 / d[k][k][n]) - math.log(1 + I[k][n])
        #r_total[n] += math.log(1 + r_temp[k]) - math.log(1 + I[k][n])
        #r_total_k[k] += math.log(1 + r_temp[k]) - math.log(1 + I[k][n])
    '''
  #for k in range(K):
  #  print("[rpq] %d: %.2f" % (k, r_total_k[k]))
  '''
  for n in range(M):
    print("(%d,%.2f)" % (n, np.sum(r, axis=0)[n] * 1.44), end='')
  print()
  '''
  return r

def dist(pos_0, pos_1):
  return np.linalg.norm(np.array(pos_0) - np.array(pos_1))

def dist2(pos_0, pos_1, h_min):
  p0 = [pos_0[0], pos_0[1], h_min]
  p1 = [pos_1[0], pos_1[1], 0]
  return np.linalg.norm(np.array(p0) - np.array(p1))

#calculate received signal power in dB
def power_fspl(pw_tx, freq, dist):
  if dist <= 0 or freq <= 0:
    return pw_tx

  loss = 20 * math.log10(4 * math.pi / 300 * freq * dist)
  return pw_tx - loss