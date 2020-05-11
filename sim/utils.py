

import numpy as np
import math

def shen_rpq(a, q, d, I, gt, gamma):
  K, M = I.shape

  r_temp = np.zeros((K))
  r_total = np.zeros((M))
  r_total_k = np.zeros((K))

  #nth time slot
  for n in range(M):
    
    r_total[n] = 0
    #kth uav-gt pair
    for k in range(K):
      #interference over all other uavs for kth pair
      r_temp[k] = I[k][n] + gamma * a[k][n]**2 / d[k][k][n]
      
      #calculate rate for kth pair at time n
      r_total[n] += math.log(1 + gamma * a[k][n]**2 / d[k][k][n]) - math.log(1 + I[k][n])
      r_total_k[k] += math.log(1 + gamma * a[k][n]**2 / d[k][k][n]) - math.log(1 + I[k][n])
      #r_total[n] += math.log(1 + r_temp[k]) - math.log(1 + I[k][n])
      #r_total_k[k] += math.log(1 + r_temp[k]) - math.log(1 + I[k][n])

  for k in range(K):
    print("[rpq] %d: %.2f" % (k, r_total_k[k]))
  return sum(r_total)

def dist(pos_0, pos_1):
  return np.linalg.norm(np.array(pos_0) - np.array(pos_1))

#calculate received signal power in dB
def power_fspl(pw_tx, freq, dist):
  if dist <= 0 or freq <= 0:
    return pw_tx

  loss = 20 * math.log10(4 * math.pi / 300 * freq * dist)
  return pw_tx - loss