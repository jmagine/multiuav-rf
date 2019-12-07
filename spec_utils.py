'''*-----------------------------------------------------------------------*---
                                                         Author: Jason Ma
                                                         Date  : Dec 03 2019
                                      SARC

  File Name  : spec_utils.py
  Description: Utility functions for spectrum analysis
---*-----------------------------------------------------------------------*'''

import matplotlib.pyplot as plt
import numpy as np
import struct
import time
from scipy import signal

'''[Global vars]------------------------------------------------------------'''
FILE_IQ_SAMP = 'rx.dat'
#FILE_IQ_SAMP = 'cap_30000000_2415000000.0_48.0.dat'
FREQ_SAMP = 25e6
N_SAMP = 3000
PLOT = True

t_start = time.time()

'''[read_binary]---------------------------------------------------------------
  Read binary IQ samples from file
  filename - path to file
  n_samples - number of samples to read
  return - array containing complex IQ samples
----------------------------------------------------------------------------'''
def read_binary(filename, n_samples):
  print('[%f][rb] start' % (time.time() - t_start))
  s = np.fromfile(filename, count=n_samples, dtype=np.complex64)
  print('[%f][rb] complete' % (time.time() - t_start))
  return s

'''[spectrogram]---------------------------------------------------------------
  Return a spectrogram of IQ samples
  samples - IQ samples
  fs - sampling frequency
  return - frequencies, times, power array indexed over freqs and times
----------------------------------------------------------------------------'''
def spectrogram(samples, fs):
  print('[%f][sg] start' % (time.time() - t_start))
  f,t,sxx = signal.spectrogram(samples, fs=fs, return_onesided=False)
  print('[%f][sg] complete' % (time.time() - t_start))
  return f, t, sxx

'''[analyze]-------------------------------------------------------------------
  Analysis function for spectrum
  f - frequencies
  t - times
  sxx - powers indexed by freqs and times
----------------------------------------------------------------------------'''
def analyze(f, t, sxx, plot):
  print('[%f][a ] start' % (time.time() - t_start))

  sxx_binary = sxx.copy()
  sxx_max = sxx.copy()
  thresh = np.percentile(sxx, 95)
  print(thresh)
  #find min/max values in each time instance
  print(np.argmax(sxx, axis=0))
  print(np.max(sxx, axis=0))

  print(np.argmin(sxx, axis=0))
  print(np.min(sxx, axis=0))

  #TODO redundant
  #determine min and max freqs for each time step
  for f_i in range(len(sxx)):
    max_val = -1e9
    max_t = -1

    for t_i in range(len(sxx[f_i])):
      if sxx[f_i][t_i] > max_val:
        max_val = sxx[f_i][t_i]
        max_t = t[t_i]
    if max_val > thresh:
      print("f: %E max_t: %E max_val: %E" % (f[f_i], max_t, max_val))
      for i in range(len(sxx[f_i])):
        sxx_binary[f_i][i] = 1
    
    for i in range(len(sxx[f_i])):
      sxx_max[f_i][i] = max_val
  
  print('[%f][a ] complete' % (time.time() - t_start))

  #plot spectrogram
  if plot:
    plt.figure()
    plt.pcolormesh(np.fft.fftshift(f), t, np.transpose(np.fft.fftshift(sxx, axes=0)))
    plt.ylabel("Time")
    plt.xlabel("Freq")

    plt.figure()
    plt.pcolormesh(np.fft.fftshift(f), t, np.transpose(np.fft.fftshift(sxx_binary, axes=0)))
    plt.ylabel("Time")
    plt.xlabel("Freq")

    plt.figure()
    plt.pcolormesh(np.fft.fftshift(f), t, np.transpose(np.fft.fftshift(sxx_max, axes=0)))
    plt.ylabel("Time")
    plt.xlabel("Freq")
    plt.show()


s = read_binary(FILE_IQ_SAMP, N_SAMP)

#TODO add in a filter step where only blocks of samples are returned?
#s = filter_samples(s)

f, t, sxx = spectrogram(s, FREQ_SAMP)
analyze(f, t, sxx, PLOT)