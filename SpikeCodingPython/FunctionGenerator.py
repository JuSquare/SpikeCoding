import numpy as np
import math
from scipy.stats import norm


def noisy_sine_wave(a, f, phi, sigma, time):
    signal = np.zeros(len(time))
    for i in range(len(time)):
        signal[i] = a*math.sin(2*math.pi*f*time[i] + phi) + np.random.rand(1)*sigma
    return signal


def sum_of_sine_waves(a, f, phi, sigma, time):
    signal = np.zeros(len(time))
    for i in range(len(time)):
        for j in range(len(a)):
            signal[i] = signal[i] + a[j]*math.sin(2*math.pi*f[j]*time[i] + phi[j]) + np.random.rand(1)*sigma
    return signal


def noisy_gaussian_wave(a, m, s, sigma, time):
    return a*norm.pdf(time, m, s) + sigma*np.random.rand(len(time))


def sum_of_gaussian_wave(a, m, s, sigma, time):
    signal = np.zeros(len(time))
    normFunctions = list()
    
    for j in range(len(a)):
        normFunctions.append(noisy_gaussian_wave(a[j], m[j], s[j], 0, time))
    
    for i in range(len(time)):
        for j in range(len(a)):
            signal[i] = signal[i] + normFunctions[j][i] + np.random.rand(1)*sigma
    return signal
