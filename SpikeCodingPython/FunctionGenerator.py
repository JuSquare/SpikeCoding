import numpy as np
import math
from scipy.stats import norm


def noisy_sine_wave(a, f, phi, sigma, time):
    mySin  = np.vectorize(math.sin)
    return a*mySin(2*math.pi*f*time + phi*np.ones(len(time))) + np.random.rand(len(time))*sigma


def sum_of_sine_waves(a, f, phi, sigma, time):
    signal = np.zeros(len(time))
    for j in range(len(a)):
        signal = signal + noisy_sine_wave(a[j], f[j], phi[j], sigma, time)
    return signal


def noisy_gaussian_wave(a, m, s, sigma, time):
    return a*norm.pdf(time, m, s) + sigma*np.random.rand(len(time))


def sum_of_gaussian_wave(a, m, s, sigma, time):
    signal = np.zeros(len(time))
    for j in range(len(a)):
        signal = signal + noisy_gaussian_wave(a[j], m[j], s[j], 0, time) + np.random.rand(len(time))*sigma
    return signal
