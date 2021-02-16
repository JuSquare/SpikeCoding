import numpy as np
import math


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

