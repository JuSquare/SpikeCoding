import numpy as np


def temporal_contrast(spikes, threshold):
    # Based on algorithm provided in:
    #   Sengupta et al. (2017)
    #   Petro et al. (2020)
    signal = np.zeros(len(spikes))
    for i in range(1, len(spikes)):
        if spikes[i] > 0:
            signal[i] = signal[i-1] + threshold
        elif spikes[i] < 0:
            signal[i] = signal[i-1] - threshold
        else:
            signal[i] = signal[i-1]
    return signal

def step_forward(spikes, threshold, startpoint):
    # Based on algorithm provided in:
    #   Petro et al. (2020)
    signal = np.zeros(len(spikes))
    signal[0] = startpoint
    for i in range(1,len(spikes)):
        if spikes[i] > 0:
            signal[i] = signal[i-1] + threshold
        elif spikes[i] < 0:
            signal[i] = signal[i-1] -threshold
        else:
            signal[i] = signal[i-1]
    return signal

def moving_window(spikes, threshold, startpoint):
    # Based on algorithm provided in:
    #   Petro et al. (2020)
    signal = np.zeros(len(spikes))
    signal[0] = startpoint
    for i in range(1,len(spikes)):
        if spikes[i] > 0:
            signal[i] = signal[i-1] + threshold
        elif spikes[i] < 0:
            signal[i] = signal[i-1] - threshold
        else:
            signal[i] = signal[i-1]
    return signal

def ben_spike(spikes, fir, shift):
    # Based on algorithm provided in:
    #   Petro et al. (2020)
    #   Sengupta et al. (2017)
    #   Schrauwen et al. (2003)
    signal = np.convolve(spikes, fir)
    signal = signal + shift*np.ones(len(signal))
    signal = signal[0:(len(signal)-len(fir)+1)]
    return signal