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