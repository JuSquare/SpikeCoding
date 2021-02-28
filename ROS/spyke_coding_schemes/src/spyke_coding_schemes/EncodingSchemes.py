import numpy as np
from scipy.stats import norm


def temporal_contrast(data, factor):
    # Based on algorithm provided in:
    #   Sengupta et al. (2017)
    #   Petro et al. (2020)
    diff = np.zeros(len(data)-1)
    spikes = np.zeros(len(data))
    for i in range(len(data)-1):
        diff[i] = data[i+1] - data[i]
    threshold = np.mean(diff) + factor * np.std(diff)
    diff = np.insert(diff, 0, diff[1])
    for i in range(len(data)):
        if diff[i] > threshold:
            spikes[i] = 1
        elif diff[i] < -threshold:
            spikes[i] = -1
    return spikes, threshold


def step_forward(data, threshold):
    # Based on algorithm provided in:
    #   Petro et al. (2020)
    startpoint = data[0]
    spikes = np.zeros(len(data))
    base = startpoint
    for i in range(1,len(data)):
        if data[i] > base + threshold:
            spikes[i] = 1
            base = base + threshold
        elif data[i] < base - threshold:
            spikes[i] = -1
            base = base - threshold
    return spikes, startpoint


def moving_window(data, threshold, window):
    # Based on algorithm provided in:
    #   Petro et al. (2020)
    startpoint = data[0]
    spikes = np.zeros(len(data))
    base = np.mean(data[0:window+1])
    for i in range(window+1):
        if data[i] > base + threshold:
            spikes[i] = 1
        elif data[i] < base - threshold:
            spikes[i] = -1
    for i in range(window+2, len(data)):
        base = np.mean(data[(i-window-1):(i-1)])
        if data[i] > base + threshold:
            spikes[i] = 1
        elif data[i] < base - threshold:
            spikes[i] = -1
    return spikes, startpoint


def hough_spike(data, fir):
    # Based on algorithm provided in:
    #   Schrauwen et al. (2003)
    spikes = np.zeros(len(data))
    shift = min(data)
    data = data - shift*np.ones(len(data))
    for i in range(len(data)):
        count = 0
        for j in range(len(fir)):
            if i+j < len(data):
                if data[i+j] >= fir[j]:
                    count = count + 1
        if count == len(fir):
            spikes[i] = 1
            for j in range(len(fir)):
                if i+j < len(data):
                    data[i+j] = data[i+j] - fir[j]
    return spikes, shift


def modified_hough_spike(data, fir, threshold):
    # Based on algorithm provided in:
    #   Schrauwen et al. (2003)
    spikes = np.zeros(len(data))
    shift = min(data)
    data = data - shift*np.ones(len(data))
    for i in range(len(data)):
        error = 0
        for j in range(len(fir)):
            if i+j < len(data):
                if data[i+j] < fir[j]:
                    error = error + fir[j] - data[i+j]
        if error <= threshold:
            spikes[i] = 1
            for j in range(len(fir)):
                if i+j < len(data):
                    data[i+j] = data[i+j] - fir[j]
    return spikes, shift


def ben_spike(data, fir, threshold):
    # Based on algorithm provided in:
    #   Petro et al. (2020)
    #   Sengupta et al. (2017)
    #   Schrauwen et al. (2003)
    spikes = np.zeros(len(data))
    shift = min(data)
    data = data - shift*np.ones(len(data))
    for i in range(len(data)-len(fir)+1):
        err1 = 0
        err2 = 0
        for j in range(len(fir)):
            err1 = err1 + abs(data[i+j] - fir[j])
            err2 = err2 + abs(data[i+j-1])
        if err1 <= err2*threshold:
            spikes[i] = 1
            for j in range(len(fir)):
                if i+j+1 < len(data):
                    data[i+j+1] = data[i+j+1] - fir[j]
    return spikes, shift


def grf_spike(data, m, min_input, max_input):
    # Adapted from algorithm provided in:
    #   Bohté et al. (2002)
    # Modifications: definition of sigma, removal of beta constant,
    #                and modified WTA process
    if np.isscalar(data):
        spikes = np.zeros(m)
        neuron_outputs = np.zeros(m)
        for i in range(m):
            mu = min_input + (2*i-3)/2*(max_input - min_input)/(m-2)
            sigma = (max_input - min_input)/(m-2)
            neuron_outputs[i] = norm.pdf(data, mu, sigma)
        spikes[np.argmax(neuron_outputs)] = 1
    else: 
        spikes = np.zeros((len(data),m))
        neuron_outputs = np.zeros(m)
        for j in range(len(data)):
            for i in range(m):
                mu = min_input + (2*i-3)/2*(max_input - min_input)/(m-2)
                sigma = (max_input - min_input)/(m-2)
                neuron_outputs[i] = norm.pdf(data[j], mu, sigma)
            spikes[j,np.argmax(neuron_outputs)] = 1
    return spikes
