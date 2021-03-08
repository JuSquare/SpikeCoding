#! /usr/bin/env python

# This file is part of the SpikeCoding repository - MAVLab TU Delft
#
# MIT License
#
# Copyright (c) 2021 Julien Dupeyroux
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# @author Julien Dupeyroux

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

def grf_spike(spikes, min_input, max_input):
    shape = spikes.shape
    signal = np.zeros(shape[0])
    for i in range(shape[0]):
        signal[i] = min_input + (2*(np.argmax(spikes[i,:]) + 1)-3)/2*(max_input - min_input)/(shape[1]-2)
    return signal