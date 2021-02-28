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

import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.stats import norm
import EncodingSchemes as ES
import DecodingSchemes as DS
import FunctionGenerator as FG


if __name__ == '__main__':
    dt = 0.01
    T_max = 4
    time = np.arange(0, T_max, dt)

    S = list()
    S.append(FG.sum_of_sine_waves([2, -0.5, 0.75], [1.0, 3.0, 5.0], [0.0, 0.0, 0.0], 0.0, time))
    S.append(FG.sum_of_sine_waves([-0.25], [1.0], [0.0], 0.05, time))
    S.append(FG.sum_of_gaussian_wave([1, 0.5], [0.2, 0.75], [0.1, 0.1], 0.1, time))

    tbr_factors = [1.005, 1.005, 1.005]
    
    sf_thresholds = [0.35, 0.05, 0.35]
    
    mw_thresholds = [0.325, 0.015, 0.225]
    mw_window = [3, 3, 3]

    for i in range(len(S)):

        spikes_TBR, threshold = ES.temporal_contrast(S[i], tbr_factors[i])
        signal_TBR = 2*DS.temporal_contrast(spikes_TBR, threshold)

        spikes_SF, startpoint = ES.step_forward(S[i], sf_thresholds[i])
        signal_SF = DS.step_forward(spikes_SF, sf_thresholds[i], startpoint)

        spikes_MW, startpoint = ES.moving_window(S[i], mw_thresholds[i], mw_window[i])
        signal_MW = DS.moving_window(spikes_MW, mw_thresholds[i], startpoint)

        plt.subplot(3*len(S),3,(1+i*3*len(S),4+i*3*len(S)))
        plt.plot(time, S[i])
        plt.plot(time, signal_TBR)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        if i == 0:
            plt.title("Temporal Contrast Algorithm TBR")

        plt.subplot(3*len(S),3,7+i*3*len(S))
        plt.stem(time, spikes_TBR)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)

        plt.subplot(3*len(S),3,(2+i*3*len(S),5+i*3*len(S)))
        plt.plot(time, S[i])
        plt.plot(time, signal_SF)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        if i == 0:
            plt.title("Step Forward Algorithm SF")

        plt.subplot(3*len(S),3,8+i*3*len(S))
        plt.stem(time, spikes_SF)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)

        plt.subplot(3*len(S),3,(3+i*3*len(S),6+i*3*len(S)))
        plt.plot(time, S[i])
        plt.plot(time, signal_MW)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        if i == 0:
            plt.title("Moving Window Algorithm MW")

        plt.subplot(3*len(S),3,9+i*3*len(S))
        plt.stem(time, spikes_MW)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        
    plt.show()

    hsa_window = [12, 15, 12]
    hsa_fir = list()
    hsa_fir.append(signal.triang(hsa_window[0]))
    hsa_fir.append(norm.pdf(np.linspace(1, hsa_window[1], hsa_window[1]), 0, 5))
    hsa_fir.append(signal.triang(hsa_window[2]))

    hsa_m_thresholds = [0.85, 0.05, 0.5]

    bsa_window = [9, 10, 8]
    bsa_fir = list()
    bsa_fir.append(signal.triang(bsa_window[0]))
    bsa_fir.append(norm.pdf(np.linspace(1, bsa_window[1], bsa_window[1]), 1.5, 3.5))
    bsa_fir.append(signal.triang(bsa_window[2]))

    bsa_thresholds = [1.175, 1.05, 1.2]

    for i in range(len(S)):

        spikes_HSA, shift = ES.hough_spike(S[i], hsa_fir[i])
        signal_HSA = DS.ben_spike(spikes_HSA, hsa_fir[i], shift)

        spikes_HSAm, shift = ES.modified_hough_spike(S[i], hsa_fir[i], hsa_m_thresholds[i])
        signal_HSAm = DS.ben_spike(spikes_HSAm, hsa_fir[i], shift)

        spikes_BSA, shift = ES.ben_spike(S[i], bsa_fir[i], bsa_thresholds[i])
        signal_BSA = DS.ben_spike(spikes_BSA, bsa_fir[i], shift)

        plt.subplot(3*len(S),3,(1+i*3*len(S),4+i*3*len(S)))
        plt.plot(time, S[i])
        plt.plot(time, signal_HSA)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        if i == 0:
            plt.title("Hough Spike Algorithm HSA")

        plt.subplot(3*len(S),3,7+i*3*len(S))
        plt.stem(time, spikes_HSA)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)

        plt.subplot(3*len(S),3,(2+i*3*len(S),5+i*3*len(S)))
        plt.plot(time, S[i])
        plt.plot(time, signal_HSAm)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        if i == 0:
            plt.title("Threshold Hough Spike Algorithm T-HSA")

        plt.subplot(3*len(S),3,8+i*3*len(S))
        plt.stem(time, spikes_HSAm)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)

        plt.subplot(3*len(S),3,(3+i*3*len(S),6+i*3*len(S)))
        plt.plot(time, S[i])
        plt.plot(time, signal_BSA)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        if i == 0:
            plt.title("Ben Spike Algorithm BSA")

        plt.subplot(3*len(S),3,9+i*3*len(S))
        plt.stem(time, spikes_BSA)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        
    plt.show()


    for i in range(len(S)):

        number_of_neurons = 15
        [min_input, max_input] = [min(S[i]), max(S[i])]
        spikes_GRF_1 = ES.grf_spike(S[i], number_of_neurons, min_input, max_input)
        signal_GRF_1 = DS.grf_spike(spikes_GRF_1, min_input, max_input)

        plt.subplot(3*len(S),3,(1+i*3*len(S),4+i*3*len(S)))
        plt.plot(time, S[i])
        plt.plot(time, signal_GRF_1)
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        if i == 0:
            plt.title("Gaussian Receptive Fields GRF")

        plt.subplot(3*len(S),3,7+i*3*len(S))
        for k in range(len(S[i])):
            plt.plot([time[k], time[k]], [np.argmax(spikes_GRF_1[k,:])-0.35, np.argmax(spikes_GRF_1[k,:])+0.35])
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)

    plt.show()