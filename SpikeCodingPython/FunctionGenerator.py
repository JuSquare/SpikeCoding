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
