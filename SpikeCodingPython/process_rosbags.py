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

import rosbag
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import DecodingSchemes as DS

if __name__ == '__main__':

    if len(sys.argv) != 2:
        sys.exit("Usage: python process_rosbags.py <bagfile.bag>")

    bag = rosbag.Bag(sys.argv[1])

    events = list()
    signal = list()
    time = list()
    threshold = list()

    min_input = None 
    max_input = None 
    startpoint = None 
    coding = None

    print(bag)

    for topic, msg, t in bag.read_messages(topics=['event']):
        events.append(msg.spike)
        time.append((msg.timestamp.to_nsec() / (1e-9)))
        signal.append(float(msg.input))
        threshold.append(float(msg.threshold))
        if coding == None: 
            coding = msg.scheme
        if startpoint == None:
            startpoint = float(msg.start)
        if min_input == None:
            min_input = float(msg.min_input)
        if max_input == None:
            max_input = float(msg.max_input)

    bag.close()
    events = np.array(events)

    if coding == "temporal_contrast":
        reconstructed_signal = 2*DS.temporal_contrast(events, np.mean(threshold))
    elif coding == "step_forward":
        reconstructed_signal = DS.step_forward(events, np.mean(threshold), startpoint)
    elif coding == "moving_window":
        reconstructed_signal = DS.moving_window(events, np.mean(threshold), startpoint)
    # elif coding == "bsa" or coding == "hsa" or coding == "threshold_hsa":
    #     # TODO: include fir, shift
    #     reconstructed_signal = DS.ben_spike(events, fir, shift)
    elif coding == "gaussian_fields": 
        reconstructed_signal = DS.grf_spike(events, min_input, max_input)
    else:
        sys.exit("Encoding scheme not recognized!")

    
    
    plt.subplot(3,1,(1,2))
    plt.plot(signal)
    plt.plot(reconstructed_signal)
    plt.title(coding)
    plt.legend(["Original signal", "Reconstructed signal"])
    plt.subplot(3,1,3)
    if coding == "gaussian_fields":
        for i in range(len(events)):
            plt.plot([i, i], [np.argmax(events[i,:])-0.35, np.argmax(events[i,:])+0.35])
    else: 
        plt.stem(events)
    plt.xlabel("Timestamps")

    plt.show()