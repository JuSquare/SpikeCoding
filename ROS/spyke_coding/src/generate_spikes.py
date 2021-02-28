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

import rospy
import rosbag
import numpy as np
import math

from std_msgs.msg import Int32, Float32, String
from spyke_msgs.msg import spyke, spyke_array
import spyke_coding_schemes.EncodingSchemes as encode
import spyke_coding_schemes.DecodingSchemes as decode

def signal_generator(dt, T_max):
    mySin = np.vectorize(math.sin)
    time = np.arange(0,T_max,dt)
    fake_signal_1D = np.zeros(len(time))
    a = [2, -0.5, 0.75]
    f = [1.0, 3.0, 5.0]
    phi = [0.0, 0.0, 0.0]
    sigma = 0.2
    for i in range(len(a)):
        fake_signal_1D += a[i]*mySin(2*math.pi*f[i]*time + phi[i]*np.ones(len(time))) + np.random.rand(len(time))*sigma
    return fake_signal_1D

def spikes_publisher(dt, signal):
    # Create ROS bag
    myBag = rosbag.Bag("encoding.bag", "w")
    minSig = min(signal)
    maxSig = max(signal)
    encoding_scheme = "temporal_contrast"

    # Variables for encoding (buffer, threshold, scaling constants, etc.)
    buffer_size = int(1/dt)
    buffer = np.zeros(buffer_size)
    factor = rospy.get_param('factor')
    threshold = rospy.get_param('threshold')
    window = rospy.get_param('window')
    neurons = rospy.get_param('neurons')
    encoding_scheme = rospy.get_param('encoding_scheme')

    # Setup publisher
    if encoding_scheme == "gaussian_fields":
        pub = rospy.Publisher('event', spyke_array, queue_size=1)
    else:
        pub = rospy.Publisher('event', spyke, queue_size=1)
    
    # Init ros node
    rospy.init_node('spikes_publisher', anonymous=True)
    rate = rospy.Rate(int(1/dt)) 

    # Execute node
    t = 0
    while not rospy.is_shutdown():

        if encoding_scheme == "gaussian_fields":
            # Encode the signal
            mySpikes = encode.grf_spike(signal[t], neurons, minSig, maxSig)
            # Store data
            spyke_event = spyke_array()
            spyke_event.spike = np.int8(mySpikes.tolist())
            spyke_event.start = signal[buffer_size-1]
            spyke_event.min_input = minSig
            spyke_event.max_input = maxSig
            spyke_event.threshold = threshold
            spyke_event.input = signal[t]
            spyke_event.scheme = encoding_scheme
            spyke_event.timestamp = rospy.get_rostime()
            myBag.write('event', spyke_event)
            # Publish event
            pub.publish(spyke_event)

        else:
            if t < buffer_size:
                buffer[t] = signal[t]
            else:
                # Update the buffer
                buffer = np.roll(buffer, -1)
                buffer[-1] = signal[t]
                # Encode the signal
                if encoding_scheme == "temporal_contrast":
                    mySpikes, threshold = encode.temporal_contrast(buffer, factor) 
                elif encoding_scheme == "step_forward":
                    mySpikes, _ = encode.step_forward(buffer, threshold)
                elif encoding_scheme == "moving_window":
                    mySpikes, _ = encode.moving_window(buffer, threshold, window)
                else:
                    rospy.signal_shutdown("No encoding scheme found!")
                # Store data
                spyke_event = spyke()
                spyke_event.spike = np.int8(mySpikes[-1])
                spyke_event.start = signal[buffer_size-1]
                spyke_event.min_input = minSig
                spyke_event.max_input = maxSig
                spyke_event.threshold = threshold
                spyke_event.input = signal[t]
                spyke_event.scheme = encoding_scheme
                spyke_event.timestamp = rospy.get_rostime()
                myBag.write('event', spyke_event)
                # Publish event
                pub.publish(spyke_event)

        rate.sleep()
        t += 1

        # Check signal validity
        if t == len(signal):
            rospy.signal_shutdown("End of signal")
    
    # Close ROS bag after execution
    myBag.close()


if __name__ == '__main__':
    # Setup fake signal for testing (to be replaced with subscriber to sensor node)
    dt = rospy.get_param('dt')
    T_max = rospy.get_param('T_max')
    signal = signal_generator(dt, T_max)
    
    # Launch node
    try:
        spikes_publisher(dt, signal)
    except rospy.ROSInterruptException:
        pass
