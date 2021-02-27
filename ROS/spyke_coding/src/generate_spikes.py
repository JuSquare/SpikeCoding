#! /usr/bin/env python
import rospy
import rosbag
import numpy as np
import math

from std_msgs.msg import Int32, Float32, String
from spyke_msgs.msg import spyke
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
    # Setup publisher
    pub = rospy.Publisher('event', spyke, queue_size=1)
    # Create ROS bag
    myBag = rosbag.Bag("encoding.bag", "w")
    minSig = min(signal)
    maxSig = max(signal)
    encoding_scheme = "temporal_contrast"
    # Variables for encoding (buffer, threshold, scaling constants, etc.)
    buffer_size = int(1/dt)
    buffer = np.zeros(buffer_size)
    threshold = 0.0
    tbr_factor = 1.005
    # Init ros node
    rospy.init_node('spikes_publisher', anonymous=True)
    rate = rospy.Rate(int(1/dt)) 
    # Execute node
    t = 0
    while not rospy.is_shutdown():

        # Update buffer
        if t < buffer_size:
            buffer[t] = signal[t]
        else:
            buffer = np.roll(buffer, -1)
            buffer[-1] = signal[t]
            mySpikes, threshold = encode.temporal_contrast(buffer, tbr_factor) 
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
    [dt, T_max] = [0.01, 5]
    signal = signal_generator(dt, T_max)
    # Launch node
    try:
        spikes_publisher(dt, signal)
    except rospy.ROSInterruptException:
        pass
