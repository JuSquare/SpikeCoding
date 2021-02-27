#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from spyke_coding_schemes.EncodingSchemes import *
from spyke_coding_schemes.DecodingSchemes import *


def spikes_publisher():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('spikes_publisher', anonymous=True)
    rate = rospy.Rate(5) # 5 Hz
    while not rospy.is_shutdown():
        hello_str = "hello world !"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        spikes_publisher()
    except rospy.ROSInterruptException:
        pass
