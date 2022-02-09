#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np

def simple_publisher():
    pub = rospy.Publisher('my_random_float', Float32, queue_size=10)
    rospy.init_node('simple_publisher', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
        random_num = np.random.rand()*10
        pub.publish(random_num)
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_publisher()
    except rospy.ROSInterruptException:
        pass
