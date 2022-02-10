#!/usr/bin/env python

from cmath import pi
from operator import indexOf
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np


def callback(data):
    dist = max(data.ranges)
    ang = data.angle_min + indexOf(data.ranges, dist)*data.angle_increment
    dist_pub = rospy.Publisher('open_space/distance', Float32, queue_size=10)
    ang_pub = rospy.Publisher('open_space/angle', Float32, queue_size=10)

    dist_pub.publish(dist)
    ang_pub.publish(ang)


def listen():
    rospy.init_node('open_space_publisher', anonymous=True)

    rospy.Subscriber('fake_scan', LaserScan, callback)

    rospy.spin()


if __name__ == '__main__':
    listen()
