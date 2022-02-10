#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from ros_exercises.msg import OpenSpace
from cmath import pi
from operator import indexOf


def callback(data):
    dist = max(data.ranges)
    ang = data.angle_min + indexOf(data.ranges, dist)*data.angle_increment
    combined = OpenSpace()
    combined.angle = ang
    combined.distance = dist
    dist_pub = rospy.Publisher('open_space/distance', Float32, queue_size=10)
    ang_pub = rospy.Publisher('open_space/angle', Float32, queue_size=10)
    custom_pub = rospy.Publisher('open_space', OpenSpace, queue_size=10)

    dist_pub.publish(dist)
    ang_pub.publish(ang)
    custom_pub.publish(combined)


def listen():
    rospy.init_node('open_space_publisher', anonymous=True)

    rospy.Subscriber('fake_scan', LaserScan, callback)

    rospy.spin()


if __name__ == '__main__':
    listen()
