#!/usr/bin/env python

from cmath import pi
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


def laser_publisher():
    rospy.init_node('fake_scan_publisher', anonymous=True)
    pub = rospy.Publisher('fake_scan', LaserScan, queue_size=10)
    rate = rospy.Rate(20)  # 20hz
    ranges = []
    for i in range(int(4.0/3.0*pi / (pi/300.0))+1):
        if i == 0:
            ranges.append(1.0)
        elif i == 1:
            ranges.append(10.0)
        else:
            ranges.append(np.random.random()*10)
    fake_scan = LaserScan()
    fake_scan.header.stamp = rospy.Time.now()
    fake_scan.header.frame_id = 'base_link'
    fake_scan.angle_min = -2.0/3.0*pi
    fake_scan.angle_max = 2.0/3.0*pi
    fake_scan.angle_increment = 1.0/300.0*pi
    fake_scan.scan_time = 1.0/20.0
    fake_scan.range_min = 1.0
    fake_scan.range_max = 10.0
    fake_scan.ranges = ranges

    while not rospy.is_shutdown():
        pub.publish(fake_scan)
        rate.sleep()


if __name__ == '__main__':
    try:
        laser_publisher()
    except rospy.ROSInterruptException:
        pass
