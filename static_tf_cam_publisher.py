#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf
import tf2_msgs.msg

def static_broadcaster():
    rospy.init_node("static_tf_cam_publisher")

    br = tf2_ros.StaticTransformBroadcaster()

    left_t = TransformStamped()
    left_t.header.frame_id = "base_link_gt"
    left_t.header.stamp = rospy.Time.now()
    left_t.child_frame_id = "left_cam"
    left_t.transform.translation.x = -0.05
    left_t.transform.rotation.w = 1.0

    right_t = TransformStamped()
    right_t.header.frame_id = "base_link_gt"
    right_t.header.stamp = rospy.Time.now()
    right_t.child_frame_id = "right_cam"
    right_t.transform.translation.x = 0.05
    right_t.transform.rotation.w = 1.0

    
    br.sendTransform([left_t,right_t])
    rospy.spin()

if __name__ == '__main__':
    static_broadcaster()



