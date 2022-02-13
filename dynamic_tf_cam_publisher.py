#!/usr/bin/env python

from turtle import right
import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf
import tf2_msgs.msg
import numpy as np


# class CameraBroadcaster:

#     def __init__(self):
#         self.left_pub_tf = rospy.Publisher("/left_cam", tf2_msgs.msg.TFMessage, queue_size=1)
#         self.right_pub_tf = rospy.Publisher("/right_cam", tf2_msgs.msg.TFMessage, queue_size=1)

#         while not rospy.is_shutdown():
#             # Run this loop at about 20Hz
#             rospy.sleep(0.05)

#             t = geometry_msgs.msg.TransformStamped()
#             t.header.frame_id = "base_link_gt"
#             t.header.stamp = rospy.Time.now()
#             t.child_frame_id = "left_cam"
#             t.transform.translation.x = 0.05
#             t.transform.translation.y = 0.0
#             t.transform.translation.z = 0.0

#             t.transform.rotation.x = 0.0
#             t.transform.rotation.y = 0.0
#             t.transform.rotation.z = 0.0
#             t.transform.rotation.w = 1.0

#             tfm = tf2_msgs.msg.TFMessage([t])
#             self.pub_tf.publish(tfm)

def broadcaster():
    rospy.init_node('dynamic_tf_cam_publisher')
    rate = rospy.Rate(20)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()

    # left_pub_tf = rospy.Publisher("/left_cam", tf2_msgs.msg.TFMessage, queue_size=1)
    # right_pub_tf = rospy.Publisher("/right_cam", tf2_msgs.msg.TFMessage, queue_size=1)

    while not rospy.is_shutdown():
        # Run this loop at about 20Hz

        try:
            base_trans = tfBuffer.lookup_transform("world", "base_link_gt", rospy.Time())
        except tf.Exception:
            rate.sleep()
            continue

        base_trans_matrix = tf.transformations.quaternion_matrix([base_trans.transform.rotation.x,base_trans.transform.rotation.y,base_trans.transform.rotation.z,base_trans.transform.rotation.w])

        base_trans_matrix[0][3] = base_trans.transform.translation.x
        base_trans_matrix[1][3] = base_trans.transform.translation.y
        base_trans_matrix[2][3] = base_trans.transform.translation.z



        left_t = TransformStamped()
        left_t.header.frame_id = "base_link_gt"
        left_t.header.stamp = rospy.Time.now()
        left_t.child_frame_id = "left_cam"
        left_t.transform.translation.x = -0.05
        left_t.transform.rotation.w = 1.0

        left_trans_matrix = tf.transformations.quaternion_matrix([left_t.transform.rotation.x,left_t.transform.rotation.y,left_t.transform.rotation.z, left_t.transform.rotation.w])
        left_trans_matrix[0][3] = left_t.transform.translation.x

        left_wrt_world = np.dot(base_trans_matrix,left_trans_matrix)

        right_t = TransformStamped()
        right_t.header.frame_id = "base_link_gt"
        right_t.header.stamp = rospy.Time.now()
        right_t.child_frame_id = "right_cam"
        right_t.transform.translation.x = 0.05
        right_t.transform.rotation.w = 1.0

        right_trans_matrix = tf.transformations.quaternion_matrix([right_t.transform.rotation.x, right_t.transform.rotation.y, right_t.transform.rotation.z, right_t.transform.rotation.w])
        right_trans_matrix[0][3] = right_t.transform.translation.x

        right_wrt_left = np.dot(np.linalg.inv(left_trans_matrix), right_trans_matrix)

        left_final_rotation = tf.transformations.quaternion_from_matrix(left_wrt_world[0:4][0:4])
        right_final_rotation = tf.transformations.quaternion_from_matrix(right_wrt_left[0:4][0:4])

        left_final = TransformStamped()
        left_final.header.frame_id = "world"
        left_final.child_frame_id = "left_cam"
        left_final.header.stamp = rospy.Time.now()
        left_final.transform.translation.x = left_wrt_world[0][3]
        left_final.transform.translation.y = left_wrt_world[1][3]
        left_final.transform.translation.z = left_wrt_world[2][3]
        left_final.transform.rotation.x = left_final_rotation[0]
        left_final.transform.rotation.y = left_final_rotation[1]
        left_final.transform.rotation.z = left_final_rotation[2]
        left_final.transform.rotation.w = left_final_rotation[3]

        right_final = TransformStamped()
        right_final.header.frame_id = "left_cam"
        right_final.child_frame_id = "right_cam"
        right_final.header.stamp = rospy.Time.now()
        right_final.transform.translation.x = right_wrt_left[0][3]
        right_final.transform.translation.y = right_wrt_left[1][3]
        right_final.transform.translation.z = right_wrt_left[2][3]
        right_final.transform.rotation.x = right_final_rotation[0]
        right_final.transform.rotation.y = right_final_rotation[1]
        right_final.transform.rotation.z = right_final_rotation[2]
        right_final.transform.rotation.w = right_final_rotation[3]
        

        br.sendTransform(left_final)
        br.sendTransform(right_final)

        # tfml = tf2_msgs.msg.TFMessage([left_t])
        # tfmr = tf2_msgs.msg.TFMessage([right_t])
        # left_pub_tf.publish(tfml)
        # right_pub_tf.publish(tfmr)

        rate.sleep()

if __name__ == '__main__':
    broadcaster()
