#!/usr/bin/env python

from turtle import right
import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf
import tf2_msgs.msg
import numpy as np

def base_pub():
    rospy.init_node("base_link_tf_pub")

    rate = rospy.Rate(20)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()

    base_world_tf = TransformStamped()
    base_world_tf.header.frame_id = "world"
    base_world_tf.child_frame_id = "base_link_gt_2"


    while not rospy.is_shutdown():

        try:
            left_wrt_world = tfBuffer.lookup_transform("world", "left_cam", rospy.Time())
        except tf.Exception:
            rate.sleep()
            continue

        left_t = TransformStamped()
        left_t.header.frame_id = "base_link_gt"
        left_t.header.stamp = rospy.Time.now()
        left_t.child_frame_id = "left_cam"
        left_t.transform.translation.x = -0.05
        left_t.transform.rotation.w = 1.0

        # this is the matrix for the left wrt the world
        left_world_mat =  tf.transformations.quaternion_matrix([left_wrt_world.transform.rotation.x,left_wrt_world.transform.rotation.y, left_wrt_world.transform.rotation.z,left_wrt_world.transform.rotation.w])
        left_world_mat[0][3] = left_wrt_world.transform.translation.x
        left_world_mat[1][3] = left_wrt_world.transform.translation.y
        left_world_mat[2][3] = left_wrt_world.transform.translation.z

        # this is the matrix for the left wrt the base
        left_base_mat = tf.transformations.quaternion_matrix([left_t.transform.rotation.x,left_t.transform.rotation.y, left_t.transform.rotation.z,left_t.transform.rotation.w])
        left_base_mat[0][3] = left_t.transform.translation.x

        #computing the base wrt the world

        base_world_mat = np.dot(left_world_mat,np.linalg.inv(left_base_mat))

        quat = tf.transformations.quaternion_from_matrix(base_world_mat[0:4][0:4])

        base_world_tf.header.stamp = rospy.Time.now()
        base_world_tf.transform.translation.x = base_world_mat[0][3]
        base_world_tf.transform.translation.y = base_world_mat[1][3]
        base_world_tf.transform.translation.z = base_world_mat[2][3]
        base_world_tf.transform.rotation.x = quat[0]
        base_world_tf.transform.rotation.y = quat[1]
        base_world_tf.transform.rotation.z = quat[2]
        base_world_tf.transform.rotation.w = quat[3]

        br.sendTransform(base_world_tf)

        rate.sleep()

if __name__ == '__main__':
    base_pub()
