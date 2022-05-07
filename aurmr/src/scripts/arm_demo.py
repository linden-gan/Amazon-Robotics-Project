#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
               'wrist_2_joint', 'wrist_3_joint']


if __name__ == '__main__':
    try:
        pub = rospy.Publisher("test_arm", PoseStamped, queue_size=10)
        rospy.init_node('arm_demo')
        ps = PoseStamped()
        ps.pose.position.x = -0.4
        ps.pose.position.y = 1.1
        ps.pose.position.z = 0.6
        pub.publish(ps)
    except rospy.ROSInterruptException:
        print("program is interrupted.")
