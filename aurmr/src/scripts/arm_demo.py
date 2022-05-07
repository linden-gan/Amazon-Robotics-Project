#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
               'wrist_2_joint', 'wrist_3_joint']

def arm_client():
    client = actionlib.SimpleActionClient('move', FollowJointTrajectoryAction)
    print("aaaaaaaaaa")
    client.wait_for_server()
    print("bbbbbbbb")
    # make goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = JOINT_NAMES
    goal.trajectory.points = toJointTrajectoryPoint([(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (-1.7667559222150564, -0.33867824650507217, 0.9539507574573997, 0.963937568648913, 1.5730515006204613, -1.7648161984839672)])
    client.send_goal(goal)

    print("cccccccccc")
    client.wait_for_result()
    print("dddddddddd")
    return client.get_result()


def toJointTrajectoryPoint(waypoints):
    """
    transform a list of waypoints to a list of JointTrajectoryPoint
    """
    list = []
    for point in waypoints:
        list.append(JointTrajectoryPoint(positions=point))

    return list


if __name__ == '__main__':
    try:
        rospy.init_node('arm_demo')
        result = arm_client()
        print(f'result is {result}')
    except rospy.ROSInterruptException:
        print("program is interrupted.")
