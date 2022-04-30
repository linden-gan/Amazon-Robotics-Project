#!/usr/bin/env python

import rospy
import actionlib
# from aurmr.srv import add
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint

import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc
import pybullet_planning as pp

import os

import aurmr.msg
from move import move_to, plan_motion
from voxel_demo import VoxelManager

HERE = os.path.dirname(__file__)

ROBOT_URDF = os.path.join(HERE, '..', 'robot_info', 'robot_with_stand.urdf')

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
               'wrist_2_joint', 'wrist_3_joint']

SELF_COLLISION_DISABLED_LINKS = [
        ('stand', 'base_link_inertia'), ('stand', 'shoulder_link'), ('stand', 'upper_arm_link'),
        ('base_link_inertia', 'shoulder_link'), ('shoulder_link', 'upper_arm_link'),
        ('upper_arm_link', 'forearm_link'), ('forearm_link', 'wrist_1_link'), ('wrist_1_link', 'wrist_2_link'),
        ('wrist_2_link', 'wrist_3_link')]


class Arm:
    # _feedback

    def __init__(self, name, robot) -> None:
        # initialize arm info
        self._robot = robot
        self._joints = pp.joints_from_names(robot, JOINT_NAMES)
        self._disabled_links = pp.get_disabled_collisions(robot, SELF_COLLISION_DISABLED_LINKS)

        # initialize action server
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction,
                       execute_cb=self.call_back, auto_start=False)
        self._server.start()
        
    def move_to_goal(self, goal):
        # compute joint destination according to end effector position
        des_config = p.calculateInverseKinematics(self._robot, self._joints[-1], goal.pose, goal.orien)

        # plan motion
        path = plan_motion(self._robot, self._joints, des_config,
                                obstacles=[], self_collisions=True,
                                disabled_collisions=self._disabled_links,
                                algorithm='rrt')

        if path is None:
            return

        # execute path
        time_step = 0.03
        for conf in path:
            pp.set_joint_positions(self._robot, self._joints, conf)

            pp.wait_for_duration(time_step)


    # def move_server():
    #     rospy.init_node('move')
    #     s = rospy.Service('add_move', add, handle_move)
    #     print("Ready to add two ints.")
    #     rospy.spin()


# class Arm:
#     def __init__(self, name, robot) -> None:
#         # initialize arm info
#         self._joints = pp.joints_from_names(robot, JOINT_NAMES)
#         self._disabled_links = pp.get_disabled_collisions(robot, SELF_COLLISION_DISABLED_LINKS)

#         # initialize action server
#         self._client = actionlib.SimpleActionClient("/arm_contorller/follow_joint_trajectory", FollowJointTrajectoryAction)
#         self._client.wait_for_server()
        
#     def move(self, pose, orien):
#         goal = FollowJointTrajectoryActionGoal()
#         move_to(robot, self._joints, pose, orien, [], self._disabled_links)


if __name__ == "__main__":
    # initialize the env
    pp.connect(use_gui=True)
    robot = pp.load_pybullet(ROBOT_URDF, fixed_base=True)
    # gravity
    p.setGravity(0, 0, -9.8)

    # add obstacles
    # TODO: change this for better modularity
    # fill voxels
    manager = VoxelManager()
    positions = [(0,2,1), (0,1,1), (0,1,1.5), (0,1,0.5), (0, 1, 0.8), (0,1,2)]
    manager.fill_voxels(positions)

    # initialize node
    rospy.init_node('move')
    server = Arm(rospy.get_name(), robot)
    rospy.spin()
