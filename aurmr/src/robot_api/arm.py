#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint

import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc
import pybullet_planning as pp

import os
from termcolor import cprint

from move import plan_motion_from_to, sim_execute_motion

HERE = os.path.dirname(__file__)

ROBOT_URDF = os.path.join(HERE, '..', 'robot_info', 'robot_with_stand.urdf')

PYBULLET_JOINT_INITIAL = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
               'wrist_2_joint', 'wrist_3_joint']

SELF_COLLISION_DISABLED_LINKS = [
        ('stand', 'base_link_inertia'), ('stand', 'shoulder_link'), ('stand', 'upper_arm_link'),
        ('base_link_inertia', 'shoulder_link'), ('shoulder_link', 'upper_arm_link'),
        ('upper_arm_link', 'forearm_link'), ('forearm_link', 'wrist_1_link'), ('wrist_1_link', 'wrist_2_link'),
        ('wrist_2_link', 'wrist_3_link')]

JOINT_ACTION_SERVER = '/pos_joint_traj_controller/follow_joint_trajectory'

TOPIC = "test_arm"


class Arm:
    def __init__(self, pb_robot, initial_joint_state=PYBULLET_JOINT_INITIAL) -> None:
        # initialize arm info
        self._pb_robot = pb_robot
        self._joint_indices = pp.joints_from_names(robot, JOINT_NAMES)
        self._disabled_links = pp.get_disabled_collisions(robot, SELF_COLLISION_DISABLED_LINKS)

        # synchronize pybullet simulator's initial pose to actual robot if actual state is not
        # equal to simulator's state
        if initial_joint_state != PYBULLET_JOINT_INITIAL:
            path = plan_motion_from_to(robot, self._joint_indices, PYBULLET_JOINT_INITIAL, initial_joint_state,
                                       obstacles=[], self_collisions=True,
                                       disabled_collisions=self._disabled_links,
                                       algorithm='rrt')
            if path is None:
                cprint('Bad initial joint state', 'red')
                return
            sim_execute_motion(self._pb_robot, self._joint_indices, path)
        # update initialized joint state
        self._joint_state = initial_joint_state

        # initialize action client
        self._trajectory_client = actionlib.SimpleActionClient(JOINT_ACTION_SERVER, FollowJointTrajectoryAction)
        # initialize end pose subscriber
        self._pose_sub = rospy.Subscriber(TOPIC, PoseStamped, callback=self.callback)

    def callback(self, goal: PoseStamped):
        self.move_to_pose(goal)

    def move_to_pose(self,
                     end_pose: PoseStamped,
                     max_plan_time=10.0,
                     execution_timeout=15,
                     tolerance=0.01,
                     orientation_constraint=None):
        # use IK to compute end joint state according to end effector position
        position = [end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z]
        orien = [end_pose.pose.orientation.x, end_pose.pose.orientation.y,
                 end_pose.pose.orientation.z, end_pose.pose.orientation.w]
        end_joint_state = p.calculateInverseKinematics(self._pb_robot, self._joint_indices[-1],
                                                       position, orien)

        # plan motion
        print(f'start conf is {self._joint_state}')
        print(f'end conf is {end_joint_state}')
        path = plan_motion_from_to(robot, self._joint_indices, self._joint_state, end_joint_state,
                                   obstacles=[], self_collisions=True,
                                   disabled_collisions=self._disabled_links,
                                   algorithm='rrt')

        if path is None:
            cprint('No path found', 'red')
            return

        # execute path in pybullet
        sim_execute_motion(self._pb_robot, self._joint_indices, path)

        # make goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points = to_joint_trajectory_point(path)
        # send goal
        self._trajectory_client.send_goal(goal)
        self._trajectory_client.wait_for_result(rospy.Duration(execution_timeout))
        # return result
        result = self._trajectory_client.get_result()

        # update current joint state
        self._joint_state = end_joint_state


def to_joint_trajectory_point(waypoints):
    """
    transform a list of waypoints of joint states to a list of JointTrajectoryPoint
    """
    res = []
    for point in waypoints:
        res.append(JointTrajectoryPoint(positions=point))

    return res


if __name__ == "__main__":
    rospy.init_node('arm')

    # initialize the env
    pp.connect(use_gui=True)
    robot = pp.load_pybullet(ROBOT_URDF, fixed_base=True)
    # gravity
    p.setGravity(0, 0, -9.8)

    # initialize arm robot
    arm = Arm(robot)

    rospy.spin()
