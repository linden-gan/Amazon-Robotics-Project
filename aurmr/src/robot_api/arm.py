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

from move import plan_motion_from_to, plan_motion
# from aurmr.src.robot_api.voxel_manager import VoxelManager

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
    def __init__(self, name, robot) -> None:
        # initialize arm info
        self._robot = robot
        self._joints = pp.joints_from_names(robot, JOINT_NAMES)
        self._joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._disabled_links = pp.get_disabled_collisions(robot, SELF_COLLISION_DISABLED_LINKS)

        # initialize action client
        self._trajectory_client = actionlib.SimpleActionClient('robot', FollowJointTrajectoryAction)
        
    def move_to_goal(self, goal: FollowJointTrajectoryGoal):
        """
        DEPRECATED
        """
        # feedback = FollowJointTrajectoryFeedback()
        # result = FollowJointTrajectoryResult()
        #
        # waypoints = []
        # # traverse through all poses of waypoints
        # for point in goal.trajectory.points:
        #     waypoints.append(point.positions)
        #
        # # plan motion
        # detail_path = [waypoints[0]]
        # for i in range(len(waypoints) - 1):
        #     detail_path.extend(plan_motion_from_to(self._robot, self._joints, waypoints[i], waypoints[i + 1],
        #                                            obstacles=[], self_collisions=True, disabled_collisions=self._disabled_links))
        #
        # if detail_path is None:
        #     return
        #
        # # execute path
        # time_step = 0.03
        # print(f'the length of detail path is {len(detail_path)}')
        # for conf in detail_path:
        #     pp.set_joint_positions(self._robot, self._joints, conf)
        #     pp.wait_for_duration(time_step)
        #
        # # make goal
        # goal = FollowJointTrajectoryGoal()
        # goal.trajectory.joint_names = JOINT_NAMES
        # goal.trajectory.points = []

    def move_to_pose(self, end_pose: PoseStamped):
        # plan motion
        path = self.compute_path_to(end_pose.pose.position, end_pose.pose.orientation)

        if path is None:
            return

        # execute path
        time_step = 0.03
        print(f'the length of detail path is {len(path)}')
        for conf in path:
            pp.set_joint_positions(self._robot, self._joints, conf)
            pp.wait_for_duration(time_step)

        # make goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points = toJointTrajectoryPoint(path)
        # send goal
        self._trajectory_client.send_goal(goal)
        self._trajectory_client.wait_for_result()
        # return result
        return self._trajectory_client.get_result()

    def compute_path_to(self, end_position, end_orien):
        # compute joint destination according to end effector position
        end_joint_state = p.calculateInverseKinematics(self._robot, self._joints[-1],
                                              end_position, end_orien)
        # plan motion
        path = plan_motion(robot, self._joints, end_joint_state,
                           obstacles=[], self_collisions=True,
                           disabled_collisions=self._disabled_links,
                           algorithm='rrt')
        return path


def toJointTrajectoryPoint(waypoints):
    """
    transform a list of waypoints to a list of JointTrajectoryPoint
    """
    res = []
    for point in waypoints:
        res.append(JointTrajectoryPoint(positions=point))

    return res


if __name__ == "__main__":
    # initialize the env
    pp.connect(use_gui=True)
    robot = pp.load_pybullet(ROBOT_URDF, fixed_base=True)
    # gravity
    p.setGravity(0, 0, -9.8)

    # initialize node
    arm = Arm()
    arm.move_to_pose()
