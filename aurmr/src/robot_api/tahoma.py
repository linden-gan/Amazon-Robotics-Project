#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

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

SELF_COLLISION_DISABLED_LINKS = [  # change /////////////////////////////////////////////////////////////////////
        ('stand', 'base_link_inertia'), ('stand', 'shoulder_link'), ('stand', 'upper_arm_link'),
        ('base_link_inertia', 'shoulder_link'), ('shoulder_link', 'upper_arm_link'),
        ('upper_arm_link', 'forearm_link'), ('forearm_link', 'wrist_1_link'), ('wrist_1_link', 'wrist_2_link'),
        ('wrist_2_link', 'wrist_3_link')]

# joint server on actual robot
JOINT_ACTION_SERVER = '/pos_joint_traj_controller/follow_joint_trajectory'

# topic about robot's current actual state
ACTUAL_STATE_TOPIC = "sensor_msgs/JointState"

THRESHOLD = 0.1


class Tahoma:
    def __init__(self, pb_robot, initial_joint_state=PYBULLET_JOINT_INITIAL) -> None:
        # initialize arm info
        self._pb_robot = pb_robot
        self._joint_indices = pp.joints_from_names(pb_robot, JOINT_NAMES)
        self._disabled_links = pp.get_disabled_collisions(pb_robot, SELF_COLLISION_DISABLED_LINKS)

        # synchronize pybullet simulator's initial pose to actual robot if actual state is not
        # equal to simulator's state
        if initial_joint_state != PYBULLET_JOINT_INITIAL:  # Threshold!
            path = plan_motion_from_to(pb_robot, self._joint_indices, PYBULLET_JOINT_INITIAL, initial_joint_state,
                                       obstacles=[], self_collisions=True,
                                       disabled_collisions=self._disabled_links,
                                       algorithm='rrt')
            if path is None:
                cprint('Bad initial joint state', 'red')
                return
            sim_execute_motion(self._pb_robot, self._joint_indices, path)
        # initialize joint state in simulator
        self._sim_joint_state = initial_joint_state
        # initialize joint state of actual robot
        self._actual_joint_state = None

        # listen to robot's joint state
        self._robot_state_sub = rospy.Subscriber(ACTUAL_STATE_TOPIC, JointState, callback=self.get_actual_state)

        # initialize action client to move robot's arm
        self._trajectory_client = actionlib.SimpleActionClient(JOINT_ACTION_SERVER, FollowJointTrajectoryAction)

    def get_actual_state(self, msg: JointState):
        self._actual_joint_state = msg.position

    def synchronize(self):
        # synchronize pybullet simulator's initial pose to actual robot if actual state is not
        # equal to simulator's state
        accurate = True
        for i in range(len(self._actual_joint_state)):
            if abs(self._actual_joint_state[i] - self._sim_joint_state[i]) > THRESHOLD:
                accurate = False
                break

        if not accurate:
            path = plan_motion_from_to(robot, self._joint_indices, self._sim_joint_state, self._actual_joint_state,
                                       obstacles=[], self_collisions=True,
                                       disabled_collisions=self._disabled_links,
                                       algorithm='rrt')
            if path is None:
                cprint('Fail to synchronize', 'red')
                return

            sim_execute_motion(self._pb_robot, self._joint_indices, path)
            # update initialized joint state
            self._sim_joint_state = path[-1]

    def move_to_pose_goal(self,
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
        print(f'start conf is {self._sim_joint_state}')
        print(f'end conf is {end_joint_state}')
        path = plan_motion_from_to(robot, self._joint_indices, self._sim_joint_state, end_joint_state,
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
        print(result)
        if result is not None:
            cprint(f'error code is {result.error_code}', 'cyan')
            if result.error_code == 0:
                # update current joint state
                self._sim_joint_state = end_joint_state
                self.synchronize()
                return True

        cprint('Failed to move actual robot', 'red')
        return False


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
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    robot = p.loadURDF(ROBOT_URDF)

    # initialize arm robot
    arm = Tahoma(robot, [-0.8574946697157134, 0.3600245844803187, -1.3603905625549857, -0.5915446627350632, 1.5741277934974172, 0.856103511284937])

    # (-0.8574946697157134, 0.3600245844803187, -1.3603905625549857, -0.5915446627350632, 1.5741277934974172, 0.856103511284937)

    rospy.spin()
