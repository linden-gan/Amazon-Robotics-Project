#!/usr/bin/env python

import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc
import pybullet_planning as pp

import os
from termcolor import cprint
from lib import sim_execute_motion, plan_motion_from_to

HERE = os.path.dirname(__file__)

ROBOT = os.path.join(HERE, 'tahoma_info', 'tahoma.urdf')
POD = os.path.join(HERE, 'tahoma_info', 'pod1.urdf')

JOINT_NAMES = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint', 'arm_wrist_1_joint',
               'arm_wrist_2_joint', 'arm_wrist_3_joint', 'gripper_finger_joint']
# JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
#                'wrist_2_joint', 'wrist_3_joint']
GRIPPER_LINK = "gripper_robotiq_arg2f_base_link"

# initialize robot in pybullet
p.connect(p.GUI)
# target = pp.create_box(0.1, 0.1, 0.1)
# x = 0.2
# y = -0.2
# z = 1.7
# # x = -0.5
# # y = 1.0
# # z = 1.0
# pp.set_pose(target, pp.Pose(pp.Point(x=x, y=y, z=z)))
pod = p.loadURDF(POD)
robot = p.loadURDF(ROBOT, basePosition=[0,-1,0])
# robot = p.loadURDF(ROBOT)

joint_indices = pp.joints_from_names(robot, JOINT_NAMES)
print(f'joint_indices are {joint_indices}')
gripper = pp.link_from_name(robot, GRIPPER_LINK)
print(f'gripper link is {gripper}')

# # attach end effector
# gripper_body = pp.create_obj("")
# gripper_link = pp.link_from_name(robot, "")
# gripper_link_pose = pp.get_link_pose(robot, gripper)
# pp.set_pose(gripper_body, gripper_link_pose)
# gripper_attach = pp.create_attachment(robot, gripper_link, gripper_body)
# gripper_attach.assign()

pp.set_joint_positions(robot, joint_indices, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
pp.wait_for_user()

pp.set_joint_positions(robot, joint_indices, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
pp.wait_for_user()

pp.set_joint_positions(robot, joint_indices, [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
pp.wait_for_user()

pp.set_joint_positions(robot, joint_indices, [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
pp.wait_for_user()

pp.set_joint_positions(robot, joint_indices, [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
pp.wait_for_user()

pp.set_joint_positions(robot, joint_indices, [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
pp.wait_for_user()

start_conf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

print('j_i', joint_indices)
print('pose1', [x, y, z])
end_conf = p.calculateInverseKinematics(robot, gripper, [x, y, z], [0, 0, 0, 1])

print(f'start conf is {start_conf}')
print(f'end conf is {end_conf}')
path = plan_motion_from_to(robot, joint_indices, start_conf, end_conf,
                                   obstacles=[], self_collisions=False,
                                   disabled_collisions=[],
                                   algorithm='rrt')

if path is None:
    cprint('No path found', 'red')
    exit(0)

# execute path in pybullet
sim_execute_motion(robot, joint_indices, path)

