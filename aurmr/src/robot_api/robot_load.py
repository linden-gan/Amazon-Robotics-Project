#!/usr/bin/env python

import pybullet
import os
import pybullet as p
import pybullet_planning as pp

HERE = os.path.dirname(__file__)

ROBOT = os.path.join(HERE, '..', 'robot_info', 'tahoma', 'tahoma.urdf')
POD = os.path.join(HERE, '..', 'robot_info', 'tahoma', 'pod1.urdf')

JOINT_NAMES = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_joint', 'arm_wrist_1_joint',
               'arm_wrist_2_joint', 'arm_wrist_3_joint']

# initialize robot in pybullet
p.connect(p.GUI)
pod = p.loadURDF(POD)
robot = p.loadURDF(ROBOT, basePosition=[0, -1, 0])

indices = pp.joints_from_names(robot, JOINT_NAMES)

print(p.getJointStates(robot, indices))
sleep(10000)
