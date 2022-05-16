import pybullet
import os

HERE = os.path.dirname(__file__)

ROBOT = os.path.join(HERE, '..', 'robot_info', 'tahoma', 'tahoma.urdf')
POD = os.path.join(HERE, '..', 'robot_info', 'tahoma', 'pod1.urdf')

robot = pybullet.connect(pybullet.GUI)
robot = pybullet.loadURDF(POD)
robot = pybullet.loadURDF(ROBOT, basePosition=[0,-1,0])
# robot = pybullet.loadURDF(ROBOT, basePosition=[0,-1,0])