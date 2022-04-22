import pybullet
import pybullet_data
import pybullet_utils.bullet_client as bc
import sys
import time
import os
from termcolor import cprint
import pybullet_planning as pp

sys.path.insert(4, '/home/zoeyc/github/AUMR_gym')

# initilize the env
p = bc.BulletClient(connection_mode=pybullet.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setRealTimeSimulation(0)
_time_step = 1 / 12  # change to the correct control frequency
p.setPhysicsEngineParameter(fixedTimeStep=_time_step, numSubSteps=20, deterministicOverlappingPairs=1)
p.setGravity(0, 0, -9.8)
p.resetSimulation()

dir_path = os.path.dirname(os.path.realpath(__file__))
urdf_path = dir_path + "/robot_info/robot_with_stand.urdf"
# urdf_path = dir_path + "/robot_info/robot.urdf"
# load robot
robot = p.loadURDF(urdf_path)
while True:
    p.stepSimulation()
    time.sleep(0.1)

