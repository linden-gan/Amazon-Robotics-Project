# Amazon-Robotics-Project

This is a pybullet interface for motion planning to move robot to a desired end pose.

## Download
```
git clone https://github.com/linden-gan/Amazon-Robotics-Project.git
```
This repo must be cloned under the **common directory** of the aurmr_tahoma repo: ```aurmr_tahoma/aurmr_task/src/common```

## pb_tahoma vs. real_tahoma


## Usage
### Prereq for Usage
- ROS
- pybullet
- pybullet_planning

### Things Users Can Modify: 
- ```ROBOT```: File path of robot urdf
- ```POD```: File path of pod urdf
- ```GRIPPER```: End effector name
- ```SELF_COLLISION_DISABLED_LINKS```: A list of pairs of links that we need disable for self collision checking
- ```JOINT_ACTION_SERVER```: Joint motion action server on actual robot
- ```ACTUAL_STATE_TOPIC```: Topic about robot's current actual state
- ```REAL_TIME_STAMP```: Time stamp (sec) for actual robot to move (smaller time stamp makes robot faster)

### Main Function to Use: 
```
move_to_pose_goal(self, end_pose: PoseStamped, max_plan_time=10.0, execution_timeout=15, tolerance=0.01, orientation_constraint=None)
```
This function takes in an end-effector pose and moves the end-effector of both simulation robot and actual robot to the desired pose within some tolerance. It will first synchronize the similutation robot with the actual robot and sample paths in simulation environment and send goal to the actual robot.

#### Parameters
- ```end_pose:PoseStamped```
- ```max_plan_time:float, default=10.0``` 
- ```execution_timeout:float, default=15```
- ```tolerance:float, defualt=0.01```
- ```orientation_constraint:float, default=None```

##### Return
- ```True``` if the actual robot is sucessfully moved to the desired end-effector pose.
- ```False``` if no valid path is found by pybullet planning, or the actual robot fails to execute the provided trajectory.

## Usage Example: Integrate this interface with StateMachine
```
import smach_ros
from smach import State, StateMachine

import rospy

from aurmr_tasks.common import states
from aurmr_tasks.common.pb_tahoma import Tahoma
import aurmr_tasks.common.control_flow as cf
from aurmr_tasks.common import motion

import std_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf_conversions import transformations

# This is a list of desired waypoints with type PoseStamped that users need to create and modify
WAYPOINTS

# 需要init node吗？？？？？？？？？？？？？？？？？？？？？
robot = Tahoma()

simulation = rospy.get_param("~simulation", False)
State.simulation = simulation
pick_sm = StateMachine(["succeeded", "preempted", "aborted"], input_keys=[], output_keys=[])

with pick_sm:
        cf.inject_userdata_auto("LOAD_POSES", "poses", WAYPOINTS)
        StateMachine.add("PICK_POSE", cf.IterateList("poses", "pose"),
                         {"repeat": "MOVE_TO_POSE", "done": "aborted"})
        StateMachine.add_auto('MOVE_TO_POSE',
                              motion.MoveEndEffectorToPose(robot),
                              ['succeeded', 'preempted', 'aborted'],
                              {'aborted': 'MOVE_TO_POSE'})
        StateMachine.add("PAUSE", states.Wait(3), {'succeeded': "PICK_POSE"})
        
# Create top state machine
sm = StateMachine(outcomes=['succeeded', "preempted", 'aborted'])


with sm:
    StateMachine.add('PLAY_POSE_SEQUENCE', pick_sm, {"succeeded": "succeeded", "aborted": "aborted"})
    

sis = smach_ros.IntrospectionServer('pick_sm', sm, '/pick')
sis.start()

outcome = sm.execute()

rospy.spin()
sis.stop()
```
**Note:** In this example, the Tahoma object ```robot``` we created is passed to ```motion.MoveEndEffectorToPose(robot)```. ```robot.move_to_pose_goal(pose)``` is called inside ```motion.MoveEndEffectorToPose()```.

## Video Demonstration
