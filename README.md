# pybullet-motion-plan

This is a motion library to move robot arm to desired end poses.

We use Pybullet to achieve motion planning as well as collision checking. To overcome the low-granularity of the path returned by original Pybullet library function, we tweak the algorithm so that it now always returns paths with high-granularity.

Here is the structure of this project:

```
3D                        --- directory for any additional, user-defined 3D model
aurmr/src/                --- a ROS package containing the whole motion library/
    robot_api             --- a prototype api that has no practical purpose, but a demo of how to use Pybullet for motion planning
    robot_info            --- urdf, mesh, and other meta data for robots in robot_api
    scripts               --- directory for all demo scripts
    tahoma_api/           --- an api that can control the tahoma robot
        tahoma_info       --- urdf, mesh, and other meta data for tahoma robot
        pb_tahoma.py      --- a class that controls robot in gazebo
        real_tahoma.py    --- a class that controls the real robot
                              The reason to have 2 classes is real robot uses different topics and other parameters than the 
                              gazebo version. Having 2 "redundant" files actually makes things easier.
```

**NOTE:** This repo is designed for the robot in Amazon Picking Challenge, an ongoing research project in Sensor System Lab in University of Washington. If you directly download this repo for other robot, it may not be compatible. Please see **Usage** section if you need modify any parameters.

## Video Demonstration
Welcome to checkout our video demonstration [HERE](https://youtu.be/GZF41x6FVtA)!

[![demo](http://img.youtube.com/vi/GZF41x6FVtA/0.jpg)](https://youtu.be/GZF41x6FVtA)


## Download
```
git clone https://github.com/linden-gan/Amazon-Robotics-Project.git
```
This repo must be cloned under the **common directory** of the aurmr_tahoma repo: ```aurmr_tahoma/aurmr_task/src/common``` 

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
- ```end_pose:geometry_msgs/PoseStamped``` The desired pose of end-effector.
- ```max_plan_time:float, default=10.0``` The maximum allowed trajectory planning time for pybulley planning.
- ```execution_timeout:float, default=15``` The maximum allowed time to wait for the real robot to execute the planned trajectory.
- ```tolerance:float, defualt=0.01``` The tolerance of joint angles in rad.
- ```orientation_constraint:moveit_msgs/OrientationConstraint, default=None```  An orientation constraint for the entire trajectory.

#### Return
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

rospy.init_node("move_arm_demo")  # Users can change this node name
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
**NOTE:** In this example, the Tahoma object ```robot``` we created is passed to ```motion.MoveEndEffectorToPose(robot)```. ```robot.move_to_pose_goal(pose)``` is called inside ```motion.MoveEndEffectorToPose()```.
