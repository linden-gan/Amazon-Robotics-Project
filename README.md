# Amazon-Robotics-Project

This is a pybullet interface for motion planning to move robot to a desired end pose.

## Download
```
git clone https://github.com/linden-gan/Amazon-Robotics-Project.git
```
This repo must be cloned under the **common directory** of the aurmr_tahoma repo: aurmr_tahoma/aurmr_task/src/common

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
- step 1: construct a Tahoma object
- 

## Usage Example

## Video Demonstration
