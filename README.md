# Amazon-Robotics-Project

This is a pybullet interface for moving robot to given waypoints.

## Download
```
git clone https://github.com/linden-gan/Amazon-Robotics-Project.git
```
repo must be cloned under the **common directory** of the aurmr_tahoma repo: aurmr_tahoma/aurmr_task/src/common

## Usage
### Prereq for Usage
- ROS
- pybullet
- pybullet_planning

### Things users can modify: 
- Joint motion action server on actual robot: JOINT_ACTION_SERVER
- Time stamp for actual robot to move (smaller time stamp makes robot faster): REAL_TIME_STAMP

### Main Function to Use: 
```
move_to_pose_goal(self, end_pose: PoseStamped, max_plan_time=10.0, execution_timeout=15, tolerance=0.01, orientation_constraint=None)
```
- step 1: construct a Tahoma object
- 
