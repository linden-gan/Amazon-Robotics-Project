import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc
import sys
import time
import os
from termcolor import cprint
import pybullet_planning as pp
import numpy as np

HERE = os.path.dirname(__file__)
# ROBOT_URDF = os.path.join(HERE, 'robot_info', 'robot.urdf')
ROBOT_URDF = os.path.join(HERE, 'robot_info', 'robot_with_stand.urdf')

X = 0.5
Y = 1.0
Z = 0.0


def main():
    # p.getCameraImage()
    pp.connect(use_gui=True, mp4='video.mp4')
    robot = pp.load_pybullet(ROBOT_URDF, fixed_base=True)

    # gravity
    p.setGravity(0, 0, -9.8)

    # get joint indices
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
                   'wrist_2_joint', 'wrist_3_joint']
    joint_indices = pp.joints_from_names(robot, joint_names)
    # alternative way to get joints
    # joint_names = pp.get_movable_joints(robot)
    # cprint('Current joint state: {}'.format(pp.get_joint_state(robot, joint_indices[0])))

    # disable adjacent links for self collision check
    robot_self_collision_disabled_link_names = [
        ('stand', 'base_link_inertia'), ('stand', 'shoulder_link'), ('stand', 'upper_arm_link'),
        ('base_link_inertia', 'shoulder_link'), ('shoulder_link', 'upper_arm_link'),
        ('upper_arm_link', 'forearm_link'), ('forearm_link', 'wrist_1_link'), ('wrist_1_link', 'wrist_2_link'),
        ('wrist_2_link', 'wrist_3_link')]
    self_collision_links = pp.get_disabled_collisions(robot, robot_self_collision_disabled_link_names)

    # test
    pose0 = [-0.6, 0.6, 1.0]
    orien0 = [0, 0, 0, 1]
    move_test(robot, joint_indices, pose0, orien0, [0.4, 1.7, 1.0], [0, 0, 0, 1], self_collision_links)
    # move_test(robot, joint_indices, pose0, orien0, [-0.1, 0.9, 1.0], [0, 0, 0, 1], self_collision_links)
    # ill
    # end config in collision! try a different IK solution
    move_test(robot, joint_indices, pose0, orien0, [0.4, 0.6, 1.0], [0, 0, 0, 1], self_collision_links)
    # move_test(robot, joint_indices, pose0, orien0, [-0.4, 0.6, 1.0], [0, 0, 0, 1], self_collision_links)


def move_test(robot, joint_indices, pose0, orien0, pose1, orien1,
              self_collision_links, debug=False):
    # first, move gripper to an initial position
    move_gripper_to(robot, joint_indices, pose0, orien0, [], self_collision_links)

    # create obstacles
    # block1 = pp.create_box(0.059, 0.09, 0.089)
    block2 = pp.create_box(0.1, 0.1, 3)
    # pp.set_pose(block1, pp.Pose(pp.Point(x=-0.5, y=0.5, z=0.7)))
    pp.set_pose(block2, pp.Pose(pp.Point(x=0.3, y=1.8, z=0.5)))
    pp.wait_for_user()

    # move gripper to desired ending position
    move_gripper_to(robot, joint_indices, pose1, orien1, [block2], self_collision_links, debug=debug)
    pp.wait_for_user()

    # clean obstacles for next time
    pp.remove_body(block2)


def move_gripper_to(robot, joint_indices, pose: list, orien: list, blocks: list,
                    self_collision_links, debug=False):
    # compute joint destination according to end effector position
    des_config = p.calculateInverseKinematics(robot, joint_indices[-1],
                                              pose, orien)

    # plan motion
    path = pp.plan_joint_motion(robot, joint_indices, des_config,
                                obstacles=blocks, self_collisions=True,
                                disabled_collisions=self_collision_links,
                                algorithm='rrt')

    # visualize end configuration
    # pp.set_joint_positions(robot, joint_indices, des_config)
    # pp.wait_for_user()

    if path is None:
        cprint('No path found', 'red')
        return

    # execute path
    time_step = 2.0 if debug else 0.03
    cprint(f'path length is {len(path)}', 'cyan')
    for conf in path:
        pp.set_joint_positions(robot, joint_indices, conf)
        if debug:
            print(f'config is {conf}')
        pp.wait_for_duration(time_step)


if __name__ == '__main__':
    main()
