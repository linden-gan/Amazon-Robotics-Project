import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc
import os
from termcolor import cprint
import pybullet_planning as pp
import numpy as np
import copy
import move

HERE = os.path.dirname(__file__)

VOXEL_SIZE = 0.1

ROBOT_URDF = os.path.join(HERE, 'robot_info', 'robot_with_stand.urdf')


def main():
    # initialize the env
    pp.connect(use_gui=True, mp4='video.mp4')
    robot = pp.load_pybullet(ROBOT_URDF, fixed_base=True)
    # gravity
    p.setGravity(0, 0, -9.8)

    # get joint indices
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
                   'wrist_2_joint', 'wrist_3_joint']
    joint_indices = pp.joints_from_names(robot, joint_names)

    robot_self_collision_disabled_link_names = [
        ('stand', 'base_link_inertia'), ('stand', 'shoulder_link'), ('stand', 'upper_arm_link'),
        ('base_link_inertia', 'shoulder_link'), ('shoulder_link', 'upper_arm_link'),
        ('upper_arm_link', 'forearm_link'), ('forearm_link', 'wrist_1_link'), ('wrist_1_link', 'wrist_2_link'),
        ('wrist_2_link', 'wrist_3_link')]
    self_collision_links = pp.get_disabled_collisions(robot, robot_self_collision_disabled_link_names)

    manager = VoxelManager()
    positions = [(0,2,1), (0,1,1), (0,1,1.5), (0,1,0.5), (0, 1, 0.8), (0,1,2)]
    clear_positions = [(1,0,0), (0,1,0), (0,0,1)]
    manager.fill_voxels(positions)

    # test
    pose0 = [-0.6, 0.6, 1.0]
    orien0 = [0, 0, 0, 1]
    move_test(robot, joint_indices, pose0, orien0, [0.8, 1.7, 1.0], [0, 0, 0, 1],
              self_collision_links, manager.get_all_voxels(), debug=True)

    # pp.wait_for_user()


def move_test(robot, joint_indices, pose0, orien0, pose1, orien1,
              self_collision_links, obstacles, debug=False):
    # first, move gripper to an initial position
    move.move_to(robot, joint_indices, pose0, orien0, [], self_collision_links)
    pp.wait_for_user()

    # move gripper to desired ending position
    move.move_to(robot, joint_indices, pose1, orien1, obstacles, self_collision_links, debug=debug)
    pp.wait_for_user()


class VoxelManager:
    def __init__(self):
        self._id_counter = 0
        self._map: dict = {}

    def fill_voxel(self, x, y, z):
        """
        fill a single voxel at (x, y, z)
        :param x: x coordinate of voxel
        :param y: y coordinate of voxel
        :param z: z coordinate of voxel
        :return true is success, false if this voxel is already filled
        """
        if self._map.get((x, y, z)) is not None:
            return False

        block = pp.create_box(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE)
        pp.set_pose(block, pp.Pose(pp.Point(x=x, y=y, z=z)))
        self._map[(x, y, z)] = block
        print(self._map)
        return True

    def fill_voxels(self, positions: list):
        """
        fill a list of voxels
        :param positions: a list of (x, y, z)
        """
        for pose in positions:
            self.fill_voxel(pose[0], pose[1], pose[2])

    def clear_voxel(self, x, y, z):
        if self._map[(x, y, z)] is None:
            return False

        block = self._map.pop((x, y, z))
        pp.remove_body(block)
        print(self._map)
        pp.wait_for_user()
        return True

    def clear_voxels(self, positions: list):
        for pose in positions:
            self.clear_voxel(pose[0], pose[1], pose[2])

    def get_all_voxels(self):
        return copy.copy(list(self._map.values()))


if __name__ == "__main__":
    main()
