from termcolor import cprint
import pybullet as p
import pybullet_planning as pp


def move_to(robot, joint_indices, pose: list, orien: list, obstacles: list,
            disabled_collision_links, debug=False):
    """
    move the gripper to destination pose and orien
    :param robot: robot descriptor
    :param joint_indices: a list of joint descriptors
    :param pose: destination position
    :param orien: destination orientation
    :param obstacles: obstacles to avoid
    :param disabled_collision_links: disabled self-collision links
    :param debug: debug flag
    :return:
    """
    # compute joint destination according to end effector position
    des_config = p.calculateInverseKinematics(robot, joint_indices[-1],
                                              pose, orien)

    # plan motion
    path = pp.plan_joint_motion(robot, joint_indices, des_config,
                                obstacles=obstacles, self_collisions=True,
                                disabled_collisions=disabled_collision_links,
                                algorithm='rrt')

    # visualize end configuration
    if debug:
        pp.set_joint_positions(robot, joint_indices, des_config)
        pp.wait_for_user()

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
