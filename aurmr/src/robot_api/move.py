from termcolor import cprint
import pybullet as p
import pybullet_planning as pp

# Collision
# MAX_DISTANCE = 1e-3
# Used in collision checking query, e.g. pybullet.getClosestPoint
# If the distance between objects exceeds this maximum distance, no points may be returned.
MAX_DISTANCE = 0.


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
    path = plan_motion(robot, joint_indices, des_config,
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


def plan_motion(body, joints, end_conf, obstacles=[], attachments=[],
                      self_collisions=True, disabled_collisions=set(), extra_disabled_collisions=set(),
                      weights=None, resolutions=None, max_distance=MAX_DISTANCE, custom_limits={}, diagnosis=False, **kwargs):
    """call birrt to plan a joint trajectory from the robot's **current** conf to ``end_conf``.
    """
    assert len(joints) == len(end_conf)
    sample_fn = pp.get_sample_fn(body, joints, custom_limits=custom_limits)
    distance_fn = pp.get_distance_fn(body, joints, weights=weights)
    extend_fn = pp.get_extend_fn(body, joints, resolutions=resolutions)
    collision_fn = pp.get_collision_fn(body, joints, obstacles=obstacles, attachments=attachments, self_collisions=self_collisions,
                                    disabled_collisions=disabled_collisions, extra_disabled_collisions=extra_disabled_collisions,
                                    custom_limits=custom_limits, max_distance=max_distance)

    start_conf = pp.get_joint_positions(body, joints)

    if not pp.check_initial_end(start_conf, end_conf, collision_fn, diagnosis=diagnosis):
        return None
    return birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs)


def birrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param kwargs: Keyword arguments
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    solutions = random_restarts(pp.rrt_connect, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                                max_solutions=1, **kwargs)
    if not solutions:
        return None
    return solutions[0]


def random_restarts(solve_fn, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                    restarts=pp.RRT_RESTARTS, smooth=pp.RRT_SMOOTHING,
                    success_cost=0., max_time=pp.INF, max_solutions=1, verbose=False, **kwargs):
    """Apply random restarts to a given planning algorithm to obtain multiple solutions.

    Parameters
    ----------
    solve_fn : function handle
        motion planner function, e.g. ``birrt``
    start : list
        start conf
    goal : list
        end conf
    distance_fn : function handle
        Distance function - `distance_fn(q1, q2)->float`
        see `pybullet_planning.interfaces.planner_interface.joint_motion_planning.get_difference_fn` for an example
    sample_fn : function handle
        configuration space sampler - `sample_fn()->conf`
        see `pybullet_planning.interfaces.planner_interface.joint_motion_planning.get_sample_fn` for an example
    extend_fn : function handle
        Extension function - `extend_fn(q1, q2)->[q', ..., q"]`
        see `pybullet_planning.interfaces.planner_interface.joint_motion_planning.get_extend_fn` for an example
    collision_fn : function handle
        Collision function - `collision_fn(q)->bool`
        see `pybullet_planning.interfaces.robots.collision.get_collision_fn` for an example
    restarts : int, optional
        number of random restarts, by default RRT_RESTARTS
    smooth : int, optional
        smoothing iterations, by default RRT_SMOOTHING
    success_cost : float, optional
        random restarts will terminate early if a path with cost lower than this number is found, by default 0.
    max_time : float, optional
        max allowed runtime, by default INF
    max_solutions : int, optional
        number of solutions wanted, random restarts will terminate early if more solutions are found, by default 1
    verbose : bool, optional
        print toggle, by default False

    Returns
    -------
    list
        list of paths, [[q', ..., q"], [[q'', ..., q""]]
    """
    start_time = pp.time.time()
    solutions = []
    path = pp.check_direct(start, goal, extend_fn, collision_fn, **kwargs)
    if path is False:
        return None
    if path is not None:
        solutions.append(path)

    detail_path = []

    for attempt in pp.irange(restarts + 1):
        if (len(solutions) >= max_solutions) or (pp.elapsed_time(start_time) >= max_time):
            break

        attempt_time = (max_time - pp.elapsed_time(start_time))
        path = solve_fn(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                        max_time=attempt_time, **kwargs)
        if path is None:
            continue

        path = pp.smooth_path(path, extend_fn, collision_fn, max_smooth_iterations=smooth,
                           max_time=max_time-pp.elapsed_time(start_time), **kwargs)

        # traverse this raggy path and connect each pair of waypoints by a detailed trajectory
        detail_path = [path[0]]
        for i in range(len(path) - 1):
            between = pp.check_direct(path[i], path[i + 1], extend_fn, collision_fn, **kwargs)
            detail_path.extend(between[1:])

        solutions.append(detail_path)
        
        # debug output
        for config in detail_path:
            print(config)

        # if find a low-cost path, then exit the loop
        if pp.compute_path_cost(detail_path, distance_fn) < success_cost:
            break
        

    solutions = sorted(solutions, key=lambda detail_path: pp.compute_path_cost(detail_path, distance_fn))
    if verbose:
        print('Solutions ({}): {} | Time: {:.3f}'.format(len(solutions), [(len(detail_path), round(pp.compute_path_cost(
            detail_path, distance_fn), 3)) for detail_path in solutions], pp.elapsed_time(start_time)))
    return solutions
