#!/usr/bin/env python

from ntpath import join
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc
import pybullet_planning as pp

import os
from termcolor import cprint

# [-2.465935500321586, -0.3614325767215414, 1.2921673670941827, 0.14091580293729233, -0.03903485799066875, -0.46069111706169075, 0.2890467944762829]

HERE = os.path.dirname(__file__)

ROBOT = os.path.join(HERE, 'tahoma_info', 'tahoma.urdf')
POD = os.path.join(HERE, 'tahoma_info', 'pod1.urdf')

PYBULLET_JOINT_INITIAL = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

JOINT_NAMES = ['arm_elbow_joint', 'arm_shoulder_lift_joint', 'arm_shoulder_pan_joint', 'arm_wrist_1_joint',
               'arm_wrist_2_joint', 'arm_wrist_3_joint'] #, 'gripper_finger_joint'
GRIPPER = 'gripper_robotiq_arg2f_base_link'

# SELF_COLLISION_DISABLED_LINKS = [  # change /////////////////////////////////////////////////////////////////////
#         ('stand', 'base_link_inertia'), ('stand', 'shoulder_link'), ('stand', 'upper_arm_link'),
#         ('base_link_inertia', 'shoulder_link'), ('shoulder_link', 'upper_arm_link'),
#         ('upper_arm_link', 'forearm_link'), ('forearm_link', 'wrist_1_link'), ('wrist_1_link', 'wrist_2_link'),
#         ('wrist_2_link', 'wrist_3_link')]
SELF_COLLISION_DISABLED_LINKS = [('stand', 'arm_base_link_inertia'), ('stand', 'arm_shoulder_link'), ('stand', 'arm_upper_arm_link'),
                                 ('arm_base_link_inertia', 'arm_shoulder_link'), ('arm_shoulder_link', 'arm_upper_arm_link'),
                                 ('arm_upper_arm_link', 'arm_forearm_link'), ('arm_forearm_link', 'arm_wrist_1_link'), ('arm_wrist_1_link', 'arm_wrist_2_link'),
                                 ('arm_wrist_2_link', 'arm_wrist_3_link')]

# joint server on actual robot
JOINT_ACTION_SERVER = '/pos_joint_traj_controller/follow_joint_trajectory'
# topic about robot's current actual state
ACTUAL_STATE_TOPIC = "/joint_states"
# # topic about robot's gripper pose
# ACTUAL_END_POSE = ""

THRESHOLD = 0.05

REAL_TIME_STAMP = 0.1


class Tahoma:
    def __init__(self) -> None:
        # initialize robot in pybullet
        p.connect(p.GUI)
        self._pod = p.loadURDF(POD)
        self._pb_robot = p.loadURDF(ROBOT, basePosition=[0.5, -0.6, 0], baseOrientation=[0, 0, 1, 1])  # ask Collin about this

        # initialize arm meta info
        self._joint_indices = pp.joints_from_names(self._pb_robot, JOINT_NAMES)
        self._disabled_links = pp.get_disabled_collisions(self._pb_robot, SELF_COLLISION_DISABLED_LINKS)  #######################

        # initialize joint state of actual robot
        self._actual_joint_state = None
        # # initialize end effector pose of actual robot
        # self._actual_end_pose = None

        # listen to robot's actual joint state
        self._robot_state_sub = rospy.Subscriber(ACTUAL_STATE_TOPIC, JointState, callback=self.get_actual_state)
        # # listen to robot's actual end effector's pose
        # self._gripper_pose_sub = rospy.Subscriber(ACTUAL_END_POSE, PoseStamped, callback=self.get_actual_end_pose)
        # initialize action client to move robot's arm
        self._trajectory_client = actionlib.SimpleActionClient(JOINT_ACTION_SERVER, FollowJointTrajectoryAction)

        while self._actual_joint_state is None:
            pass

        # synchronize pybullet simulator's initial pose to actual robot's
        self.synchronize()

    def get_actual_state(self, msg: JointState):
        self._actual_joint_state = msg.position[:6]

    # def get_actual_end_pose(self, msg: PoseStamped):
    #     self._actual_end_pose = msg.pose

    def synchronize(self):
        # synchronize simulator's states to actual states
        pp.wait_for_duration(REAL_TIME_STAMP * 10)
        cprint(f'actual joint states are {self._actual_joint_state}', "yellow")
        pp.set_joint_positions(self._pb_robot, self._joint_indices, self._actual_joint_state)

    def move_to_pose_goal(self,
                          end_pose: PoseStamped,
                          max_plan_time=10.0,
                          execution_timeout=15,
                          tolerance=0.01,
                          orientation_constraint=None):
        self.synchronize()
        # use IK to compute end joint state according to end effector position
        position = [end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z]
        orien = [end_pose.pose.orientation.x, end_pose.pose.orientation.y,
                 end_pose.pose.orientation.z, end_pose.pose.orientation.w]
        mark_target(position)
        end_joint_state = p.calculateInverseKinematics(self._pb_robot, pp.link_from_name(self._pb_robot, GRIPPER),
                                                       position, orien)
        # plan motion
        path = plan_motion_from_to(self._pb_robot, self._joint_indices, self._actual_joint_state, end_joint_state,
                                   obstacles=[], self_collisions=True,
                                   disabled_collisions=self._disabled_links,
                                   algorithm='rrt')
        print(f'path is {path}')

        if path is None:
            cprint('Pybullet cannot find any path...', 'red')
            self.synchronize()
            return False

        # meanwhile, move arm in pybullet sim
        sim_execute_motion(self._pb_robot, self._joint_indices, path)

        # make goal
        goal = FollowJointTrajectoryGoal()
        # for trajectory
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points = to_joint_trajectory_point(path)
        # for tolerance
        goal.path_tolerance = to_joint_tolerance(0.018)
        goal.goal_tolerance = to_joint_tolerance(0.009)
        # for goal_time_tolerance
        goal.goal_time_tolerance = rospy.Duration(10 * REAL_TIME_STAMP)
        # send goal
        self._trajectory_client.send_goal(goal)
        cprint("Sent this goal", 'yellow')
        # wait to get result
        self._trajectory_client.wait_for_result(rospy.Duration(execution_timeout))
        # return result
        result = self._trajectory_client.get_result()
        self.synchronize()
        if result is not None:
            if result.error_code == 0:
                cprint('SUCCESS!', 'cyan')
                return True
            else:
                cprint(f'FAILURE: error string is {result.error_string}', 'red')
                return False
        
        cprint("Got no result", 'yellow')

        # change to comparing goal and actual pose
        return all_close(self._actual_joint_state, end_joint_state, THRESHOLD)


def all_close(state1: list, state2: list, threshold):
    assert len(state1) == len(state2)
    for i in range(len(state1)):
        if abs(state1[i] - state2[i]) > threshold:
            return False
    return True


def to_joint_trajectory_point(waypoints):
    """
    transform a list of waypoints of joint states to a list of JointTrajectoryPoint
    """
    res = []
    for i, point in enumerate(waypoints):
        dur = rospy.Duration(i * REAL_TIME_STAMP)
        res.append(JointTrajectoryPoint(positions=point, time_from_start=dur))
    return res


def to_joint_tolerance(joint_tol):
    """
    transform a tolerance float value to a list of JointTolerance objects.
    for simplicity, we set all joints' position, velocity, and acceleration to the same tolerance value.
    """
    res = []
    for i in range(6):
        obj = JointTolerance()
        obj.position = joint_tol
        obj.velocity = joint_tol
        obj.acceleration = joint_tol
        obj.name = JOINT_NAMES[i]
        res.append(obj)
    return res


def mark_target(position: list):
    target = pp.create_box(0.1, 0.1, 0.1)
    pp.set_pose(target, pp.Pose(pp.Point(x=position[0], y=position[1], z=position[2])))

def sim_execute_motion(robot, joint_indices, path):
    """
    execute path in pybullet
    """
    time_step = 0.03
    for conf in path:
        pp.set_joint_positions(robot, joint_indices, conf)
        pp.wait_for_duration(time_step)


def plan_motion(body, joints, end_conf, obstacles=[], attachments=[],
                self_collisions=True, disabled_collisions=set(), extra_disabled_collisions=set(),
                weights=None, resolutions=None, max_distance=0.0, custom_limits={}, diagnosis=False, **kwargs):
    """call birrt to plan a joint trajectory from the robot's **current** conf to ``end_conf``.
    """
    assert len(joints) == len(end_conf)
    sample_fn = pp.get_sample_fn(body, joints, custom_limits=custom_limits)
    distance_fn = pp.get_distance_fn(body, joints, weights=weights)
    extend_fn = pp.get_extend_fn(body, joints, resolutions=resolutions)
    collision_fn = pp.get_collision_fn(body, joints, obstacles=obstacles, attachments=attachments,
                                       self_collisions=self_collisions,
                                       disabled_collisions=disabled_collisions,
                                       extra_disabled_collisions=extra_disabled_collisions,
                                       custom_limits=custom_limits, max_distance=max_distance)

    start_conf = pp.get_joint_positions(body, joints)

    if not pp.check_initial_end(start_conf, end_conf, collision_fn, diagnosis=diagnosis):
        return None
    return birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs)


def plan_motion_from_to(body, joints, start_conf, end_conf, obstacles=[], attachments=[],
                        self_collisions=True, disabled_collisions=set(), extra_disabled_collisions=set(),
                        weights=None, resolutions=None, max_distance=0.0, custom_limits={}, diagnosis=False,
                        **kwargs):
    """
    call birrt to plan a joint trajectory from the robot's start_conf conf to end_conf
    """
    assert len(joints) == len(end_conf)
    sample_fn = pp.get_sample_fn(body, joints, custom_limits=custom_limits)
    distance_fn = pp.get_distance_fn(body, joints, weights=weights)
    extend_fn = pp.get_extend_fn(body, joints, resolutions=resolutions)
    collision_fn = pp.get_collision_fn(body, joints, obstacles=obstacles, attachments=attachments,
                                       self_collisions=self_collisions,
                                       disabled_collisions=disabled_collisions,
                                       extra_disabled_collisions=extra_disabled_collisions,
                                       custom_limits=custom_limits, max_distance=max_distance)

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
                              max_time=max_time - pp.elapsed_time(start_time), **kwargs)

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
