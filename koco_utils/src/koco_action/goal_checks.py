import numpy as np

import rospy

from geometry_msgs.msg import Quaternion

from koco_geometry.diffs import pose_diff
from koco_conversions.pose_conversions import (
    convert_array_7_to_pose,
    transform_to_pose,
    matrix_to_pose)


def _get_flange_pose(robot_pose):
    """Helper function that returns the pose of the robot's flange.

    Args:
        robot_pose (koco_msgs/KocoPose): The current pose of the robot
    """
    return transform_to_pose(robot_pose.tool0_pose)  # Legacy naming ...


def _get_tcp_pose(robot_pose):
    """Helper function that returns the pose of the robot's TCP.

    Args:
        robot_pose (koco_msgs/KocoPose): The current pose of the robot's TCP
    """
    return transform_to_pose(robot_pose.tcp_pose)  # Legacy naming ...


def is_movel_within_threshold(robot_pose, movel_goal, threshold):
    """Checks if the distance between the provided pose and the provided MoveL goal
    is within the threshold.

    Args:
        robot_pose (koco_msgs/KocoPose): The current pose of the robot. The transform between `tool0_pose`
                                         and 'tcp_pose' field must match the transform of the tool that we
                                         want the robot to move to the goal with.
        movel_goal (koco_msgs/MoveLGoal): The MoveJ goal of the action
        threshold (float or list(float)): The positional [mm] and (optional) rotational [deg] offset

    Returns:
        bool: True or False if the goal is within the provided threshold

    """
    robot_pose = _get_tcp_pose(robot_pose)

    if is_target_pose_msg(movel_goal):
        target_pose = movel_goal.target_pose
    elif movel_goal.target_array:
        target_pose = convert_array_7_to_pose(movel_goal.target_array)
    else:
        raise KeyError("Cannot parse goal {0}\nof type {1}.".format(movel_goal, type(movel_goal)))

    pos_diff, quat_diff, _ = pose_diff(robot_pose, target_pose)
    quat_diff = correct_quat_diff(quat_diff)
    rospy.logdebug("[MoveL] Current pose: {}".format(robot_pose))
    rospy.logdebug("[MoveL] Target pose: {}".format(target_pose))
    rospy.logdebug("[MoveL] pos_diff: {}".format(pos_diff))
    rospy.logdebug("[MoveL] quat_diff: {}".format(quat_diff))
    return is_diff_within_threshold(pos_diff, quat_diff, threshold)


def is_movej_within_threshold(robot_pose, movej_goal, threshold, kin_model=None):
    """Checks if the distance between the provided pose and the provided MoveJ goal
    is within the threshold.

    Args:
        robot_pose (koco_msgs/KocoPose): The current pose of the robot. The transform between `tool0_pose`
                                         and 'tcp_pose' field must match the transform of the tool that we
                                         want the robot to move to the goal with.
        movej_goal (koco_msgs/MoveJGoal): The MoveJ goal of the action
        threshold (float or list(float)): The positional [mm] and (optional) rotational [deg] offset
        kin_model (KDLKinematics): The kinematic model for the robot until the robot's flange

    Returns:
        bool: True or False if the goal is within the provided threshold

    Note:
        When providing the kinematic model of the robot it is very important to
        create it properly. The general syntax of KDLKinemacitcs is the following:
        ```
            kin_model = KDLKinematics(urdf, base_link, end_link)
        ```
        The `urdf` can be retrieved from the parameter server. Take care
        that you provide the `base_link` and `end_link` of the robot you're
        using. Especially important that they match the URDF.
    """

    if is_target_pose_msg(movej_goal):
        target_pose = movej_goal.target_pose
        robot_pose = _get_tcp_pose(robot_pose)
    elif kin_model is not None:
        if movej_goal.target_joint.position:
            joints_array = movej_goal.target_joint.position
        elif movej_goal.target_array:
            joints_array = movej_goal.target_array
        else:
            raise AttributeError("Provided joints not in correct format. Check goal {}".format(movej_goal))
        target_pose = matrix_to_pose(np.asarray(kin_model.forward(joints_array)))
        robot_pose = _get_flange_pose(robot_pose)
    else:
        raise KeyError("Cannot parse goal {0}\nof type {1}.".format(movej_goal, type(movej_goal)))

    rospy.logdebug("[MoveJ] Current pose: {}".format(robot_pose))
    rospy.logdebug("[MoveJ] Target pose: {}".format(target_pose))
    pos_diff, quat_diff, _ = pose_diff(robot_pose, target_pose)
    quat_diff = correct_quat_diff(quat_diff)
    rospy.logdebug("[MoveJ] pos_diff: {}".format(pos_diff))
    rospy.logdebug("[MoveJ] quat_diff: {}".format(quat_diff))
    return is_diff_within_threshold(pos_diff, quat_diff, threshold)


def correct_quat_diff(quat_diff):
    if abs(quat_diff) > np.pi:
        return np.sign(quat_diff) * 2 * np.pi - quat_diff
    else:
        return quat_diff


def is_target_pose_msg(goal):
    """Helper function that checks whether the goal was provided in the form of
    geometry_msgs/Pose format."""
    return goal.target_pose.orientation != Quaternion()


def is_diff_within_threshold(pos_diff, quat_diff, threshold):
    """Helper function that checks wheter the difference between two poses
    is within the defined threshold."""
    diff_array = [pos_diff, quat_diff]
    less_than_array = np.less_equal(diff_array, threshold)
    is_close_array = np.isclose(diff_array, threshold)

    return all(less_than_array | is_close_array)
