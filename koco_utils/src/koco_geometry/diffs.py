#! /usr/bin/env python

import logging
import numpy as np
from tf.transformations import quaternion_multiply, quaternion_conjugate


def quat_diff(q1, q2):
    """
    Calculate the difference in angle between two quaternions
    @param q1: Quaternion from
    @param q2: Quaternion to
    @type q1: np.array or list
    @type q2: np.array or list
    @return: A quaternion as list
    @rtype: float
    """

    # Make sure both quaternions have the same signs
    # q2 = np.sign(q2[-1]) * np.asarray(q2)
    # q1 = np.sign(q1[-1]) * np.asarray(q1)

    return quaternion_multiply(q1, quaternion_conjugate(q2))


def quat_log(q):
    log_q = np.array([0, 0, 0])
    if (np.linalg.norm(q[0:3]) > 1.0e-12):
        log_q = np.arccos(q[3]) * q[0:3] / np.linalg.norm(q[0:3])
        if np.linalg.norm(log_q) > np.pi:
            log_q = (2 * np.pi - 2 * np.arccos(q[3])) * (-q[0:3]) / np.linalg.norm(q[0:3])
    return log_q


def quat_to_angle(q):
    """
    Calculate the total rotation angle (in radians) a quaternion represents
    @param q: Quaternion
    @type q: np.array or list
    @return: Angle in radians
    @rtype: float
    """
    return 2 * np.linalg.norm(quat_log(q))


def quat_angle_diff(q1, q2):
    return quat_to_angle(quat_diff(q1, q2))


def position_diff(p1, p2):
    """
    Calculate the distance between two points
    @param p1: Point from
    @param p2: Point to
    @type p1: np.array or list
    @type p2: np.array or list
    @return: Distance between two points in meters
    @rtype: float
    """
    pos_diff = np.linalg.norm(np.asarray(p1) - np.asarray(p2))
    return pos_diff


def pose_diff(pose1, pose2):
    """
    Calculate the distance between two poses, both positional and rotational.
    @param pose1: Pose from
    @param pose2: Pose to
    @type pose1: np.array or geometry_msgs.msg.PoseStamped
    @type pose2: np.array or geometry_msgs.msg.PoseStamped
    """
    p1 = [0, 0, 0]
    p2 = [0, 0, 0]
    q1 = [0, 0, 0, 1]
    q2 = [0, 0, 0, 1]
    try:
        p2 = pose2[0:3]
        q2 = pose2[3:]
    except Exception:
        try:
            p2 = [pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z]
            q2 = [
                pose2.pose.orientation.x,
                pose2.pose.orientation.y,
                pose2.pose.orientation.z,
                pose2.pose.orientation.w]
        except Exception:
            try:
                p2 = [pose2.position.x, pose2.position.y, pose2.position.z]
                q2 = [
                    pose2.orientation.x,
                    pose2.orientation.y,
                    pose2.orientation.z,
                    pose2.orientation.w]
            except Exception:
                logging.error("WRONG POSE2 INPUT FOR pose_diff !!")
                logging.error("pose2=\n{0}".format(pose2))
    try:
        p1 = pose1[0:3]
        q1 = pose1[3:]
    except Exception:
        try:
            p1 = [pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z]
            q1 = [
                pose1.pose.orientation.x,
                pose1.pose.orientation.y,
                pose1.pose.orientation.z,
                pose1.pose.orientation.w]
        except Exception:
            try:
                p1 = [pose1.position.x, pose1.position.y, pose1.position.z]
                q1 = [
                    pose1.orientation.x,
                    pose1.orientation.y,
                    pose1.orientation.z,
                    pose1.orientation.w]
            except Exception:
                logging.error("WRONG POSE1 INPUT FOR pose_diff !!")
                logging.error("pose1=\n{0}".format(pose1))

    p_diff = position_diff(p1, p2)
    q_diff = quat_to_angle(quat_diff(q1, q2))
    return [p_diff, q_diff, np.linalg.norm([p_diff, q_diff])]
