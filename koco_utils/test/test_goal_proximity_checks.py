#! /usr/bin/env python

import unittest
import rosunit

from urdf_parser_py import urdf
from pykdl_utils.kdl_kinematics import KDLKinematics

from geometry_msgs.msg import Transform, Pose, Point, Quaternion
from koco_msgs.msg import KocoPose, MoveJGoal, MoveLGoal
from sensor_msgs.msg import JointState

from koco_action import goal_checks
from koco_conversions.pose_conversions import convert_matrix_to_transform, convert_pose_to_array_7

THRESHOLD = 0.1


def create_fake_robot():
    robot = urdf.Robot(name='fakie', version='1.0')
    links = [
        urdf.Link(name='link_{}'.format(i)) for i in range(7)]
    joints = [
        urdf.Joint(
            name='link_{}'.format(i),
            joint_type='revolute',
            parent='link_{}'.format(i),
            child='link_{}'.format(i + 1),
            origin=urdf.Pose(xyz=[0, 0, 1]),
            axis=[0, 0, 1]) for i in range(6)]

    for joint in joints:
        robot.add_joint(joint)
    for link in links:
        robot.add_link(link)

    return KDLKinematics(robot, 'link_0', 'link_6')


class TestGoalCheckers(unittest.TestCase):

    @classmethod
    def setUpClass(cls):

        # Define the positions and poses we want to check for
        valid_quaternion = Quaternion(w=1)
        robot_position = Point(z=1)
        too_close_position = Point(z=(robot_position.z + THRESHOLD))
        too_close_pose = Pose(position=too_close_position, orientation=valid_quaternion)
        same_position = Point(z=robot_position.z)
        same_pose = Pose(position=same_position, orientation=valid_quaternion)
        far_enough_position = Point(z=(robot_position.z + THRESHOLD * 1.01))
        far_enough_pose = Pose(position=far_enough_position, orientation=valid_quaternion)

        # Define the joints we want to check for
        invalid_joints = JointState()
        cls.robot_joints = JointState(position=list(range(6)))
        too_close_joints = cls.robot_joints.position[:5] + [cls.robot_joints.position[5] + THRESHOLD]
        same_joints = cls.robot_joints.position
        far_enough_joints = cls.robot_joints.position[:5] + [cls.robot_joints.position[5] + THRESHOLD * 1.01]

        # To check if the methods return the appropriate exceptions
        cls.invalid_pose = Pose()
        cls.valid_pose = Pose(orientation=valid_quaternion)

        # Create the robot kinematic model for joint-type comparisons
        cls.robot_kinematics = create_fake_robot()
        pose_mat = cls.robot_kinematics.forward(cls.robot_joints.position)
        cls.koco_pose_from_joints = KocoPose(tool0_pose=convert_matrix_to_transform(pose_mat))

        # The robot pose to be compared in pose-type comparisons
        cls.robot_pose = KocoPose(
            tool0_pose=Transform(translation=robot_position, rotation=valid_quaternion),
            tcp_pose=Transform(translation=robot_position, rotation=valid_quaternion))

        # Define the MoveL type goals
        cls.move_l_invalid = MoveLGoal(target_pose=cls.invalid_pose)
        cls.move_l_too_close_pose = MoveLGoal(target_pose=too_close_pose)
        cls.move_l_same_pose = MoveLGoal(target_pose=same_pose)
        cls.move_l_far_enough_pose = MoveLGoal(target_pose=far_enough_pose)
        cls.move_l_too_close_array = MoveLGoal(target_array=convert_pose_to_array_7(too_close_pose))
        cls.move_l_same_array = MoveLGoal(target_array=convert_pose_to_array_7(same_pose))
        cls.move_l_far_enough_array = MoveLGoal(target_array=convert_pose_to_array_7(far_enough_pose))

        # Define the MoveJ type goals
        cls.move_j_invalid_pose = MoveJGoal(target_pose=cls.invalid_pose)
        cls.move_j_too_close_pose = MoveJGoal(target_pose=too_close_pose)
        cls.move_j_same_pose = MoveJGoal(target_pose=same_pose)
        cls.move_j_far_enough_pose = MoveJGoal(target_pose=far_enough_pose)
        cls.move_j_invalid_joints = MoveJGoal(target_joint=invalid_joints)
        cls.move_j_too_close_joints = MoveJGoal(target_joint=JointState(position=too_close_joints))
        cls.move_j_same_joints = MoveJGoal(target_joint=JointState(position=same_joints))
        cls.move_j_far_enough_joints = MoveJGoal(target_joint=JointState(position=far_enough_joints))
        cls.move_j_too_close_array = MoveJGoal(target_array=too_close_joints)
        cls.move_j_same_array = MoveJGoal(target_array=same_joints)
        cls.move_j_far_enough_array = MoveJGoal(target_array=far_enough_joints)

    def test_movej_invalid_pose(self):
        with self.assertRaises(KeyError):
            goal_checks.is_movej_within_threshold(self.robot_pose, self.move_j_invalid_pose, THRESHOLD)

    def test_movej_no_kin_model(self):
        with self.assertRaises(KeyError):
            goal_checks.is_movej_within_threshold(self.robot_pose, self.move_j_too_close_joints, THRESHOLD)

    def test_movej_invalid_joints(self):
        with self.assertRaises(AttributeError):
            goal_checks.is_movej_within_threshold(
                self.robot_pose, self.move_j_invalid_joints, THRESHOLD, self.robot_kinematics)

    def test_movej_too_close_pose(self):
        self.assertTrue(
            goal_checks.is_movej_within_threshold(self.robot_pose, self.move_j_too_close_pose, THRESHOLD))

    def test_movej_too_close_joint(self):
        self.assertTrue(
            goal_checks.is_movej_within_threshold(
                self.koco_pose_from_joints, self.move_j_too_close_joints, THRESHOLD, self.robot_kinematics))

    def test_movej_too_close_array(self):
        self.assertTrue(
            goal_checks.is_movej_within_threshold(
                self.koco_pose_from_joints, self.move_j_too_close_array, THRESHOLD, self.robot_kinematics))

    def test_movej_same_pose(self):
        self.assertTrue(goal_checks.is_movej_within_threshold(self.robot_pose, self.move_j_same_pose, THRESHOLD))

    def test_movej_same_joint(self):
        self.assertTrue(
            goal_checks.is_movej_within_threshold(
                self.koco_pose_from_joints, self.move_j_same_joints, THRESHOLD, self.robot_kinematics))

    def test_movej_same_array(self):
        self.assertTrue(
            goal_checks.is_movej_within_threshold(
                self.koco_pose_from_joints, self.move_j_same_array, THRESHOLD, self.robot_kinematics))

    def test_movej_far_enough_pose(self):
        self.assertFalse(
            goal_checks.is_movej_within_threshold(self.robot_pose, self.move_j_far_enough_pose, THRESHOLD))

    def test_movej_far_enough_joint(self):
        self.assertFalse(
            goal_checks.is_movej_within_threshold(
                self.koco_pose_from_joints, self.move_j_far_enough_joints, THRESHOLD, self.robot_kinematics))

    def test_movej_far_enough_array(self):
        self.assertFalse(
            goal_checks.is_movej_within_threshold(
                self.koco_pose_from_joints, self.move_j_far_enough_array, THRESHOLD, self.robot_kinematics))

    def test_movel_invalid_pose(self):
        with self.assertRaises(KeyError):
            goal_checks.is_movel_within_threshold(self.robot_pose, self.move_l_invalid, THRESHOLD)

    def test_movel_too_close_pose(self):
        self.assertTrue(
            goal_checks.is_movel_within_threshold(self.robot_pose, self.move_l_too_close_pose, THRESHOLD))

    def test_movel_too_close_array(self):
        self.assertTrue(
            goal_checks.is_movel_within_threshold(
                self.robot_pose, self.move_l_too_close_array, THRESHOLD))

    def test_movel_same_pose(self):
        self.assertTrue(
            goal_checks.is_movel_within_threshold(self.robot_pose, self.move_l_same_pose, THRESHOLD))

    def test_movel_same_array(self):
        self.assertTrue(
            goal_checks.is_movel_within_threshold(
                self.robot_pose, self.move_l_same_array, THRESHOLD))

    def test_movel_far_enough_pose(self):
        self.assertFalse(
            goal_checks.is_movel_within_threshold(self.robot_pose, self.move_l_far_enough_pose, THRESHOLD))

    def test_movel_far_enough_array(self):
        self.assertFalse(
            goal_checks.is_movel_within_threshold(
                self.robot_pose, self.move_l_far_enough_array, THRESHOLD))


if __name__ == '__main__':
    PKG = 'koco_utils'
    rosunit.unitrun(PKG, 'test_goal_proximity_checks', TestGoalCheckers)
