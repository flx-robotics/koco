#! /usr/bin/env python

import numpy as np
import rospy

from koco_msgs.msg import MoveSequenceActionGoal, MoveSequenceActionResult
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import unittest
import rosunit

NUM_SAMPLES = 100
SMALL_AMPLITUDE = 0.01
RATE = 10
TRAJ_PUB_RATE = 1000


class TestTrajWatchdog(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        traj_time = np.linspace(0, 2*np.pi, int(NUM_SAMPLES))
        cls.big_circ_seq = JointTrajectory()
        for t in traj_time:
            point = JointTrajectoryPoint()
            point.positions = (np.sin(t), np.cos(t))
            point.velocities = (0, 0)
            point.time_from_start = rospy.Duration(t)
            cls.big_circ_seq.points.append(point)

        cls.small_circ_seq = JointTrajectory()
        for t in traj_time:
            point = JointTrajectoryPoint()
            point.positions = (SMALL_AMPLITUDE*np.sin(t), SMALL_AMPLITUDE*np.cos(t))
            point.velocities = (0, 0)
            point.time_from_start = rospy.Duration(t)
            cls.small_circ_seq.points.append(point)

        cls.small_circ_path = 2*np.pi*SMALL_AMPLITUDE

        cls.rate = rospy.Rate(RATE)
        cls.traj_pub_rate = rospy.Rate(TRAJ_PUB_RATE)

    def setUp(self):
        # Setting the publishers and subscribers for each test
        self.create_publishers()
        self.create_subscribers()
        # The executed_traj attribute is set to a default value
        self.executed_traj = None

    def tearDown(self):
        # And tearing them down down after each test
        self.clear_subs_pubs()

    def create_subscribers(self):
        self.executed_traj_sub = rospy.Subscriber(
            'executed_trajectory', JointTrajectory, self.executed_traj_cb)
        self.stdout_sub = rospy.Subscriber('/rosout', Log, self._rosout_cb, queue_size=1)

    def create_publishers(self):
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.seq_start_pub = rospy.Publisher('/move_sequence_action/goal', MoveSequenceActionGoal, queue_size=1)
        self.seq_end_pub = rospy.Publisher('/move_sequence_action/result', MoveSequenceActionResult, queue_size=1)
        self.rate.sleep()

    def executed_traj_cb(self, msg):
        self.executed_traj = msg

    def _rosout_cb(self, msg):
        self.rosout_msg = msg.msg

    def clear_subs_pubs(self):
        self.executed_traj_sub.unregister()
        self.stdout_sub.unregister()
        self.seq_start_pub.unregister()
        self.seq_end_pub.unregister()
        self.joint_pub.unregister()

    def mark_traj_start(self):
        self.rate.sleep()  # Required so that everything works.
        self.seq_start_pub.publish(MoveSequenceActionGoal())
        self.rate.sleep()  # Without this break the watchdog can miss the first trajectory sample

    def mark_traj_end(self):
        self.rate.sleep()  # Without this break the watchdog can miss the last trajectory sample
        self.seq_end_pub.publish(MoveSequenceActionResult())
        self.rate.sleep()  # Required so that the message of the trajectory actually gets published.

    def test_empty_traj(self):
        self.mark_traj_start()
        self.mark_traj_end()
        self.assertIn("Received an 'end'", self.rosout_msg, "Did not receive the expected message.")
        self.assertIsNone(self.executed_traj)

    def test_too_short_traj(self):
        rospy.set_param('/executed_trajectory_watchdog/threshold', self.small_circ_path + 0.1)
        self.mark_traj_start()
        self.publish_traj_to_joint_topic(self.small_circ_seq)
        self.mark_traj_end()
        self.assertIn("too short", self.rosout_msg, "Did not receive the expected message.")
        self.assertIsNone(self.executed_traj)

    def test_full_traj(self):
        self.mark_traj_start()
        self.publish_traj_to_joint_topic(self.big_circ_seq)
        self.mark_traj_end()
        self.assertIs(
            len(self.big_circ_seq.points), len(self.executed_traj.points),
            "The received trajectory does not have same number of points as the sent one!")
        self.assertTrue(self.compare_traj_points(self.big_circ_seq, self.executed_traj))

    def publish_traj_to_joint_topic(self, traj):
        for point in traj.points:
            self.joint_pub.publish(self.traj_point_2_joint_state(point))
            self.traj_pub_rate.sleep()

    @staticmethod
    def traj_point_2_joint_state(traj_point):
        return(JointState(position=traj_point.positions,
                          velocity=traj_point.velocities,
                          effort=traj_point.effort))

    @staticmethod
    def compare_traj_points(traj_1, traj_2):
        traj_1_points_list = [point.positions for point in traj_1.points]
        traj_2_points_list = [point.positions for point in traj_2.points]
        return np.allclose(traj_1_points_list, traj_2_points_list)


class TestTrajWatchdogSuite(unittest.TestSuite):
    def __init__(self):
        super(TestTrajWatchdogSuite, self).__init__()
        self.addTests(map(TestTrajWatchdog, [
            'test_empty_traj',
            'test_too_short_traj',
            'test_full_traj',
        ]))


if __name__ == '__main__':
    rospy.init_node("test_traj_watchdog", log_level=rospy.INFO)
    rosunit.unitrun(
        'koco_nodes',
        'test_traj_watchdog',
        'test_executed_trajectory_watchdog.TestTrajWatchdogSuite')