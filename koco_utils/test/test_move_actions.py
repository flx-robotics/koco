#! /usr/bin/env python

import sys
import unittest

from rosgraph_msgs.msg import Log

from std_srvs.srv import Trigger

from geometry_msgs.msg import Transform
import rospy
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from industrial_msgs.msg import RobotStatus, TriState
from koco_msgs.msg import MoveJAction, MoveLAction
from koco_msgs.msg import MoveJGoal, MoveLGoal
from koco_msgs.msg import MoveSequenceAction
from koco_msgs.msg import KocoPose
from koco_msgs.srv import TriggerString


from actionlib_msgs.msg import GoalStatus

import rosunit


class DummyASClients(object):

    def __init__(self):
        rospy.init_node("dummy_test_clients", log_level=rospy.INFO)

        self.create_targets()
        self.create_clients()
        self.create_publishers()
        self.create_subscribers()
        self.create_services()

        rospy.sleep(0.2)

    def create_services(self):
        rospy.wait_for_service('empty_sequence_buffer', 1.0)
        rospy.wait_for_service('set_tool', 1.0)
        self.empty_sequence_buffer_srv = rospy.ServiceProxy('empty_sequence_buffer', Trigger)
        self.set_tool_srv = rospy.ServiceProxy('set_tool', TriggerString)

    def create_targets(self):
        self.j_1 = [0, 0, 1.57, 0, 0, 0]
        self.j_2 = [0, 0, 0, 0, 0, 0]
        self.p_1 = Pose(position=Point(*[0, 0, 0]), orientation=Quaternion(*[0, 0, 0, 1]))
        self.p_2 = Pose(position=Point(*[0, 0, 0]), orientation=Quaternion(*[0, 0, 1, 0]))

        self.j_goal_1 = MoveJGoal(target_array=self.j_1)
        self.j_goal_2 = MoveJGoal(target_joint=JointState(position=self.j_2))
        self.j_goal_p1 = MoveJGoal(target_pose=self.p_1)
        self.j_goal_p2 = MoveJGoal(target_pose=self.p_2)
        self.l_goal_p1 = MoveLGoal(target_pose=self.p_1)
        self.l_goal_p2 = MoveLGoal(target_pose=self.p_2)

    def create_clients(self):
        # Create action servers
        self.client_j = actionlib.SimpleActionClient('movej_action', MoveJAction)
        self.client_l = actionlib.SimpleActionClient('movel_action', MoveLAction)
        self.client_j_seq = actionlib.SimpleActionClient('add_seq_movej', MoveJAction)
        self.client_l_seq = actionlib.SimpleActionClient('add_seq_movel', MoveLAction)
        self.client_s = actionlib.SimpleActionClient('move_sequence_action', MoveSequenceAction)
        if not(self.client_j.wait_for_server(timeout=rospy.Duration(2))):
            rospy.logerr('client_j did not connect')
            sys.exit()
        if not(self.client_l.wait_for_server(timeout=rospy.Duration(2))):
            rospy.logerr('client_l did not connect')
            sys.exit()
        if not(self.client_j_seq.wait_for_server(timeout=rospy.Duration(2))):
            rospy.logerr('client_j_seq did not connect')
            sys.exit()
        if not(self.client_l_seq.wait_for_server(timeout=rospy.Duration(2))):
            rospy.logerr('client_l_seq did not connect')
            sys.exit()
        if not(self.client_s.wait_for_server(timeout=rospy.Duration(2))):
            rospy.logerr('client_s did not connect')
            sys.exit()
        rospy.loginfo("Connected to all action servers! Proceeding with test.")

    def create_subscribers(self):
        rospy.Subscriber('/rosout', Log, self._rosout_cb, queue_size=10)

    def create_publishers(self):
        self.robot_state_pub = rospy.Publisher('robot_status', RobotStatus, queue_size=10)
        self.joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.joint_states_fkin_pub = rospy.Publisher('joint_states_fkin', JointState, queue_size=10)
        self.robot_pose_pub = rospy.Publisher('robot_pose', KocoPose, queue_size=10)
        rospy.loginfo("Created publisher")

    def send_movej_goal_blocking(self, goal):
        return self.send_goal_to_server_blocking(self.client_j, goal)

    def send_goal_to_server_blocking(self, server, goal):
        try:
            server.send_goal(goal)
            wait_result = server.wait_for_result(rospy.Duration(2))
            if not wait_result:
                raise BufferError
            return server.get_state()
        except BufferError:
            rospy.logwarn("Waited for result for more than 2 seconds ...")
            return GoalStatus.LOST
        except Exception as e:
            rospy.logerr("Unhandled exception: {}".format(e))
            return -1

    def send_goal_to_server_nb(self, server, goal):
        try:
            server.send_goal(goal)
        except Exception as e:
            rospy.logwarn("Could not send goal. Reason:\n{}".format(e))

    def send_goal_nb(self, goal):
        self.send_goal_to_server_nb(self.client_j, goal)

    def send_j1_to_movej_nb(self):
        self.send_goal_nb(self.j_goal_1)

    def send_j1_to_movej_blocking(self):
        return self.send_movej_goal_blocking(self.j_goal_1)

    def _rosout_cb(self, msg):
        self.rosout_msg = msg.msg


class TestServerReady(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.helper = DummyASClients()

    def test_no_pub(self):
        as_state = self.helper.send_goal_to_server_blocking(self.helper.client_j, self.helper.j_goal_1)
        self.assertEquals(
            as_state, GoalStatus.ABORTED, "Expected state {0} but got {1}".format(GoalStatus.ABORTED, as_state))

    def test_basic_pub(self):
        self.helper.robot_state_pub.publish()
        as_state = self.helper.send_goal_to_server_blocking(self.helper.client_j, self.helper.j_goal_1)
        self.assertEquals(
            as_state, GoalStatus.ABORTED, "Expected state {0} but got {1}".format(GoalStatus.ABORTED, as_state))

    def test_robot_not_ready(self):
        self.helper.robot_state_pub.publish(
            RobotStatus(motion_possible=TriState(1)))
        rospy.sleep(0.2)
        as_state = self.helper.send_goal_to_server_blocking(self.helper.client_j, self.helper.j_goal_1)
        self.assertEquals(
            as_state, GoalStatus.LOST, "Expected state {0} but got {1}".format(GoalStatus.LOST, as_state))


class TestMoveActions(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.helper = DummyASClients()

    def test_movej_too_close_pose(self):
        self.helper.robot_state_pub.publish(RobotStatus(
            motion_possible=TriState(1), in_motion=TriState(0)))
        current_pose = KocoPose(tool0_pose=Transform(
            translation=self.helper.p_1.position,
            rotation=self.helper.p_1.orientation,
        ))
        self.helper.robot_pose_pub.publish(current_pose)
        rospy.sleep(0.2)
        result = self.helper.send_goal_to_server_blocking(self.helper.client_j, self.helper.j_goal_p1)
        if result == GoalStatus.SUCCEEDED:
            rosout_msg = self.helper.rosout_msg
            self.assertTrue(
                'too close' in rosout_msg, "The keyword was not present in the log message: {}".format(rosout_msg))
        else:
            self.fail("The action server did not succeed!")

    def test_movej_too_close_joints(self):
        self.helper.robot_state_pub.publish(RobotStatus(
            motion_possible=TriState(1), in_motion=TriState(0)))
        self.helper.joint_states_fkin_pub.publish(
            JointState(position=self.helper.j_1))
        rospy.sleep(0.2)
        result = self.helper.send_goal_to_server_blocking(self.helper.client_j, self.helper.j_goal_1)
        if result == GoalStatus.SUCCEEDED:
            rosout_msg = self.helper.rosout_msg
            self.assertTrue(
                'too close' in rosout_msg, "The keyword was not present in the log message: {}".format(rosout_msg))
        else:
            self.fail("The action server did not succeed!")

    def test_movej_too_close_pose_with_tool(self):
        self.helper.robot_state_pub.publish(RobotStatus(
            motion_possible=TriState(1), in_motion=TriState(0)))
        current_pose = KocoPose(
            tool0_pose=Transform(
                translation=self.helper.p_2.position,
                rotation=self.helper.p_2.orientation),
            tcp_pose=Transform(
                translation=self.helper.p_1.position,
                rotation=self.helper.p_1.orientation,
            ))
        self.helper.robot_pose_pub.publish(current_pose)
        rospy.sleep(0.2)
        self.helper.set_tool_srv('tool1')
        result = self.helper.send_goal_to_server_blocking(self.helper.client_j, self.helper.j_goal_p1)
        self.helper.set_tool_srv('tool0')
        if result == GoalStatus.SUCCEEDED:
            rosout_msg = self.helper.rosout_msg
            self.assertTrue(
                'too close' in rosout_msg, "The keyword was not present in the log message: {}".format(rosout_msg))
        else:
            self.fail("The action server did not succeed!")

    def test_movej_too_close_joints_with_tool(self):
        self.helper.robot_state_pub.publish(RobotStatus(
            motion_possible=TriState(1), in_motion=TriState(0)))
        self.helper.joint_states_fkin_pub.publish(
            JointState(position=self.helper.j_1))
        rospy.sleep(0.2)
        self.helper.set_tool_srv('tool1')
        result = self.helper.send_goal_to_server_blocking(self.helper.client_j, self.helper.j_goal_1)
        self.helper.set_tool_srv('tool0')
        if result == GoalStatus.SUCCEEDED:
            rosout_msg = self.helper.rosout_msg
            self.assertTrue(
                'too close' in rosout_msg, "The keyword was not present in the log message: {}".format(rosout_msg))
        else:
            self.fail("The action server did not succeed!")

    def test_movej_too_close_joints_sequence(self):
        self.helper.empty_sequence_buffer_srv.call()
        self.helper.joint_states_fkin_pub.publish(
            JointState(position=self.helper.j_1))
        rospy.sleep(0.2)
        result = self.helper.send_goal_to_server_blocking(self.helper.client_j_seq, self.helper.j_goal_1)
        if result == GoalStatus.SUCCEEDED:
            rosout_msg = self.helper.rosout_msg
            self.assertTrue(
                'too close' in rosout_msg, "The keyword was not present in the log message: {}".format(rosout_msg))
        else:
            self.fail("The action server did not succeed!")

    def test_movej_joints_sequence(self):
        self.helper.empty_sequence_buffer_srv.call()
        self.helper.joint_states_fkin_pub.publish(
            JointState(position=self.helper.j_1))
        rospy.loginfo("Clearing rosout ...")
        rospy.sleep(0.2)
        results = []
        for goal in [self.helper.j_goal_2, self.helper.j_goal_1]:
            result = self.helper.send_goal_to_server_blocking(self.helper.client_j_seq, goal)
            results.append(result)

        if all(result == GoalStatus.SUCCEEDED for result in results):
            rosout_msg = self.helper.rosout_msg
            self.assertFalse(
                'too close' in rosout_msg, "The keyword was present in the log message: {}".format(rosout_msg))
        else:
            self.fail("The action server did not succeed! Results array: {}".format(results))

    def test_movel_too_close_pose(self):
        self.helper.robot_state_pub.publish(RobotStatus(
            motion_possible=TriState(1), in_motion=TriState(0)))
        current_pose = KocoPose(tool0_pose=Transform(
            translation=self.helper.p_1.position,
            rotation=self.helper.p_1.orientation,
        ))
        self.helper.robot_pose_pub.publish(current_pose)
        rospy.sleep(0.2)
        self.helper.send_goal_to_server_blocking(self.helper.client_l, self.helper.l_goal_p1)
        rosout_msg = self.helper.rosout_msg
        self.assertTrue(
            'too close' in rosout_msg, "The keyword was not present in the log message: {}".format(rosout_msg))

    def test_movel_too_close_pose_with_tool(self):
        self.helper.robot_state_pub.publish(RobotStatus(
            motion_possible=TriState(1), in_motion=TriState(0)))
        current_pose = KocoPose(
            tool0_pose=Transform(
                translation=self.helper.p_2.position,
                rotation=self.helper.p_2.orientation),
            tcp_pose=Transform(
                translation=self.helper.p_1.position,
                rotation=self.helper.p_1.orientation,
            ))
        self.helper.set_tool_srv('tool1')
        self.helper.robot_pose_pub.publish(current_pose)
        self.helper.set_tool_srv('tool0')
        rospy.sleep(0.2)
        self.helper.send_goal_to_server_blocking(self.helper.client_l, self.helper.l_goal_p1)
        rosout_msg = self.helper.rosout_msg
        self.assertTrue(
            'too close' in rosout_msg, "The keyword was not present in the log message: {}".format(rosout_msg))

    def test_movel_too_close_pose_sequence(self):
        self.helper.robot_state_pub.publish(RobotStatus(
            motion_possible=TriState(1), in_motion=TriState(0)))
        current_pose = KocoPose(tool0_pose=Transform(
            translation=self.helper.p_1.position,
            rotation=self.helper.p_1.orientation,
        ))
        self.helper.robot_pose_pub.publish(current_pose)
        rospy.sleep(0.2)
        result = self.helper.send_goal_to_server_blocking(self.helper.client_l_seq, self.helper.l_goal_p1)
        if result == GoalStatus.SUCCEEDED:
            rosout_msg = self.helper.rosout_msg
            self.assertTrue(
                'too close' in rosout_msg, "The keyword was not present in the log message: {}".format(rosout_msg))
        else:
            self.fail("The action server did not succeed!")

    def test_proper_data(self):
        self.helper.robot_state_pub.publish(RobotStatus(
            motion_possible=TriState(1), in_motion=TriState(0)))
        self.helper.send_goal_to_server_nb(self.helper.client_j, self.helper.j_goal_1)
        rospy.sleep(0.2)
        self.helper.robot_state_pub.publish(RobotStatus(
            motion_possible=TriState(1), in_motion=TriState(1)))
        rospy.sleep(0.2)
        self.helper.robot_state_pub.publish(RobotStatus(
            motion_possible=TriState(1), in_motion=TriState(0)))
        rospy.sleep(0.2)
        as_state = self.helper.client_j.get_state()
        self.assertEquals(
            as_state, GoalStatus.SUCCEEDED, "Expected state {} but got {}".format(GoalStatus.SUCCEEDED, as_state))


class TestGoalParsingSuite(unittest.TestSuite):

    def __init__(self):
        super(TestGoalParsingSuite, self).__init__()
        self.addTests(map(TestMoveActions, [
            'test_proper_data',
            'test_movej_too_close_pose',
            'test_movej_too_close_joints',
            'test_movej_too_close_pose_with_tool',
            'test_movej_too_close_joints_with_tool',
            'test_movej_too_close_joints_sequence',
            'test_movej_joints_sequence',
            'test_movel_too_close_pose',
            'test_movel_too_close_pose_with_tool',
            'test_movel_too_close_pose_sequence',
        ]))


class TestActionServerReadySuite(unittest.TestSuite):
    def __init__(self):
        super(TestActionServerReadySuite, self).__init__()
        self.addTests(map(TestServerReady, [
            'test_no_pub',
            'test_basic_pub',
            'test_robot_not_ready'
        ]))


if __name__ == '__main__':
    PKG = 'koco_utils'

    try:
        test_name = sys.argv[1]
    except IndexError:
        print("No test name provided. Running all tests!")
        rosunit.unitrun(PKG, 'test_movej_actions', 'test_move_actions.TestActionServerReadySuite')
        rosunit.unitrun(PKG, 'test_movej_actions', 'test_move_actions.TestGoalParsingSuite')
    else:
        if test_name == 'test-server-ready':
            rosunit.unitrun(PKG, 'test_movej_actions', 'test_move_actions.TestActionServerReadySuite')
        elif test_name == 'test-goal-parsing':
            rosunit.unitrun(PKG, 'test_movej_actions', 'test_move_actions.TestGoalParsingSuite')
        else:
            print("The test [{}] does not exist".format(test_name))
