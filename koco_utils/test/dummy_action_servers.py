#! /usr/bin/env python
import rospy

from koco_conversions import convert_matrix_to_transform

from sensor_msgs.msg import JointState

from koco_action import KocoMove

# Used to create a fake robot
from urdf_parser_py import urdf

# Import messages
from koco_msgs.msg import KocoPose
from koco_msgs.srv import TriggerString, TriggerStringResponse
from std_srvs.srv import Trigger, TriggerResponse

from koco_action.goal_checks import is_movej_within_threshold, is_movel_within_threshold
from koco_action.exceptions import GoalTooClose

from pykdl_utils.kdl_kinematics import KDLKinematics

TOOL_NAME_PARAM = 'tool_name'


class DummyActionServers(KocoMove):

    def __init__(self):

        rospy.init_node('dummy_actions', log_level=rospy.INFO)
        self._stop_seq_srv = rospy.Service(
            'stop', Trigger, self.empty_buffer_srv)

        super(DummyActionServers, self).__init__()

        self._empty_seq_srv = rospy.Service(
            'empty_sequence_buffer', Trigger, self.empty_buffer_srv)

        rospy.set_param(TOOL_NAME_PARAM, 'tool0')
        self._set_robot_tool = rospy.Service(
            'set_tool', TriggerString, self._set_tool_srv)

        # Create the action servers and register their callback functions
        self._movej_as = self.create_movej_as(callback=self.execute_j_cb)
        self._seq_movej_as = self.create_movej_as(callback=self.add_seq_movej_cb, name='add_seq_movej')
        self._movel_as = self.create_movel_as(callback=self.execute_l_cb)
        self._seq_movel_as = self.create_movel_as(callback=self.add_seq_movel_cb, name='add_seq_movel')
        self._move_seq_as = self.create_move_seq_as(callback=self.execute_sequence_cb, name='move_sequence_action')

        # Start all of the action servers
        self._movej_as.start()
        self._seq_movej_as.start()
        self._movel_as.start()
        self._seq_movel_as.start()
        self._move_seq_as.start()

        # Create an initial robot pose to avoid "no attribute" errors in the prechecker
        self.current_robot_pose = KocoPose()

        # Create the fake robot
        self.create_fake_robot()

        # Create a subscriber to the joints values of a robot
        rospy.Subscriber('joint_states', JointState, callback=self.joint_states_cb, queue_size=1)
        rospy.Subscriber('joint_states_fkin', JointState, callback=self.joint_states_fkin_cb, queue_size=1)
        rospy.Subscriber('robot_pose', KocoPose, callback=self.koco_pose_cb, queue_size=1)

        rospy.loginfo('Dummy action servers started')
        rospy.spin()

    def _set_tool_srv(self, request):
        srv_resp = TriggerStringResponse()

        tool_name = request.message
        if not tool_name:
            msg = "No tool name provided !!"
            rospy.logerr(msg)
            srv_resp.message = msg
            return srv_resp

        rospy.set_param(TOOL_NAME_PARAM, tool_name)

        srv_resp.message = "Tool successfully set on the parameter server!"
        srv_resp.success = True

        return srv_resp

    def custom_prechecks(self):
        movej_goal = self._movej_as.current_goal.get_goal()
        movel_goal = self._movel_as.current_goal.get_goal()
        if movej_goal is not None:
            if is_movej_within_threshold(self.current_robot_pose, movej_goal, 0.1, self.tool_0_kinematics):
                raise GoalTooClose()
        if movel_goal is not None:
            if is_movel_within_threshold(self.current_robot_pose, movel_goal, 0.1):
                raise GoalTooClose()

    def joint_states_cb(self, msg):
        self.joint_states = msg

    def joint_states_fkin_cb(self, msg):
        current_tool = rospy.get_param(TOOL_NAME_PARAM)
        # print("Current tool name: {}".format(current_tool))
        tool0_pose_mat = self._robot_kinematics.forward(msg.position, base_link='link_0', end_link='tool0')
        current_tool_pose_mat = self._robot_kinematics.forward(msg.position, base_link='link_0', end_link=current_tool)
        self.current_robot_pose.tool0_pose = convert_matrix_to_transform(tool0_pose_mat)
        self.current_robot_pose.tcp_pose = convert_matrix_to_transform(current_tool_pose_mat)

    def koco_pose_cb(self, msg):
        self.current_robot_pose = msg

    def execute_j_cb(self, goal):
        try:
            if super(DummyActionServers, self).execute_cb(goal):
                self._movej_as.set_succeeded()
            else:
                self._movej_as.set_aborted()
        except Exception as e:
            rospy.logerr("Failed to process goal. Reason:\n{}".format(e))
            self._movej_as.set_aborted()

    def execute_l_cb(self, goal):
        try:
            if super(DummyActionServers, self).execute_cb(goal):
                self._movel_as.set_succeeded()
            else:
                self._movel_as.set_aborted()
        except Exception as e:
            rospy.logerr("Failed to process goal. Reason:\n{}".format(e))
            self._movel_as.set_aborted()

    def add_seq_movej_cb(self, goal):
        try:
            if not self._move_seq:
                rospy.loginfo("Checking first point in the sequence ...")
                if is_movej_within_threshold(self.current_robot_pose, goal, 0.1, self.tool_0_kinematics):
                    rospy.logwarn("First point in sequence too close to starting position. Skipping")
                    raise GoalTooClose()
            self._move_seq.append(goal)
            self._seq_movej_as.set_succeeded()
        except GoalTooClose:
            self._move_seq.append(goal)
            self._seq_movej_as.set_succeeded()
        except Exception as e:
            rospy.logerr("Failed to add the goal to the sequence: {}".format(e))
            self._empty_seq_buffers()
            self._seq_movej_as.set_aborted()

    def add_seq_movel_cb(self, goal):
        try:
            if not self._move_seq:
                rospy.loginfo("Checking first point in the sequence ...")
                if is_movel_within_threshold(self.current_robot_pose, goal, 0.1):
                    rospy.logwarn("First point in sequence too close to starting position. Skipping")
                    raise GoalTooClose()
            self._move_seq.append(goal)
            self._seq_movel_as.set_succeeded()
        except GoalTooClose:
            self._move_seq.append(goal)
            self._seq_movel_as.set_succeeded()
        except Exception as e:
            rospy.logerr("Failed to add the goal to the sequence: {}".format(e))
            self._empty_seq_buffers()
            self._seq_movel_as.set_aborted()

    def execute_sequence_cb(self, goal):
        self._empty_seq_buffers(self)
        self._move_seq_as.set_succeeded()

    def empty_buffer_srv(self, req):
        self._empty_seq_buffers()
        return TriggerResponse()

    def create_fake_robot(self):
        self._robot = urdf.Robot(name='fakie', version='1.0')
        links = [
            urdf.Link(name='link_{}'.format(i)) for i in range(7)]
        joints = [
            urdf.Joint(
                name='joint_{}'.format(i),
                joint_type='revolute',
                parent='link_{}'.format(i),
                child='link_{}'.format(i + 1),
                origin=urdf.Pose(xyz=[0, 0, 1]),
                axis=[0, 0, 1]) for i in range(6)]

        for joint in joints:
            self._robot.add_joint(joint)
        for link in links:
            self._robot.add_link(link)

        self._robot.add_link(urdf.Link(name='tool0'))
        self._robot.add_joint(urdf.Joint(
            name='tool0_joint',
            joint_type='fixed',
            parent='link_6',
            child='tool0',
            origin=urdf.Pose(xyz=[0, 0, 0]),
            axis=[0, 0, 1]))
        self._robot.add_link(urdf.Link(name='tool1'))
        self._robot.add_joint(urdf.Joint(
            name='tool1_joint',
            joint_type='fixed',
            parent='tool0',
            child='tool1',
            origin=urdf.Pose(xyz=[0.1, 0.2, 0.3]),
            axis=[0, 0, 1]))

        self._robot_kinematics = KDLKinematics(self._robot, 'link_0', 'tool1')
        self.tool_0_kinematics = KDLKinematics(self._robot, 'link_0', 'tool0')


if __name__ == '__main__':
    DummyActionServers()
