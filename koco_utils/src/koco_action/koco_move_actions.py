import rospy
import actionlib

# Import standard ROS msgs
from std_msgs.msg import Bool
from industrial_msgs.msg import RobotStatus

# Import action definitions
from koco_msgs.msg import MoveJAction, MoveLAction, MoveSequenceAction
from std_srvs.srv import Trigger, TriggerResponse

# Import KoCo stuff
from koco_action.exceptions import (
    RobotNotReady,
    MovementNotStarted,
    RobotInError,
    ActionPreempted,
    GoalTooClose)


class KocoMove(object):

    _move_seq = []

    # Initialize this attribute to False in case the topic does not exist
    is_paused = False

    def __init__(self, rate=100):
        self._rstatus_sub = rospy.Subscriber(
            "robot_status", RobotStatus, self._robot_status_cb)

        self._rpaused_sub = rospy.Subscriber(
            "is_paused", Bool, self._paused_cb)

        self.rate = rospy.Rate(rate)

        try:
            rospy.wait_for_service('stop', 30)
            self._stop_robot_srv = rospy.ServiceProxy(
                'stop', Trigger)
        except:
            raise Exception("Robot stop service not defined properly")

    def _robot_status_cb(self, topic_data):
        self.robot_status = topic_data
        self.motion_possible = self.robot_status.motion_possible.val == 1
        self.robot_idle = self.robot_status.in_motion.val == 0
        self.in_error = self.robot_status.in_error.val == 1
        self.robot_in_motion = not self.robot_idle

    def _paused_cb(self, topic_data):
        self.is_paused = topic_data.data

    def robot_ready_to_move(self):
        """The function checks if the robot is ready to move."""
        rospy.logdebug(self.motion_possible)
        return self.motion_possible

    def send_move_to_robot(self, data_to_robot):
        """Function that sends the data to the robot."""
        rospy.logdebug("Data sent to robot, waiting that the motion starts")

        t_start = rospy.Time.now()
        while (rospy.Time.now() - t_start).to_sec() < 4:
            rospy.logdebug_once("Waiting for the motion to start ...")
            if self.robot_in_motion:
                rospy.logdebug("Motion started after {}s!".format(
                    (rospy.Time.now() - t_start).to_sec()))
                return True
            self.rate.sleep()
        raise MovementNotStarted

    def execution_monitoring(self):
        """The function monitors the state of the robot and sends feedback to the action client."""
        t_start = rospy.Time.now()
        while (self.robot_in_motion or self.is_paused) and not self.in_error:
            rospy.logdebug_once("Waiting for the motion to be concluded ...")
            if  \
                self._movej_as.is_preempt_requested() or \
                self._movel_as.is_preempt_requested() or \
                    self._move_seq_as.is_preempt_requested():
                        self.preempt_as()
            self.rate.sleep()
        rospy.logdebug_once("Motion concluded after {}s".format(
            (rospy.Time.now() - t_start).to_sec()))

    def preempt_as(self):
        """Stops robot execution and raises an exception"""
        self._stop_robot_srv()
        raise ActionPreempted(
            "Action was preempted!")

    def check_motion_success(self):
        """The function checks that the robot is not in error state. If it is, throws an exception."""
        if self.in_error:
            raise RobotInError(
                "The robot is in error state after it finished moving.")

    def custom_prechecks(self):
        """This function gets called before any data is sent to the robot. It
        is intended to be overridden when implementing the action servers.
        """
        pass

    def custom_postchecks(self):
        """This function gets called after all the other regular checks have been
        performed. It is intended to be overridden when implementing the action servers.
        """
        pass

    def execute_cb(self, robot_command):
        """Method that executs a callback"""
        rospy.logdebug("Executing the KOCO action callback")
        t_start = rospy.Time.now()
        try:
            rospy.logdebug('robot_command = {}'.format(robot_command))

            if not(self.motion_possible):
                raise RobotNotReady

            self.custom_prechecks()

            self.send_move_to_robot(robot_command)

            self.execution_monitoring()

            self.check_motion_success()

            self.custom_postchecks()

            return True
        except RobotNotReady:
            rospy.logwarn("The robot is not ready to execute the motion ...")
        except MovementNotStarted:
            rospy.logwarn("Command send to robot but motion did not start.")
        except RobotInError:
            rospy.logwarn("The robot finished moving with an error.")
        except ActionPreempted:
            rospy.logwarn("Action was preempted.")
        except GoalTooClose:
            rospy.logwarn("The robot is too close to the provided goal. Action still successful.")
            return True
        except Exception as e:
            rospy.logerr("No idea what happened ...\nException: {}".format(e))

        return False
        rospy.logdebug_once("execute_cb() in {}s".format(
            (rospy.Time.now() - t_start).to_sec()))

    def _empty_seq_buffers(self):
        self._move_seq = []

    def empty_buffer_srv(self, req):
        self._empty_seq_buffers()
        return TriggerResponse(message="Sequence buffers emptied!", success=True)

    @staticmethod
    def create_movej_as(callback, name='movej_action', **kwargs):
        return actionlib.SimpleActionServer(
            name,
            MoveJAction,
            execute_cb=callback,
            auto_start=False)

    @staticmethod
    def create_movel_as(callback, name='movel_action', **kwargs):
        return actionlib.SimpleActionServer(
            name,
            MoveLAction,
            execute_cb=callback,
            auto_start=False)

    @staticmethod
    def create_move_seq_as(callback, name='move_sequence_action', **kwargs):
        return actionlib.SimpleActionServer(
            name,
            MoveSequenceAction,
            execute_cb=callback,
            auto_start=False)
