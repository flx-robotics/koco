import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller

from std_srvs.srv import Trigger, TriggerRequest
from koco_msgs.msg import MoveSequenceAction, MoveSequenceGoal
from actionlib_msgs.msg import GoalStatus


class MoveSequenceActionState(EventState):
    '''
    State for using move sequence action server.

    -- execute_moves    bool                Confirmation for the buffered moves to be executed.

    -- speed            float32             Scaling down the speed of all the moves.

    -- robot_ns         string              Robot action servers and services namespace (optional).

    <= succeeded                            If action completed successfuly.

    <= failed                               If action preempted, aborted or something else went wrong.

    '''

    def __init__(self, execute_moves=True, speed=1.0, robot_ns=''):
        super(MoveSequenceActionState, self).__init__(
            outcomes=['succeeded', 'failed'])

        self._goal = MoveSequenceGoal(execute_moves, speed)
        self._robot_ns = robot_ns
        self._error = False

        if robot_ns != '':
            self._robot_ns = "/" + self._robot_ns.strip('/')

        self._topic = self._robot_ns + '/robot_arm/move_sequence_action'
        self._client = ProxyActionClient({self._topic: MoveSequenceAction})

        self._play_srv = self._robot_ns + "/robot_arm/resume"
        self._pause_srv = self._robot_ns + "/robot_arm/pause"
        self._stop_srv = self._robot_ns + "/robot_arm/stop"
        self._service_client = ProxyServiceCaller({self._play_srv: Trigger, self._pause_srv: Trigger, self._stop_srv: Trigger})

    def execute(self, userdata):
        if self._error:
            return 'failed'

        if self._client.has_result(self._topic):
            status = self._client.get_state(self._topic)
            if status == GoalStatus.SUCCEEDED:
                return 'succeeded'
            else:
                Logger.logwarn('Action failed: %s' % str(status))
                return 'failed'
        
    def on_enter(self, userdata):
        self._error = False
        try:
            self._client.send_goal(self._topic, self._goal)
        except Exception as e:
            self._error = True
            Logger.logerr('Exception occurred:\n{}'.format(e))

    def on_exit(self, userdata):
        if self._client.get_state(self._topic) in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')

    def on_start(self):
        pass

    def on_stop(self):
        try:
            if self._client.get_state(self._topic) in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                self._client.cancel(self._topic)
                Logger.loginfo('Cancelled active action goal.')
                self._service_client.call(self._stop_srv, TriggerRequest())
        except Exception as e:
            Logger.logerr('Exception occurred:\n{}'.format(e))

    def on_pause(self):
        try:
            self._service_client.call(self._pause_srv, TriggerRequest())
        except Exception as e:
            Logger.logerr('Exception occurred:\n{}'.format(e))

    def on_resume(self, userdata):
        try:
            self._service_client.call(self._play_srv, TriggerRequest())
        except Exception as e:
            Logger.logerr('Exception occurred:\n{}'.format(e))
