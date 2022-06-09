import rospy

from std_srvs.srv import Trigger, TriggerRequest
from koco_msgs.msg import MoveJAction, MoveJGoal, KocoPose
from actionlib_msgs.msg import GoalStatus
from koco_proxies.proxies import KocoMessageStoreProxy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller


class MoveJointsFromDb(EventState):
    '''
    Moves the robot to the joints from the database.

    -- db_name          string                  Name of the database input where the joints are stored as a KocoPose object.

    -- speed            float32                 Speed of the move.

    -- config_flags     int8[]                  Robot configuration flags for the move.

    -- robot_ns         string                  Robot action servers and services namespace (optional).

    <= succeeded                                If move successfully done.

    <= failed                                   If something goes wrong.

    '''

    def __init__(self, db_name='', speed=0.0, config_flags=[], robot_ns=''):
        super(MoveJointsFromDb, self).__init__(
            outcomes=['succeeded', 'failed'])

        self._db_name = db_name
        self._speed = speed
        self._config_flags = config_flags
        self._robot_ns = robot_ns
        self._error = False

        if robot_ns != '':
            self._robot_ns = "/" + self._robot_ns.strip('/')

        try:
            self._db_proxy = KocoMessageStoreProxy().get_proxy()
            db_result = self._db_proxy.query(KocoPose._type, {"child_frame_id": self._db_name})
            db_joints = db_result[-1][0].joints
            self._goal = MoveJGoal(target_joint=db_joints, speed=self._speed, config_flags=self._config_flags)
        except Exception as e:
            Logger.logerr("Move joints from db failed for the db name: {}".format(db_name))
            raise rospy.ROSException('Cannot get the db joints!')

        self._topic = self._robot_ns + '/robot_arm/movej_action'
        self._client = ProxyActionClient({self._topic: MoveJAction})

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
        pass

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
