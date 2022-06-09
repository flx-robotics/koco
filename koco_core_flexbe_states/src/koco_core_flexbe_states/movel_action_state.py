import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller

from std_srvs.srv import Trigger, TriggerRequest
from koco_msgs.msg import MoveLAction, MoveLGoal, KocoPose
from actionlib_msgs.msg import GoalStatus
from koco_conversions.pose_conversions import offset_to_pose, convert_transform_to_pose
from koco_proxies.proxies import KocoTransformListenerProxy, KocoMessageStoreProxy

BASE_TF_LINK = "base_link"


class MovelActionState(EventState):
    '''
    State for using MoveL action server.

    -- target_array     float32[]           Cartesian space target written as an array.

    -- speed            float32             Robot speed for requested move.

    -- offset           float32[]           Offset from the target pose (optional).

    -- config_flags     int8[]              Robot configuration flags for the move.

    -- robot_ns         string              Robot action servers and services namespace (optional).

    ># target_pose      geometry_msgs/Pose  Input key containing the desired pose.

    ># tf_pose_name     string              Input key containing the pose name from the tf.

    ># db_pose_name     string              Input key containing the pose name from the database.

    <= succeeded                            If action completed successfuly.

    <= failed                               If action preempted, aborted or something else went wrong.

    '''

    def __init__(self, target_array=[], speed=0.0, offset=[], config_flags=[], robot_ns=''):
        super(MovelActionState, self).__init__(
            outcomes=['succeeded', 'failed'],
            input_keys=['target_pose', 'tf_pose_name', 'db_pose_name'])

        self._target_array = target_array
        self._speed = speed
        self._offset = offset
        self._blend_radius = 0.0
        self._config_flags = config_flags
        self._robot_ns = robot_ns
        self._error = False

        if robot_ns != '':
            self._robot_ns = "/" + self._robot_ns.strip('/')

        self._tf_listener = KocoTransformListenerProxy().get_buffer()
        self._db_proxy = KocoMessageStoreProxy().get_proxy()

        self._topic = self._robot_ns + '/robot_arm/movel_action'
        self._client = ProxyActionClient({self._topic: MoveLAction})

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
            conf_flags = self._config_flags

            check_inputs = [len(userdata.tf_pose_name), len(userdata.db_pose_name), userdata.target_pose is not None, len(self._target_array)]
            check_inputs = map(bool, check_inputs)
            if check_inputs.count(True) != 1:
                Logger.logerr("Exactly one of the target_array, tf_pose_name, db_pose_name and target_pose should be passed in and not 0 or more than 1!")
                self._error = True
                return
            elif len(self._target_array) and len(self._offset):
                Logger.logerr("Offset cannot be applied to the target array.")
                self._error = True
                return

            trg_pose = None
            if len(userdata.tf_pose_name):
                t_stamped = self._tf_listener.lookup_transform(BASE_TF_LINK, userdata.tf_pose_name, rospy.Time(), rospy.Duration(1))
                trg_pose = convert_transform_to_pose(t_stamped.transform)
                if not self._config_flags:
                    db_result = self._db_proxy.query(KocoPose._type, {"child_frame_id": userdata.tf_pose_name})
                    if db_result:
                        conf_flags = db_result[-1][0].config_flags
            elif len(userdata.db_pose_name):
                db_result = self._db_proxy.query(KocoPose._type, {"child_frame_id": userdata.db_pose_name})
                trg_pose = convert_transform_to_pose(db_result[-1][0].tcp_pose)
                if not self._config_flags:
                    conf_flags = db_result[-1][0].config_flags
            elif userdata.target_pose is not None:
                trg_pose = userdata.target_pose
            
            if len(self._offset) and trg_pose is not None:
                trg_pose = offset_to_pose(trg_pose, self._offset).pose

            goal = MoveLGoal()

            if len(self._target_array):
                goal.target_array = self._target_array
            else:
                goal.target_pose = trg_pose

            goal.speed = self._speed
            goal.blend_radius = self._blend_radius
            goal.config_flags = conf_flags

            self._client.send_goal(self._topic, goal)
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
