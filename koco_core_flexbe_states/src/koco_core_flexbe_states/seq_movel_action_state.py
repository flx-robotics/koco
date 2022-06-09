import rospy

from flexbe_core.proxy import ProxyActionClient

from movel_action_state import MovelActionState

from koco_msgs.msg import MoveLAction


class SequenceMovelActionState(MovelActionState):
    '''
    State for using the sequence movel action server.

    -- target_array     float32[]           Cartesian space target written as an array.

    -- speed            float32             Robot speed for requested move.

    -- offset           float32[]           Offset from the target pose (optional).

    -- blend_radius     float64             Blend radius for the movel in sequence.

    -- config_flags     int8[]              Robot configuration flags for the move.

    -- robot_ns         string              Robot action servers and services namespace (optional).

    ># target_pose      geometry_msgs/Pose  Input key containing the desired pose.

    ># tf_pose_name     string              Input key containing the pose name from the tf.

    ># db_pose_name     string              Input key containing the pose name from the database.

    <= succeeded                            If action completed successfuly.

    <= failed                               If action preempted, aborted or something else went wrong.

    '''

    def __init__(self, target_array=[], speed=0.0, offset=[], blend_radius=0.0, config_flags=[], robot_ns=''):
        super(SequenceMovelActionState, self).__init__(target_array, speed, offset, config_flags, robot_ns)

        self._blend_radius = blend_radius

        self._topic = self._robot_ns + '/robot_arm/add_seq_movel'
        self._client = ProxyActionClient({self._topic: MoveLAction})
