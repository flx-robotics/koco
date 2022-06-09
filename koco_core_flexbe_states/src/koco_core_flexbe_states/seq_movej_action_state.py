import rospy

from flexbe_core.proxy import ProxyActionClient

from movej_action_state import MovejActionState

from koco_msgs.msg import MoveJAction


class SequenceMovejActionState(MovejActionState):
    '''
    State for using the sequence movej action server.

    -- joints           float32[]           Robot joints (6 value array) for requested move.

    -- speed            float32             Robot speed for requested move (optional).

    -- offset           float32[]           Offset from the target pose (optional).

    -- blend_radius     float64             Blend radius for the movej in sequence.

    -- config_flags     int8[]              Robot configuration flags for the move.

    -- robot_ns         string              Robot action servers and services namespace (optional).

    ># target_pose      geometry_msgs/Pose  Input key containing the desired pose.

    ># tf_pose_name     string              Input key containing the pose name from the tf.

    ># db_pose_name     string              Input key containing the pose name from the database.

    <= succeeded                            If action completed successfuly.

    <= failed                               If action preempted, aborted or something else went wrong.

    '''

    def __init__(self, joints=[], speed=0.0, offset=[], blend_radius=0.0, config_flags=[], robot_ns=''):
        super(SequenceMovejActionState, self).__init__(joints, speed, offset, config_flags, robot_ns)

        self._blend_radius = blend_radius

        self._topic = self._robot_ns + '/robot_arm/add_seq_movej'
        self._client = ProxyActionClient({self._topic: MoveJAction})
