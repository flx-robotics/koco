import rospy

from flexbe_core.proxy import ProxyActionClient
from move_joints_from_db import MoveJointsFromDb
from koco_msgs.msg import MoveJAction


class SeqMoveJointsFromDb(MoveJointsFromDb):
    '''
    Moves the robot to the joints from the database with the use of sequence action server.

    -- db_name          string                  Name of the database input where the joints are stored as a KocoPose object.

    -- speed            float32                 Speed of the move.

    -- blend_radius     float64                 Blend radius for the movej in sequence.

    -- config_flags     int8[]                  Robot configuration flags for the move.

    -- robot_ns         string                  Robot action servers and services namespace (optional).

    <= succeeded                                If move successfully done.

    <= failed                                   If something goes wrong.

    '''

    def __init__(self, db_name='', speed=0.0, blend_radius=0.0, config_flags=[], robot_ns=''):
        super(SeqMoveJointsFromDb, self).__init__(db_name, speed, config_flags, robot_ns)

        self._goal.blend_radius = blend_radius

        self._topic = self._robot_ns + '/robot_arm/add_seq_movej'
        self._client = ProxyActionClient({self._topic: MoveJAction})
