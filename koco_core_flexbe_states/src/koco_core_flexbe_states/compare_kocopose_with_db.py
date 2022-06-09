import rospy

import numpy as np

from flexbe_core import EventState, Logger

from koco_msgs.msg import KocoPose
from koco_proxies.proxies import KocoMessageStoreProxy
from koco_conversions.pose_conversions import convert_transform_to_pose, convert_pose_to_array_7
from koco_geometry.diffs import quat_diff

from tf.transformations import euler_from_quaternion


class CompareKocoPoseWithDB(EventState):
    '''
    State for comparing a KocoPose in userdata with the ones stored in DB.

    -- x_t      float32[]                   The allowed translational tolerance (+/-) in the X axis.

    -- y_t      float32[]                   The allowed translational tolerance (+/-) in the Y axis.

    -- z_t      float32[]                   The allowed translational tolerance (+/-) in the Z axis.

    -- x_r      float32[]                   The allowed rotational tolerance (+/-) in the X axis.

    -- y_r      float32[]                   The allowed rotational tolerance (+/-) in the Y axis.

    -- z_r      float32[]                   The allowed rotational tolerance (+/-) in the Z axis.

    ># comparing_pose   koco_msgs/KocoPose  Input key containing the KocoPose that we will be comparing.

    <= within_tolerance                     If the pose is within tolerance

    <= outside_tolerance                    If the pose is NOT within tolerance

    <= error                                If an error has occured

    '''

    def __init__(self, x_t=[], y_t=[], z_t=[], x_r=[], y_r=[], z_r=[]):
        super(CompareKocoPoseWithDB, self).__init__(
            outcomes=['within_tolerance', 'outside_tolerance', 'error'],
            input_keys=['comparing_pose'])

        self._x_t = x_t
        self._y_t = y_t
        self._z_t = z_t
        self._x_r = x_r
        self._y_r = y_r
        self._z_r = z_r
        self._allowed_errors = [x_t, y_t, z_t, x_r, y_r, z_r]

        self._db_proxy = KocoMessageStoreProxy().get_proxy()

    def execute(self, userdata):
        if self._error:
            return 'error'
        if self._within_tolerance:
            return 'within_tolerance'
        else:
            return 'outside_tolerance'

    def on_enter(self, userdata):
        self._error = False
        try:
            comparing_pose = userdata.comparing_pose
            db_pose = self._db_proxy.query(KocoPose._type, {"child_frame_id": comparing_pose.child_frame_id})[-1][0]

            db_p = convert_pose_to_array_7(convert_transform_to_pose(db_pose.tcp_pose))
            comp_p = convert_pose_to_array_7(convert_transform_to_pose(comparing_pose.tcp_pose))

            pos_diff = (np.array(db_p[:3]) - np.array(comp_p[:3])).tolist()
            q_diff = quat_diff(db_p[3:], comp_p[3:])

            diff_pose = pos_diff + list(euler_from_quaternion(q_diff))

            within_tolerance = []
            for idx, allowed_error in enumerate(self._allowed_errors):
                if allowed_error:
                    within_tolerance.append((diff_pose[idx] > allowed_error[0]) and (diff_pose[idx] < allowed_error[1]))

            self._within_tolerance = all(within_tolerance)
            if not self._within_tolerance:
                rospy.logwarn("The new pose is not within tolerance:\n{}".format(diff_pose))
                rospy.logwarn("Tolerance array:\n{}".format(within_tolerance))

        except Exception as e:
            self._error = True
            Logger.logerr('Exception occurred:\n{}'.format(e))

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass

    def on_pause(self):
        pass

    def on_resume(self, userdata):
        pass