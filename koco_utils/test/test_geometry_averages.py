#!/usr/bin/env python
import unittest
import numpy as np
import rospy
from tf import transformations as tfs

from koco_geometry.averages import average_quaternion


class TestGeometryAverages(unittest.TestCase):

    def setUp(self):
        pass

    def test_average_quaternion(self):
        quaternions = [
            tfs.quaternion_from_euler(0.5, 0, 0, 'sxyz'),
            tfs.quaternion_from_euler(-0.5, 0, 0, 'sxyz'),
            tfs.quaternion_from_euler(0, 0.8, 0, 'sxyz'),
            tfs.quaternion_from_euler(0, -0.8, 0, 'sxyz'),
            tfs.quaternion_from_euler(0, 0, 1.2, 'sxyz'),
            tfs.quaternion_from_euler(0, 0, -1.2, 'sxyz')
        ]

        avg_quaternion = average_quaternion(quaternions)
        avg_euler = tfs.euler_from_quaternion(avg_quaternion, 'sxyz')
        expected_avg_euler = np.array([0, 0, 0])
        np.testing.assert_allclose(avg_euler, expected_avg_euler, rtol=1e-10, atol=1e-13)


if __name__ == '__main__':
    rospy.init_node("test_geometry_averages")
    import rostest
    rostest.rosrun('koco_utils', 'test_geometry_averages', TestGeometryAverages)
