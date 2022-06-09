#!/usr/bin/env python
import os
import unittest

import rospkg
import rospy
from koco_toolbox.ros_path import get_rospkg_name_from_path, resolve_ros_path

PREFIX = 'package://'


class TestRosPathResolver(unittest.TestCase):

    def try_resolve_path(self, prefix, suffix):
        """
        Takes prefix and suffix and add them to predefined ROS package name. 
        Then it tests if path is resolved as expected.
        """
        ros_path = os.path.join(prefix, self.pkg_name, suffix)
        abs_path = resolve_ros_path(ros_path)
        self.assertEquals(abs_path, os.path.join(self.abs_pkg_path, suffix))

    def try_resolve_pkg_name(self, prefix, suffix):
        """
        Takes prefix and suffix and add them to predefined ROS package name. 
        Then it tests if package name is correctly resolved from path.
        """
        ros_path = os.path.join(prefix, self.pkg_name, suffix)
        pkg_name = get_rospkg_name_from_path(ros_path)
        self.assertEquals(pkg_name, self.pkg_name)

    @classmethod
    def setUpClass(cls):
        # It could be any module, but this exists for sure.
        cls.pkg_name = 'koco_utils'
        cls.abs_pkg_path = rospkg.RosPack().get_path(cls.pkg_name)

    def test_basic(self):
        self.try_resolve_path(PREFIX, '')

    def test_bare_bones(self):
        self.try_resolve_path('', '')

    def test_with_subdir(self):
        self.try_resolve_path(PREFIX, 'config/some_non_existing_file')

    def test_bare_bones_with_subdir(self):
        self.try_resolve_path('', 'config/some_non_existing_file')

    def test_absolute_path(self):
        abs_path = '/some/absolute/path/to/pkg'
        resolved_path = resolve_ros_path(abs_path)
        self.assertEquals(abs_path, resolved_path)

    def test_getting_pkg_name_bare_bones(self):
        self.try_resolve_pkg_name('', '')

    def test_getting_pkg_name_basic(self):
        self.try_resolve_pkg_name(PREFIX, '')

    def test_getting_pkg_name_with_subdir(self):
        self.try_resolve_pkg_name('', 'config/some_non_existing_file')

    def test_getting_pkg_name_full(self):
        self.try_resolve_pkg_name(PREFIX, 'config/some_non_existing_file')

    def test_getting_pkg_name_fail(self):
        with self.assertRaises(Exception):
            self.try_resolve_pkg_name('/some/path', 'config/some_non_existing_file')


if __name__ == '__main__':
    rospy.init_node("test_string_macro")
    import rostest
    rostest.rosrun('koco_toolbox', 'test_ros_path_resolver', TestRosPathResolver)
