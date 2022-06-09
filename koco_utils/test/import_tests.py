#! /usr/bin/env python
import rospy
import unittest

PKG = 'koco_utils'


class TestImports(unittest.TestCase):

    def test_conversions(self):
        try:
            import koco_conversions
        except:
            self.fail("Cannot import convertions __init__.")

    def test_pose_conversions(self):
        try:
            import koco_conversions.pose_conversions
        except:
            self.fail("Cannot import pose_conversions script.")

    def test_macros(self):
        try:
            import koco_conversions.macros
        except:
            self.fail("Cannot import macros script.")

    def test_geometry(self):
        try:
            import koco_geometry
        except:
            self.fail("Cannot import geometry __init__.")

    def test_geometry_diffs(self):
        try:
            import koco_geometry.diffs
        except:
            self.fail("Cannot import diffs script.")

    def test_proxies(self):
        try:
            import koco_proxies.proxies
        except:
            self.fail("Cannot import proxies.")

    def test_actions(self):
        try:
            import koco_action
        except:
            self.fail("Cannot koco actions.")

    def test_actions_utils(self):
        try:
            import koco_action.utils
        except:
            self.fail("Cannot koco actions' utilities.")

    def test_actions_exceptions(self):
        try:
            import koco_action.exceptions
        except:
            self.fail("Cannot import koco actions' exceptions.")

    def test_toolbox(self):
        try:
            import koco_toolbox
        except:
            self.fail("Cannot import koco_toolbox __init__.")

    def test_toolbox_ros_path(self):
        try:
            import koco_toolbox.ros_path
        except:
            self.fail("Cannot import ros_path script.")


if __name__ == '__main__':
    rospy.init_node("imports_test")
    import rostest
    rostest.rosrun(PKG, 'test_imports', TestImports)
