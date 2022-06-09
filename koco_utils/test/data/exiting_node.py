#!/usr/bin/env python3

import traceback
import rospy


class ExitingNode(object):

    def __init__(self):
        rospy.sleep(3)
        raise rospy.ROSException("My job is to exit.")


if __name__ == '__main__':
    rospy.init_node('exiting_node')
    try:
        ExitingNode()
    except Exception as e:
        rospy.logerr("Exception occurred in the initialization of the node. Exception:\n{}".format(e))
        rospy.logerr("Traceback:\n{}".format(traceback.format_exc()))
    finally:
        rospy.loginfo("Exiting ...")