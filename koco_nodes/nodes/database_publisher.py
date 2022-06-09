#!/usr/bin/env python
import rospy
from mongodb_store.message_store import MessageStoreProxy

import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger, TriggerResponse

# KoCo messages
from koco_msgs.msg import KocoPose, KocoTool

TOOL0_FRAME = rospy.get_param("tool0", "tool0")


class Publisher(object):
    msg_store = None
    static_tf_pub = None

    def __init__(self):
        rospy.loginfo("Constructing object")
        self.msg_store = MessageStoreProxy()

        self.static_tf_pub = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()

        self.reload_and_publish()
        self.reload_tool_params()

        rospy.Service('reload_database', Trigger, self._cb)

        rospy.spin()

    def reload_and_publish(self):
        try:
            read = self.msg_store.query(KocoPose._type)
            database_content = {}
            for item in read:
                data = TransformStamped()
                data.header = item[0].header
                data.child_frame_id = item[0].child_frame_id
                data.transform = item[0].tcp_pose
                database_content[data.child_frame_id] = data

            read = self.msg_store.query(KocoTool._type)
            db_tool_content = {}
            for item in read:
                data = TransformStamped()
                data.transform = item[0].transform
                data.header.frame_id = TOOL0_FRAME
                data.header.stamp = rospy.Time.now()
                data.child_frame_id = 'db/' + item[0].tool_name
                db_tool_content[data.child_frame_id] = data

            rospy.loginfo('Read {0} unique KocoPose messages from the DB and make them TransformStamped for publishing.'.format(len(database_content)))
            rospy.loginfo('Read {0} unique KocoTool messages from the DB and make them TransformStamped for publishing.'.format(len(db_tool_content)))

            self.static_tf_pub.sendTransform(list(database_content.values()) + list(db_tool_content.values()))

            return True
        except Exception as e:
            rospy.logerr("Failed to reload the database. Reason:\n{}".format(e))
            return False

    def reload_tool_params(self):
        read = self.msg_store.query(KocoTool._type)
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        for item in read:
            transl = item[0].transform.translation
            rot = item[0].transform.rotation
            tool0_offset = [transl.x, transl.y, transl.z, rot.x, rot.y, rot.z, rot.w]
            rospy.set_param("/tools/{}/tool0_offset".format(item[0].tool_name), tool0_offset)
            rospy.set_param("/tools/{}/weight".format(item[0].tool_name), item[0].weight)
            rospy.set_param("/tools/{}/cog".format(item[0].tool_name), [item[0].cog.x, item[0].cog.y, item[0].cog.z])
            rospy.set_param("/tools/{}/path_to_model_file".format(item[0].tool_name), item[0].path_to_model_file)

        return True

    def _cb(self, req):
        rospy.loginfo("Reloading database")
        res = TriggerResponse()
        res.success = self.reload_and_publish() and self.reload_tool_params()
        return res


if __name__ == '__main__':
    rospy.init_node('database_publisher', log_level=rospy.ERROR)
    Publisher()