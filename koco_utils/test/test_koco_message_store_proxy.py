#!/usr/bin/env python

import unittest
import pymongo

import rospy
from koco_proxies.proxies import KocoMessageStoreProxy, AgeLimitExceeded
from std_msgs.msg import String


class TestKocoMessageStoreProxy(unittest.TestCase):

    def purge_database(self):
        """Deletes the 'message_store' collection in the database."""
        mongo_client = pymongo.MongoClient("mongodb://localhost:62345/")
        db = mongo_client.message_store
        collection = db.message_store
        collection.drop()

    @classmethod
    def setUpClass(cls):
        cls.msg1 = String(data="msg1")
        cls.msg2 = String(data="msg2")
        cls.msg3 = String(data="msg3")
        cls.msg4 = String(data="msg4")
        cls.type = String._type

        cls.proxy = KocoMessageStoreProxy()

    def setUp(self):
        self.purge_database()

    def test_singleton(self):
        proxy1 = KocoMessageStoreProxy().get_proxy()
        proxy2 = KocoMessageStoreProxy().get_proxy()
        self.assertIs(proxy1, proxy2)

    def test_msg_comparison(self):
        self.assertNotEquals(self.msg1, self.msg2)

    def test_insert_and_retrieve_msg(self):
        self.proxy.insert_msg('msg', self.msg1)
        db_msg1 = self.proxy.query_latest_msg('msg', self.type)
        self.assertEqual(db_msg1, self.msg1)

    def test_retrieve_not_existing(self):
        with self.assertRaises(KeyError):
            self.proxy.query_latest_msg('msg', self.type)

    def test_retrieve_latest_msg(self):
        self.proxy.insert_msg('msg', self.msg1)
        self.proxy.insert_msg('msg', self.msg2)
        db_msg2 = self.proxy.query_latest_msg('msg', self.type)
        self.assertEqual(db_msg2, self.msg2)

    def test_update_latest_msg(self):
        self.proxy.insert_msg('msg', self.msg1)
        self.proxy.insert_msg('msg', self.msg2)
        self.proxy.update_latest_msg('msg', self.msg3)
        msgs = self.proxy.get_proxy().query_named('msg', self.type, single=False)
        self.assertEqual(len(msgs), 2)  # no additional entry created

        db_msg3 = self.proxy.query_latest_msg('msg', self.type)
        self.assertEqual(db_msg3, self.msg3)

    def test_update_not_existing(self):
        self.proxy.insert_msg('msg1', self.msg1)
        with self.assertRaises(KeyError):
            self.proxy.update_latest_msg('msg2', self.msg2)

    def test_update_not_existing_upsert(self):
        self.proxy.insert_msg('msg1', self.msg1)
        self.proxy.update_latest_msg('msg2', self.msg2, upsert=True)
        self.proxy.update_latest_msg('msg2', self.msg3, upsert=True)
        db_msgs = self.proxy.get_proxy().query_named('msg2', self.type, single=False)
        self.assertEqual(len(db_msgs), 1)

    def test_retrieve_latest_msg_advance(self):
        """Test behavior if non last inserted message was last updated."""
        id1 = self.proxy.insert_msg('msg', self.msg1)
        id2 = self.proxy.insert_msg('msg', self.msg2)
        self.proxy.update_latest_msg('msg', self.msg3)
        self.proxy.get_proxy().update_id(id1, self.msg4)
        db_msg4 = self.proxy.get_proxy().query_named('msg', self.type, single=False)[0][0]
        self.assertEqual(db_msg4, self.msg4)

        db_msg3 = self.proxy.query_latest_msg('msg', self.type)
        self.assertEqual(db_msg3, self.msg3)

    def test_edit_latest_msg(self):
        self.proxy.insert_msg('msg', self.msg1)
        self.proxy.insert_msg('msg', self.msg2)
        with self.proxy.edit_latest_msg('msg', self.type) as msg:
            self.assertEqual(msg, self.msg2)
            msg.data = self.msg3.data
        db_msg3 = self.proxy.query_latest_msg('msg', self.type)
        self.assertEqual(db_msg3, self.msg3)

    def test_edit_not_existing(self):
        with self.assertRaises(KeyError):
            with self.proxy.edit_latest_msg('msg', self.type) as msg:
                pass

    def test_query_latest_msgs_case1(self):
        self.proxy.insert_msg('msg', self.msg1)
        self.proxy.insert_msg('msg', self.msg2)
        self.proxy.insert_msg('msg_new', self.msg3)
        db_result = self.proxy.query_latest_msgs('^msg$', self.type)

        self.assertEqual(db_result, {'msg': self.msg2})

    def test_query_latest_msgs_case2(self):
        self.proxy.insert_msg('msg', self.msg1)
        self.proxy.insert_msg('msg', self.msg2)
        self.proxy.insert_msg('msg_new', self.msg3)
        db_result = self.proxy.query_latest_msgs('msg', self.type)
        self.assertEqual(db_result, {'msg': self.msg2, 'msg_new': self.msg3})

    def test_query_latest_msgs_not_existing(self):
        self.proxy.insert_msg('msg1', self.msg1)
        db_result = self.proxy.query_latest_msgs('msg2', self.type)
        self.assertEqual(db_result, {})

    def test_delete_msg(self):
        self.proxy.insert_msg('msg', self.msg1)
        self.proxy.insert_msg('msg', self.msg2)
        self.proxy.delete_msg('msg', self.type, only_latest=True)
        db_msg1 = self.proxy.query_latest_msg('msg', self.type)
        self.assertEqual(db_msg1, self.msg1)

    def test_delete_msg_non_existing(self):
        with self.assertRaises(KeyError):
            self.proxy.delete_msg('msg', self.type, only_latest=True)

    def test_delete_msgs(self):
        self.proxy.insert_msg('msg1', self.msg1)
        self.proxy.insert_msg('msg2', self.msg2)
        self.proxy.delete_msgs('msg', self.type, only_latest=False)
        with self.assertRaises(KeyError):
            self.proxy.query_latest_msg('msg', self.type)

    def test_delete_msgs_non_existing(self):
        with self.assertRaises(KeyError):
            self.proxy.delete_msgs('msg', self.type)

    def test_query_entries(self):
        self.proxy.insert_msg('msg1', self.msg1)
        self.proxy.insert_msg('msg2', self.msg2)

        entries = self.proxy.query_entries('msg', self.type, is_regex=True, only_latest=False, max_age=None)
        self.assertEqual(len(entries), 2)
        self.assertEqual(entries[0][0], self.msg1)

    def test_query_entries_age(self):
        self.proxy.insert_msg('msg1', self.msg1)
        rospy.sleep(0.5)
        self.proxy.insert_msg('msg1', self.msg2)
        self.proxy.insert_msg('msg1', self.msg3)
        self.proxy.insert_msg('msg4', self.msg4)

        entries1 = self.proxy.query_entries('msg', self.type, is_regex=True, only_latest=False,
                                            max_age=0.5)
        entries2 = self.proxy.query_entries('msg', self.type, is_regex=True, only_latest=False,
                                            max_age=rospy.Duration(0.5))
        self.assertEqual(len(entries1), 3)
        self.assertEqual(entries1, entries2)
        for entry in entries1:
            self.assertIn(entry[0], [self.msg2, self.msg3, self.msg4])

    def test_query_entries_age_and_latest(self):
        self.proxy.insert_msg('msg1', self.msg1)
        rospy.sleep(0.5)
        self.proxy.insert_msg('msg1', self.msg2)
        self.proxy.insert_msg('msg1', self.msg3)
        self.proxy.insert_msg('msg4', self.msg4)

        entries = list(self.proxy.query_entries('msg', self.type, is_regex=True, only_latest=True, max_age=0.5))
        self.assertEqual(len(entries), 2)
        for entry in entries:
            self.assertIn(entry[0], [self.msg3, self.msg4])

    def test_query_entries_latest(self):
        self.proxy.insert_msg('msg1', self.msg1)
        self.proxy.insert_msg('msg1', self.msg2)
        entries = list(self.proxy.query_entries('msg', self.type, is_regex=True, only_latest=True))
        self.assertEqual(entries[0][0], self.msg2)

    def test_query_entries_no_regex(self):
        self.proxy.insert_msg('msg1', self.msg1)
        self.proxy.insert_msg('msg11', self.msg2)
        entries = list(self.proxy.query_entries('msg1', self.type, is_regex=False, only_latest=True))
        self.assertEqual(entries[0][0], self.msg1)

    # TESTING MAX_AGE FILTER

    def test_get_latest_msg_age(self):
        self.proxy.insert_msg('msg', self.msg1)
        age = self.proxy.get_latest_msg_age('msg', self.type)
        self.assertIsInstance(age, float)
        self.assertTrue(age > 0)

    def test_query_msg_with_age(self):
        self.proxy.insert_msg('msg', self.msg1)
        db_msg1 = self.proxy.query_latest_msg('msg', self.type, max_age=0.5)
        self.assertEqual(db_msg1, self.msg1)

    def test_query_msg_with_age_rospy_duration(self):
        self.proxy.insert_msg('msg', self.msg1)
        db_msg1 = self.proxy.query_latest_msg('msg', self.type, max_age=rospy.Duration.from_sec(0.5))
        self.assertEqual(db_msg1, self.msg1)

    def test_query_msg_with_age_exceed(self):
        self.proxy.insert_msg('msg', self.msg1)
        self.proxy.insert_msg('msg', self.msg2)
        self.proxy.insert_msg('msg_new', self.msg3)
        rospy.sleep(0.5)
        with self.assertRaises(AgeLimitExceeded):
            self.proxy.query_latest_msg('msg', self.type, max_age=0.5)
        with self.assertRaises(AgeLimitExceeded):
            self.proxy.query_latest_msg('msg', self.type, max_age=rospy.Duration.from_sec(0.5))

    def test_query_latest_msgs_age(self):
        self.proxy.insert_msg('msg', self.msg1)
        self.proxy.insert_msg('msg', self.msg2)
        self.proxy.insert_msg('msg_new', self.msg3)
        rospy.sleep(0.5)
        db_result = self.proxy.query_latest_msgs('msg', self.type, max_age=2)

        self.assertEqual(len(db_result.values()), 2)
        self.assertEqual(db_result['msg'], self.msg2)
        self.assertEqual(db_result['msg_new'], self.msg3)

        db_result = self.proxy.query_latest_msgs('msg', self.type, max_age=0.5)
        self.assertEqual(db_result, {})

    def test_query_latest_msgs_age_non_existing(self):
        self.proxy.insert_msg('msg', self.msg1)
        rospy.sleep(0.5)
        db_result = self.proxy.query_latest_msgs('msg', self.type, max_age=0.5)
        self.assertEqual(db_result, {})

    def test_delete_latest_msg_with_age(self):
        self.proxy.insert_msg('msg', self.msg1)
        self.proxy.insert_msg('msg', self.msg2)
        rospy.sleep(0.5)

        with self.assertRaises(KeyError):
            self.proxy.delete_msg('msg', self.type, only_latest=False, max_age=0.5)

        self.proxy.delete_msg('msg', self.type, only_latest=False, max_age=2)


if __name__ == '__main__':
    rospy.init_node("test_koco_message_store_proxy")
    import rostest
    rostest.rosrun('koco_proxies', 'test_koco_message_store_proxy', TestKocoMessageStoreProxy)
