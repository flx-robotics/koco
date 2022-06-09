#! /usr/bin/env python

import copy
import json
import re
from contextlib import contextmanager
from datetime import datetime

import rospy
import tf2_ros
from mongodb_store.message_store import MessageStoreProxy
from std_msgs.msg import String


class KocoTransformListenerProxy(object):
    _listener = None
    _buffer = None

    def __init__(self):
        if KocoTransformListenerProxy._listener is None:
            KocoTransformListenerProxy._buffer = tf2_ros.Buffer()
            KocoTransformListenerProxy._listener = tf2_ros.TransformListener(KocoTransformListenerProxy._buffer)

    def get_buffer(self):
        return KocoTransformListenerProxy._buffer


class AgeLimitExceeded(Exception):
    """Exception that indicates when an age limit for the message has been exceeded."""


class KocoMessageStoreProxy(object):
    _db_proxy = None

    def __init__(self):
        if KocoMessageStoreProxy._db_proxy is None:
            KocoMessageStoreProxy._db_proxy = MessageStoreProxy()

    def get_proxy(self):
        return KocoMessageStoreProxy._db_proxy

    @contextmanager
    def edit_latest_msg(self, name, type):
        """ Context manager for editing the existing message from the database and updating it afterwards.
            Usage:
            >>> with edit_latest_msg('msg_name', msg_type) as msg:
            ...     msg.some_field = something_else
        Args:
            name (str): Message name.
            type (str): Message type.
        Yields:
            ROS msg: tessage from the database.
        """
        msg = self.query_latest_msg(name, type)
        yield msg
        self.update_latest_msg(name, msg)

    def _query_latest_entry(self, name, type, max_age=None):
        """ Queries the last inserted message by name and returns the corresponding database entry.
        Args:
            name (str): Message name.
            type (str): Message type.
            max_age (rospy.Duration, int, optional): Max time passed in seconds.
                If entry exceeds the age limit, 'AgeLimitExceeded' exception is risen.
        Raises:
            KeyError: When queried message doesn't exists.
            AgeLimitExceeded: When queried message exceeds the age limit.
        Returns:
            tuple[ROS msg, meta_dict]: Latest entry from the database.
        """
        latest_entry = KocoMessageStoreProxy._db_proxy.query(
            type, meta_query={'name': name}, single=True, sort_query=[("$natural", -1)])

        if not latest_entry[0]:
            raise KeyError("Could not find message named '{}' in the database.".format(name))

        if not self._check_entry_age_condition(latest_entry, max_age):
            raise AgeLimitExceeded("Message named '{}' exceeds age limit.".format(name))

        return latest_entry

    def _check_entry_age_condition(self, db_entry, max_age):
        """Returns boolean value whether the database entry is newer than age limit."""
        if max_age is not None:
            if isinstance(max_age, rospy.Duration):
                max_age = max_age.to_sec()
            return self._get_entry_age_in_secs(db_entry) < max_age
        else:
            return True

    def _get_entry_age_in_secs(self, db_entry):
        """Returns age of the database entry in seconds."""
        try:
            entry_time = db_entry[1]['last_updated_at']
        except KeyError:
            entry_time = db_entry[1]['inserted_at']
        return (datetime.now(entry_time.tzinfo) - entry_time).total_seconds()

    def get_latest_msg_age(self, name, type):
        """Retruns age in seconds of the latest inserted message with the given name.
        Args:            
            name (str): Message name.
            type (str): Message type.
        Raises:
            KeyError: If the message is not found in the database.
        Returns:
            float: Age of the message in seconds.
        """
        db_entry = self._query_latest_entry(name, type)
        return self._get_entry_age_in_secs(db_entry)

    def query_latest_msg(self, name, type, max_age=None):
        """ Queries the last inserted message with the given name.
        Args:
            name (str): Message name.
            type (str): Message type.
            max_age (rospy.Duration, int, optional): Max time passed in seconds.
                If entry exceeds the age limit, 'AgeLimitExceeded' exception is risen.
        Raises:
            KeyError: When queried message doesn't exists.
            AgeLimitExceeded: When queried message exceeds the age limit.
        Returns:
            ROS msg: The latest message.
        """
        latest_msg, _ = self._query_latest_entry(name, type, max_age=max_age)
        return latest_msg

    def query_latest_msgs(self, regex_filter, type, max_age=None):
        """ Queries the last inserted messages with names matching the given regular expression.
        Args:
            regex_filter (str): Regular expression, used to filter messages by name field.
            type (str): Message type.
            max_age (rospy.Duration, int, optional): Only the latest messages that 
                doesn't exceed maximum specified age are returned.
        Returns:
            dict[str, ROS msg]: Returns a dictionary of messages with names as keys and messages as values.
        """
        entries = self.query_entries(regex_filter, type, is_regex=True, only_latest=True, max_age=max_age)
        latest_msgs = {}
        for msg, meta in entries:
            latest_msgs[meta['name']] = msg

        return latest_msgs

    def update_latest_msg(self, name, message, upsert=False):
        """ Updates the latest message with the given name. Function doesn't return 
            information if the message was updated or inserted, when upsert flag is set.
        Args:
            name (str): Message name.
            message (ROS msg): Updated message.
            upsert (bool, optional): If set to true, the message will be inserted if not found, otherwise exception is raised.
        Raises:
            KeyError: If the message with the given name doesn't exist (only if uspert flag is set to false).
            Exception: If the message could not be updated.
        Returns:
            str: The ObjectId of the MongoDB document containing the stored message.
        """
        meta = {}
        try:
            db_entry = self._query_latest_entry(name, message._type)
            id_ = str(db_entry[1]['_id'])
        except KeyError:
            if upsert:
                meta['name'] = name  # needed if upsert is True
                id_ = None
            else:
                raise

        response = KocoMessageStoreProxy._db_proxy.update_id(id_, message, meta=meta, upsert=upsert)
        if not response.success:
            raise Exception("Message named '{}' could not be updated in the database.".format(name))

        if id_ is None:  # If insertion was made
            id_ = response.id
        return id_

    def insert_msg(self, name, message):
        """ Inserts a new message in the database, even if the message with the same name already exists there.
            Function behaves exactly the same as 'MessageStoreProxy().insert_named()'.
        Args:
            name (str): Message name.
            message (ROS msg): Message.
        Returns:
            str: The ObjectId of the MongoDB document containing the stored message.
        """
        return KocoMessageStoreProxy._db_proxy.insert_named(name, message)

    def query_entries(self, name, type, is_regex=False, only_latest=True, max_age=None):
        """ Queries entries from the database and filters them according to the given arguments.
        Args:
            name (str): Message name or regular expression.
            type (str): Message type.
            is_regex (bool, optional): Flag to use name argument as accurate name or as regex. Defaults to False.
            only_latest (bool, optional): If true, query will be done only amongst latest messages.
                that are uniquely named. Defaults to True.
            max_age (rospy.Duration, int, optional): Additionally filters queried messages by age limit. 
                Max time passed is defined in seconds or by 'rospy.Duration' object.
        Returns:
            list or generator of tuple[ROS msg, meta_dict]: Filtered entry from the database.
        """
        # Create query for the name field.
        if is_regex:
            query = {"_meta.name": {'$regex': name}}
        else:
            query = {'_meta.name': name}

        # Add age limit to query if necessary.
        if max_age and not only_latest:
            if isinstance(max_age, rospy.Duration):
                max_age = max_age.to_sec()
            date_limit = datetime.utcfromtimestamp(rospy.get_rostime().to_sec() - max_age)
            query = {"$and": [
                query,
                {"$or": [
                    {"_meta.last_updated_at": {"$gt": date_limit}},
                    {"_meta.inserted_at": {"$gt": date_limit}}
                ]}
            ]}

        entries = KocoMessageStoreProxy._db_proxy.query(type, message_query=query, single=False)

        # Pipe entries through necessary filters.
        if only_latest:
            entries = self._filter_latest_entries(entries)
            if max_age:
                entries = (entry for entry in entries if self._check_entry_age_condition(entry, max_age))

        return entries

    def _filter_latest_entries(self, entries):
        """FIlters list to only latest inserted messages that are uniquely named. Returns generator object."""
        names = set()
        for entry in reversed(entries):
            name = entry[1]['name']
            if name not in names:
                names.add(name)
                yield entry

    def delete_msg(self, name, type, only_latest=True, max_age=None):
        """ Deletes all or just last inserted message with the same given name. 
            Only the ones that doesn't exceed age limit are deleted if 'max_age' is set.
        Args:
            name (str): Message name.
            type (str): Message type.
            only_latest(bool): If set, only the latest one is deleted.
            max_age (rospy.Duration, int, optional): Message can be deleted only 
                if it doesn't exceed maximum specified age (in seconds).
        Raises:
            KeyError:If no messages in the database match the given name and flags.
            Exception: If the message(s) could not be deleted.
        """
        self.delete_msgs('^{}$'.format(name), type=type, only_latest=only_latest, max_age=max_age)

    def delete_msgs(self, regex_filter, type, only_latest=True, max_age=None):
        """ Deletes all the messages that match the given regex and 
            comply with both: the 'only_latest' and 'max_age' flag.
        Args:
            regex_filter (str): Message name.
            type (str): Message type.
            only_latest(bool): If set, only the latest ones are deleted.
            max_age (rospy.Duration, int, optional): If set, only the ones that 
                don't exceed maximum specified age (in seconds) are deleted.
        Raises:
            KeyError: If no messages in the database match the given regex and flags.
            Exception: If the message(s) could not be deleted.
        """
        entries = self.query_entries(regex_filter, type, is_regex=True,
                                     only_latest=only_latest, max_age=max_age)
        count = 0
        failed_deletions = []
        for msg, meta in entries:
            id_ = str(meta['_id'])
            if not KocoMessageStoreProxy._db_proxy.delete(id_):
                failed_deletions.append(meta['name'])
            count += 1

        if not count:
            raise KeyError("No message in the database matched the query.")

        if failed_deletions:
            raise Exception("Not all messages could be deleted from the database. "
                            "Following messages could not be deleted: {}".format(failed_deletions))


class KocoParameterToDbStoreProxy(object):
    _db_proxy = None

    def __init__(self):
        if KocoParameterToDbStoreProxy._db_proxy is None:
            KocoParameterToDbStoreProxy._db_proxy = KocoMessageStoreProxy().get_proxy()

    def store_to_db(self, parameter_name, db_entry_name):
        """
        Reads parameter (could be dict or just single value) and stores it to db as a String msg
        @param parameter_name: name of the parameter that needs to be stored
        @type  parameter_name: string
        @param db_entry_name: name of the database entry to which store the parameter value (should not have slash at start)
        @type  db_entry_name: string
        """
        param_val = rospy.get_param(parameter_name)
        serialized_val = json.dumps(param_val)
        self._db_proxy.update_named(db_entry_name, String(serialized_val), upsert=True)

    def retrieve_from_db(self, db_entry_name):
        """
        Retrieves a String type database entry and deserializes it
        @param db_entry_name: name of the database entry to which the parameter value is stored (should not have slash at start)
        @type  db_entry_name: string
        @return: deserialized database entry content
        @rtype: parameter value type or dict or None if entry not found
        """
        db_result = self._db_proxy.query_named(db_entry_name, String._type)
        if db_result[0] is not None:
            deserialized_val = json.loads(db_result[0].data)
            return deserialized_val
        return None

    def retrieve_from_db_and_update_parameter(self, db_entry_name, parameter_name, delete_existing=False):
        """
        Retrieves a String type database entry and deserializes it and loads it to the parameter server
        @param db_entry_name: name of the database entry to which store the parameter value (should not have slash at start)
        @type  db_entry_name: string
        @param parameter_name: name of the parameter that the entry content should be loaded
        @type  parameter_name: string
        @param delete_existing: (optional) set True if the current parameter value should be deleted
        @type  delete_existing: bool
        @return: deserialized database entry content
        @rtype: parameter value type or dict or None if entry not found
        """
        deserialized_result = self.retrieve_from_db(db_entry_name)
        if deserialized_result is not None:
            if rospy.has_param(parameter_name) and delete_existing:
                rospy.delete_param(parameter_name)
            rospy.set_param(parameter_name, deserialized_result)
            return deserialized_result
        return None
