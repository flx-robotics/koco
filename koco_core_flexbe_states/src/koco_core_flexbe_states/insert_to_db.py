import rospy

from flexbe_core import EventState, Logger
from koco_proxies.proxies import KocoMessageStoreProxy


class InsertToDb(EventState):
    '''
    Insert (or update) an object to the db.

    -- db_name         string       Database name (can be empty for the KocoPose).

    -- update          bool         If true the object will be updated otherwise inserted.

    ># ros_object      object       Input key containing an object to the db.

    <= done                         If successfully inserted (updated).

    <= error                        If something goes wrong.

    '''

    def __init__(self, db_name='', update=True):
        super(InsertToDb, self).__init__(
            outcomes=['done', 'error'],
            input_keys=['ros_object'])

        self._db_name = db_name
        self._update = update
        self._msg_store = KocoMessageStoreProxy().get_proxy()
        self._error = False

    def execute(self, userdata):
        if not self._error:
            return 'done'
        return 'error'

    def on_enter(self, userdata):
        self._error = False
        try:
            db_name = self._db_name if self._db_name else userdata.ros_object.child_frame_id
            if not self._update:
                self._msg_store.insert_named(db_name, userdata.ros_object)
            else:
                results = self._msg_store.query_named(db_name, userdata.ros_object._type, single=False)
                if results:
                    self._error = not self._msg_store.update_id(results[-1][1]['_id'], userdata.ros_object).success
                else:
                    self._msg_store.insert_named(db_name, userdata.ros_object)
        except Exception as e:
            self._error = True
            Logger.logerr('Exception occurred in InsertToDb:\n{}'.format(e))

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
