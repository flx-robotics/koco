import rospy
import rostopic

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from koco_msgs.msg import DigitalIOArray


class WaitForDigitalInput(EventState):
    '''
    Waits for a digital input to hold a predefined value.

    -- topic            string      Name of the digital inputs topic.
    
    -- input_address    int         The address of the digital input.

    -- desired_state    bool        Desired state of the digital input.

    -- timeout          float32     Amount of time till timeout.

    <= true                         If digital input is in a desired state.

    <= false                        If digital input is not in a desired state.

    <= error                        If topic does not exist or something else goes wrong.

    '''

    def __init__(self, topic, input_address, desired_state, timeout=0.0):
        super(WaitForDigitalInput, self).__init__(
            outcomes=['true', 'false', 'error'])

        self._topic = topic
        self._input_address = input_address
        self._desired_state = desired_state
        self._timeout = timeout
        self._start_time = rospy.get_time()
        self._connected = False
        
        _, msg_topic, _ = rostopic.get_topic_type(self._topic)

        if msg_topic == self._topic:
            self._sub = ProxySubscriberCached({self._topic: DigitalIOArray})
            self._connected = True
        else:
            error = 'Topic %s for state %s not yet available.\nFound: %s\nWill try again when entering the state...'
            Logger.logwarn(error % (self._topic, self.name, str(msg_topic)))

    def execute(self, userdata):
        if not self._connected:
            return 'error'

        if self._sub.has_msg(self._topic):
            data = self._sub.get_last_msg(self._topic)
            # self._sub.remove_last_msg(self._topic)
            try:
                for io in data.io_array:
                    if io.address == self._input_address and io.state == self._desired_state:
                        return 'true'
            except Exception as e:
                rospy.logerr('Exception occured:\n{}'.format(e))
                return 'error'
        if rospy.get_time() - self._start_time > self._timeout:
            return 'false'

    def on_enter(self, userdata):
        if not self._connected:
            _, msg_topic, _ = rostopic.get_topic_type(self._topic)
            if msg_topic == self._topic:
                self._sub = ProxySubscriberCached({self._topic: DigitalIOArray})
                self._connected = True
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topic)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._topic)

        self._start_time = rospy.get_time()

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
