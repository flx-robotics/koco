import rospy
import rostopic

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from koco_msgs.msg import DigitalIOArray


class CheckDigitalInput(EventState):
    '''
    Checks the state of a digital input.

    -- topic            string      Name of the digital inputs topic.

    -- input_address    int         The address of the digital input.

    <= true                         If digital input is True

    <= false                        If digital input is False

    <= error                        If topic does not exist or something else goes wrong.

    '''

    def __init__(self, topic, input_address):
        super(CheckDigitalInput, self).__init__(
            outcomes=['true', 'false', 'error'])

        self._topic = topic
        self._input_address = input_address
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

            input_state = None
            for io in data.io_array:
                if io.address == self._input_address:
                    input_state = io.state
                    
                    if input_state is True:
                        return 'true'
                    elif input_state is False:
                        return 'false'

            Logger.logwarn('The topic did not provide the expected data')
            Logger.logwarn('input_state = {}'.format(input_state))
            return 'error'
        else:
            Logger.logwarn('The topic did not provide any data')
            return 'error'

    def on_enter(self, userdata):
        if not self._connected:
            _, msg_topic, _ = rostopic.get_topic_type(self._topic)
            if msg_topic == self._topic:
                self._sub = ProxySubscriberCached({self._topic: DigitalIOArray})
                self._connected = True
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topic)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._topic)

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
