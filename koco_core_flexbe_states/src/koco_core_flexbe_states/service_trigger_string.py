import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from koco_msgs.srv import TriggerString, TriggerStringRequest


class ServiceTriggerString(EventState):
    '''
    Calls a desired service with TriggerString message type.

    -- service_name     string      Name of a TriggerString service.

    ># req_message      string      Input key containing a service request message.

    #> success          bool        Output key containing a success of a service call.

    #> message          string      Output key containing response message of a service call.

    <= true                         If service call returns true.

    <= false                        If service call returns false.

    <= failed                       If something goes wrong with a service call.

    '''

    def __init__(self, service_name=""):
        super(ServiceTriggerString, self).__init__(
            outcomes=['true', 'false', 'failed'],
            input_keys=['req_message'],
            output_keys=['success', 'message'])

        self._service_name = service_name
        self._client = ProxyServiceCaller({self._service_name: TriggerString})
        self._response = None
        self._error = False

    def execute(self, userdata):
        if not self._error:
            return str(self._response.success).lower()
        return 'failed'

    def on_enter(self, userdata):
        self._error = False
        try:
            req_message = userdata.req_message
            self._response = self._client.call(self._service_name, TriggerStringRequest(req_message))
            userdata.success = self._response.success
            userdata.message = self._response.message
        except Exception as e:
            self._error = True
            Logger.logerr('Exception occurred:\n{}'.format(e))

    def on_exit(self, userdata):
        pass

    def on_start(self):
        pass

    def on_stop(self):
        pass
