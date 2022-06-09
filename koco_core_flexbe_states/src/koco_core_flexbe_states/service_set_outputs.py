import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from koco_msgs.srv import SetOutputs, SetOutputsRequest
from koco_msgs.msg import DigitalIO


class ServiceSetOutputs(EventState):
    '''
    Calls a desired service with SetOutputs message type.

    -- service_name     string          Name of a SetOutputs service.

    -- addresses        array[int]      Array of output addresses that need to be set.

    -- states           array[bool]     Array of states (one state for each address) that outputs need to be set to.

    #> success          bool            Output key containing a success of a service call.

    #> message          string          Output key containing response message of a service call.

    <= true                             If service call returns true.

    <= false                            If service call returns false.

    <= failed                           If something goes wrong with a service call.

    '''

    def __init__(self, service_name="", addresses=[], states=[]):
        super(ServiceSetOutputs, self).__init__(
            outcomes=['true', 'false', 'failed'],
            output_keys=['success', 'message'])

        self._service_name = service_name
        self._addresses = addresses
        self._states = states
        self._client = ProxyServiceCaller({self._service_name: SetOutputs})
        self._response = None
        self._error = False

    def execute(self, userdata):
        if not self._error:
            return str(self._response.success).lower()
        return 'failed'

    def on_enter(self, userdata):
        self._error = False

        if not len(self._addresses):
            self._error = True
            Logger.logerr('Provided array of addresses is empty!')
            return
        elif len(self._addresses) != len(self._states):
            self._error = True
            Logger.logerr('Provided arrays of addresses and states have different sizes!')
            return

        outputs = []
        for (i, j) in zip(self._addresses, self._states):
            if type(i) is not int or type(j) is not bool:
                self._error = True
                Logger.logerr('Provided data in arrays are not of proper type!')
                return
            outputs.append(DigitalIO(address=i, state=j))

        try:
            self._response = self._client.call(self._service_name, SetOutputsRequest(outputs))
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
