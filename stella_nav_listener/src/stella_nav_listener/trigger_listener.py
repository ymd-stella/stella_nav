class TriggerListener(object):
    def __init__(self, state_machine, **kwargs):
        self._state_machine = state_machine

    def __call__(self, msg):
        self._state_machine.trigger(msg.data)
