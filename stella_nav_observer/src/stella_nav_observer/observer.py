
class Observer(object):
    def __init__(self, **kwargs):
        self._listeners = []

    def add_listener(self, listener):
        self._listeners.append(listener)

    def _call_event(self, msg):
        for listener in self._listeners:
            listener(msg)
