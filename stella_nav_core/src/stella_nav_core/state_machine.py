from transitions import Machine


class StateMachine(object):
    def __init__(self, states, transitions, initial=None):
        if initial is None:
            initial = states[0]
        self._machine = Machine(
            model=self,
            states=states,
            transitions=transitions,
            initial=initial,
            ignore_invalid_triggers=True)
