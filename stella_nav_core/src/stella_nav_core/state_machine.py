from transitions.extensions import GraphMachine


class StateMachine(object):
    def __init__(self, states, transitions, draw_file="/tmp/state_machine.png", draw_prog="dot", initial=None):
        if initial is None:
            initial = states[0]
        self._machine = GraphMachine(
            model=self,
            states=states,
            transitions=transitions,
            initial=initial,
            ignore_invalid_triggers=True)
        if draw_file:
            self._machine.get_graph().draw(draw_file, prog=draw_prog)
