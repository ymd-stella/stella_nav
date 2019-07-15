import rospy

class ForceRecoveryListener(object):
    def __init__(self, state_machine, **kwargs):
        self._state_machine = state_machine

    def __call__(self, msg):
        rospy.logdebug("force recovery event")
        self._state_machine.trigger("recoverable")
