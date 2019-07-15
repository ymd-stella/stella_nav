import rospy
from std_msgs.msg import String


class StateMachineListener(object):
    def __init__(self, handlers, state_machine, **kwargs):
        self._state_machine = state_machine
        self._handlers = handlers
        self._pub = rospy.Publisher("~state", String, queue_size=1)

    def __call__(self, msg):
        state_str = str(self._state_machine.state)
        try:
            self._pub.publish(String(data=state_str))
        except rospy.ROSException as e:
            rospy.logdebug("StateMachineListener: {}".format(e))
        self._handlers["common_msg"].update_msg("~state_overlay", state_str)
