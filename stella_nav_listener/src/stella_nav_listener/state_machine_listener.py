import rclpy
from std_msgs.msg import String
from stella_nav_core.stella_nav_node import get_node


class StateMachineListener(object):
    def __init__(self, handlers, state_machine, **kwargs):
        self._state_machine = state_machine
        self._handlers = handlers
        self._pub = get_node().create_publisher(String, "~state", rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1))

    def __call__(self, msg):
        state_str = str(self._state_machine.state)
        try:
            self._pub.publish(String(data=state_str))
        except RuntimeError as e:
            get_node().get_logger().debug("StateMachineListener: {}".format(e))
        self._handlers["common_msg"].update_msg("~state_overlay", state_str)
