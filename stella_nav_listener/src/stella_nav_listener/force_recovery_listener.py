import rclpy
from stella_nav_core.stella_nav_node import get_node

class ForceRecoveryListener(object):
    def __init__(self, state_machine, **kwargs):
        self._state_machine = state_machine

    def __call__(self, msg):
        get_node().get_logger().debug("force recovery event")
        self._state_machine.trigger("recoverable")
