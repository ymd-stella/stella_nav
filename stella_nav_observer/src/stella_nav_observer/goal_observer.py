import rclpy
from .topic_observer import TopicObserver
from geometry_msgs.msg import PoseStamped
from stella_nav_core.stella_nav_node import get_node


class GoalObserver(TopicObserver):
    def __init__(self, **kwargs):
        super(GoalObserver, self).__init__(**kwargs)
        # move_base_simple/goal in navigation stack
        self._goal_subscriber = get_node().create_subscription(PoseStamped, "~goal", self._update_msg, rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL))
