import rclpy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, PoseStamped
from stella_nav_core.stella_nav_node import get_node

class GoalListener(object):
    def __init__(self, handlers, handler, **kwargs):
        self._goal_handler = handlers[handler]
        self._goals_pub = get_node().create_publisher(PoseArray, "~" + handler+"/goals", rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1))
        self._current_goal_pub = get_node().create_publisher(PoseStamped, "~" + handler+"/current_goal", rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1))

    def __call__(self, msg):
        goals = self._goal_handler.get_goals()
        goals_msg = PoseArray(
            header=Header(
                frame_id="map",
                stamp=get_node().get_clock().now().to_msg()),
            poses=[g.pose for g in goals]
        )
        try:
            current_goal = self._goal_handler.get_current_goal()
            if current_goal:
                self._current_goal_pub.publish(current_goal.pose_stamped)
            self._goals_pub.publish(goals_msg)
        except RuntimeError as e:
            get_node().get_logger().debug("GoalHandler: {}".format(e))
