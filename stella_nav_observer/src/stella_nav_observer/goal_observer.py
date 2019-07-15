import rospy
from .topic_observer import TopicObserver
from geometry_msgs.msg import PoseStamped


class GoalObserver(TopicObserver):
    def __init__(self, **kwargs):
        super(GoalObserver, self).__init__(**kwargs)
        # move_base_simple/goal in navigation stack
        self._goal_subscriber = rospy.Subscriber("~goal", PoseStamped, self._update_msg)
