import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, PoseStamped

class GoalListener(object):
    def __init__(self, handlers, handler, **kwargs):
        self._goal_handler = handlers[handler]
        self._goals_pub = rospy.Publisher("~" + handler+"/goals", PoseArray, queue_size=1)
        self._current_goal_pub = rospy.Publisher("~" + handler+"/current_goal", PoseStamped, queue_size=10)

    def __call__(self, msg):
        goals = self._goal_handler.get_goals()
        goals_msg = PoseArray(
            header=Header(
                frame_id="map",
                stamp=rospy.Time.now()),
            poses=[g.pose for g in goals]
        )
        try:
            current_goal = self._goal_handler.get_current_goal()
            if current_goal:
                self._current_goal_pub.publish(current_goal.pose_stamped)
            self._goals_pub.publish(goals_msg)
        except rospy.ROSException as e:
            rospy.logdebug("GoalHandler: {}".format(e))
