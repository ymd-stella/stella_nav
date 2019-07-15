import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
import tf

class PlanListener(object):
    def __init__(self, handlers, handler, topic, frame_id="map", **kwargs):
        super(PlanListener, self).__init__()
        self._handler = handlers[handler]
        self._topic = topic
        self._frame_id = frame_id
        self._pub = rospy.Publisher(topic, Path, queue_size=1)

    def __call__(self, msg):
        msg = Path()
        msg.header.frame_id = self._frame_id
        plan = self._handler.plan()
        if plan is not None:
            msg.poses = [PoseStamped(
                header=Header(frame_id=self._frame_id),
                pose=Pose(
                    position=Point(x=p[0], y=p[1], z=0),
                    orientation=Quaternion(x=0, y=0, z=0, w=1)))
                         for p in plan]
            try:
                self._pub.publish(msg)
            except rospy.ROSException as e:
                rospy.logdebug("PlanListener: {}".format(e))
