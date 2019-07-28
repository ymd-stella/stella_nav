import rclpy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
import tf
from stella_nav_core.stella_nav_node import get_node

class PlanListener(object):
    def __init__(self, handlers, handler, topic, frame_id="map", **kwargs):
        super(PlanListener, self).__init__()
        self._handler = handlers[handler]
        self._topic = topic
        self._frame_id = frame_id
        self._pub = get_node().create_publisher(Path, topic, rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1))

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
            except RuntimeError as e:
                get_node().get_logger().debug("PlanListener: {}".format(e))
