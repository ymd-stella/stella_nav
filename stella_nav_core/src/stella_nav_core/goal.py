from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, PoseStamped
from .geometry_utils import GeometryUtils
import rclpy
import tf
from stella_nav_core.stella_nav_node import get_node


class Goal(object):
    def __init__(self, header=None, pose=None, **kwargs):
        if header is None:
            self.header = Header(
                frame_id="map",
                stamp=get_node().get_clock().now().to_msg())
        else:
            self.header = header
        if pose is None:
            self.pose = Pose(
                position=Point(kwargs["x"], kwargs["y"], 0),
                orientation=GeometryUtils.get_quaternion(kwargs["theta"]))
        else:
            self.pose = pose
        self.pose_stamped = PoseStamped(header=self.header, pose=self.pose)
        self.data = kwargs

    @staticmethod
    def from_msg(msg):
        return Goal(header=msg.header, pose=msg.pose)

    def __repr__(self):
        return "<Goal {}>".format(self.data)

    def __str__(self):
        return "<Goal {}>".format(self.data)
