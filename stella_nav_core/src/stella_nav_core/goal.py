from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, PoseStamped
from .geometry_utils import GeometryUtils
import rospy
import tf


class Goal(object):
    def __init__(self, header=None, pose=None, **kwargs):
        if header is None:
            self.header = Header(
                frame_id="map",
                stamp=rospy.Time.now())
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
