import rclpy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from stella_nav_core.stella_nav_node import get_node

class GridMap2DListener(object):
    def __init__(self, topic, handler, handlers, **kwargs):
        super(GridMap2DListener, self).__init__()
        self._handler = handlers[handler]
        self._pub = get_node().create_publisher(OccupancyGrid, topic, rclpy.qos.QoSProfile(history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1))
        self._costmap = None

    def __call__(self, msg):
        costmap = self._handler.costmap()
        if costmap is not None:
            cells = costmap.cells
            origin = costmap.origin
            resolution = costmap.resolution
            msg = OccupancyGrid()
            msg.header.frame_id = "map"
            msg.header.stamp = costmap.stamp
            msg.info = MapMetaData(
                map_load_time=get_node().get_clock().now().to_msg(),
                resolution=resolution,
                width=cells.shape[0],
                height=cells.shape[1],
                origin=Pose(
                    position=Point(origin[1] * resolution, origin[0] * resolution, 0),
                    orientation=Quaternion(0, 0, 0, 1))
            )
            data = cells.flatten()
            data_out = np.full(data.shape, 100)
            mask = data <= 0.99
            data_out[mask] = (data[mask] * 98 + 0.5).astype(int)
            msg.data = data_out
            try:
                self._pub.publish(msg)
            except RuntimeError as e:
                get_node().get_logger().debug("GridMap2DListener: {}".format(e))
