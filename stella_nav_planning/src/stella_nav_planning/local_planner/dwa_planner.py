import threading
import numpy as np
import time
import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from stella_nav_core.geometry_utils import GeometryUtils
from stella_nav_core.config import CostConfig, MotionConfig


class DWAPlanner(object):
    LETHAL_COST = 1000.0

    def __init__(
            self, costmaps, costmap, linear_motion_config, angular_motion_config,
            dt=0.1, heading_lookahead=0.1, predict_time=1.0, predict_time_lethal=2.0,
            default_road_width=0.5, heading_lethal_angle=np.pi/4, debug_cloud=False,
            angular_speed_cost_config=CostConfig(0.01, 1.0), speed_cost_config=CostConfig(0.01, 1.0),
            heading_cost_config=CostConfig(0.01, 1.0), goal_cost_config=CostConfig(1.0, 5.0),
            obstacle_cost_config=CostConfig(100.0, 100.0)
    ):
        self._linear_motion_config = MotionConfig(**linear_motion_config)
        self._angular_motion_config = MotionConfig(**angular_motion_config)
        self._dt = dt
        self._predict_time = predict_time
        self._predict_time_lethal = predict_time_lethal
        self._twist = None
        self._heading_lookahead = heading_lookahead
        self._debug_cloud = debug_cloud
        self._angular_speed_cost_config = CostConfig(**angular_speed_cost_config)
        self._speed_cost_config = CostConfig(**speed_cost_config)
        self._heading_cost_config = CostConfig(**heading_cost_config)
        self._goal_cost_config = CostConfig(**goal_cost_config)
        self._obstacle_cost_config = CostConfig(**obstacle_cost_config)
        self._default_road_width = default_road_width
        self._heading_lethal_angle = heading_lethal_angle
        self._costmaps = costmaps
        self._costmap = costmaps[costmap]
        self._cost_pub = rospy.Publisher("~dwa_planner/cost_cloud", PointCloud2, queue_size=1)
        self._lethal_cost_pub = rospy.Publisher("~dwa_planner/lethal_cost_cloud", PointCloud2, queue_size=1)
        self._rotation_cost_pub = rospy.Publisher("~dwa_planner/rotation_cost_cloud", PointCloud2, queue_size=1)
        self._fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="speed", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="obstacle", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="goal", offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name="angular_speed", offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name="heading", offset=28, datatype=PointField.FLOAT32, count=1),
            PointField(name="total", offset=32, datatype=PointField.FLOAT32, count=1),
        ]
        self.lock = threading.RLock()

    def update_twist(self, twist):
        self.lock.acquire()
        self._twist = twist
        self.lock.release()

    def _trajectory(self, x, y, theta, vx, avz):
        t = np.linspace(0, self._predict_time_lethal, self._predict_time_lethal / self._dt)[np.newaxis, :, np.newaxis]
        v = np.repeat(
            np.vstack((vx * np.cos(theta), vx * np.sin(theta), np.zeros(vx.shape))).T[:, np.newaxis, :],
            t.shape[1], axis=1)
        pos = np.array((x, y, theta))[np.newaxis, np.newaxis, :]
        traj = np.zeros(v.shape)
        traj[avz != 0.0] = np.vstack(
            ((vx / avz) * (np.sin(avz * t + theta) - np.sin(theta)) + x,
             (vx / avz) * (np.cos(theta) - np.cos(avz * t + theta)) + y,
             avz * t + theta)).T
        return traj

    def _heading_cost(self, scoring_point, goal):
        target_yaw = GeometryUtils.get_yaw(goal.pose.orientation)
        angle = np.abs(GeometryUtils.regulate_rad(target_yaw - scoring_point[:, 0, 2]))
        cost = self._heading_cost_config.get_cost(angle / np.pi)
        cost[angle > self._heading_lethal_angle] += DWAPlanner.LETHAL_COST
        return cost

    def _angular_speed_cost(self, avz):
        return self._angular_speed_cost_config.get_cost(np.abs(avz) / self._linear_motion_config.max_speed)

    def _speed_cost(self, vx):
        max_speed = max(self._linear_motion_config.max_speed, -self._linear_motion_config.min_speed)
        return self._speed_cost_config.get_cost(max_speed - np.abs(vx)) / max_speed

    def _speed_cost2(self, vx, scoring_point, goal):
        target_yaw = GeometryUtils.get_yaw(goal.pose.orientation)
        theta = scoring_point[:, 0, 2] - target_yaw
        max_speed = max(self._linear_motion_config.max_speed, -self._linear_motion_config.min_speed)
        return self._speed_cost_config.get_cost(
            (max_speed - np.abs(vx) * np.cos(theta)) / max_speed
        )

    def _obstacle_cost(self, traj, scoring_point, costmap):
        yaw = scoring_point[:, :, 2]
        bias = np.stack((self._heading_lookahead * np.cos(yaw), self._heading_lookahead * np.sin(yaw)), axis=-1)
        lethal_cost = np.zeros((scoring_point.shape[0], 1))
        lethal_yaw = traj[:, :, 2]
        lethal_look_point = traj[:, :, :2] + np.stack((self._heading_lookahead * np.cos(lethal_yaw), self._heading_lookahead * np.sin(lethal_yaw)), axis=-1)

        current_pos = traj[:, 0:1, :2]
        current_yaw = traj[:, 0:1, 2]
        current_bias = np.stack((self._heading_lookahead * np.cos(current_yaw), self._heading_lookahead * np.sin(current_yaw)), axis=-1)
        current_look_point = current_pos + current_bias

        lethal_cost[np.any(
            (costmap.get_value_from_world(lethal_look_point) > 0.99) * (np.linalg.norm(current_look_point - lethal_look_point, axis=2) > 1e-3),
            axis=1)] = DWAPlanner.LETHAL_COST
        look_point = scoring_point[:, :, :2] + bias
        cost = self._obstacle_cost_config.get_cost(costmap.get_value_from_world(look_point))
        return (cost + lethal_cost).reshape(cost.shape[0])

    def _explicit_goal_cost(self, scoring_point, goal):
        yaw = scoring_point[:, 0, 2]
        return self._goal_cost_config.get_cost(np.hypot(
            goal.pose.position.x - (scoring_point[:, 0, 0] + self._heading_lookahead * np.cos(yaw)),
            goal.pose.position.y - (scoring_point[:, 0, 1] + self._heading_lookahead * np.sin(yaw))))

    def _goal_cost(self, scoring_point, goal):
        robot_yaw = scoring_point[:, 0, 2]
        robot_pos = np.array(
            (scoring_point[:, 0, 0] + self._heading_lookahead * np.cos(robot_yaw),
             scoring_point[:, 0, 1] + self._heading_lookahead * np.sin(robot_yaw))).T
        goal_pos = np.array((goal.pose.position.x, goal.pose.position.y))
        u = robot_pos - goal_pos
        goal_yaw = GeometryUtils.get_yaw(goal.pose.orientation)
        v = (np.cos(goal_yaw), np.sin(goal_yaw))
        square_distance = np.square(np.cross(u, v)) / np.square(goal.data.get("road_width", self._default_road_width))
        cost = self._goal_cost_config.get_cost(square_distance)
        cost[square_distance > 1.0] += DWAPlanner.LETHAL_COST
        return cost

    def _cost(self, trajectory, scoring_point, costmap, goal, vx, avz):
        # speed_cost = self._speed_cost(vx)
        speed_cost = self._speed_cost2(vx, scoring_point, goal)
        obstacle_cost = self._obstacle_cost(trajectory, scoring_point, costmap)
        if goal.data.get("explicit", None):
            goal_cost = self._explicit_goal_cost(scoring_point, goal)
        else:
            goal_cost = self._goal_cost(scoring_point, goal)
        angular_speed_cost = self._angular_speed_cost(avz)
        heading_cost = self._heading_cost(scoring_point, goal)
        costs = (speed_cost, obstacle_cost, goal_cost, angular_speed_cost, heading_cost)
        return sum(costs), costs

    def plan(self, pose, goal):
        if self._twist is None:
            return np.array((0.0, 0.0)), None
        self._costmap.lock.acquire()
        costmap = self._costmap.clone()
        self._costmap.lock.release()

        twist = self._twist
        linear_vx = twist.linear.x
        angular_vz = twist.angular.z
        dw = [
            max(self._linear_motion_config.min_speed, min(self._linear_motion_config.max_speed, linear_vx - self._linear_motion_config.max_accel * self._dt)),
            min(self._linear_motion_config.max_speed, max(self._linear_motion_config.min_speed, linear_vx + self._linear_motion_config.max_accel * self._dt)),
            max(self._angular_motion_config.min_speed, min(self._angular_motion_config.max_speed, angular_vz - self._angular_motion_config.max_accel * self._dt)),
            min(self._angular_motion_config.max_speed, max(self._angular_motion_config.min_speed, angular_vz + self._angular_motion_config.max_accel * self._dt))
        ]
        x, y, theta = pose.pose.position.x, pose.pose.position.y, GeometryUtils.get_yaw(pose.pose.orientation)
        vx = np.linspace(dw[0], dw[1], self._linear_motion_config.samples)
        avz = np.linspace(dw[2], dw[3], self._angular_motion_config.samples)
        avz[avz == 0.0] = 1e-6
        _vx, _avz = np.meshgrid(vx, avz)
        _vx = _vx.flatten()
        _avz = _avz.flatten()

        trajectory = self._trajectory(x, y, theta, _vx, _avz)
        scoring_idx = int(self._predict_time / self._dt)
        scoring_point = trajectory[:, scoring_idx:scoring_idx+1, :]
        cost, costs = self._cost(trajectory, scoring_point, costmap, goal, _vx, _avz)

        if self._debug_cloud:
            header = Header(frame_id="map")
            points = np.vstack((trajectory[:, scoring_idx, :2].T, np.zeros(cost.shape), costs, cost)).T
            mask = _vx == dw[0]
            theta_rot = trajectory[mask, scoring_idx, 2:3].T
            r = 0.1
            points_rot = np.vstack((x + r * np.cos(theta_rot), y + r * np.sin(theta_rot), np.zeros(cost[mask].shape), [c[mask] for c in costs], cost[mask])).T
            points_rot_filtered = points_rot[points_rot[:, 3 + len(costs)] < DWAPlanner.LETHAL_COST]
            points_filtered = points[points[:, 3 + len(costs)] < DWAPlanner.LETHAL_COST]
            points_filtered_out = points[points[:, 3 + len(costs)] > DWAPlanner.LETHAL_COST - 1]

            cost_msg = point_cloud2.create_cloud(header, self._fields, points_filtered)
            lethal_cost_msg = point_cloud2.create_cloud(header, self._fields, points_filtered_out)
            rotation_cost_msg = point_cloud2.create_cloud(header, self._fields, points_rot_filtered)
            try:
                self._cost_pub.publish(cost_msg)
                self._lethal_cost_pub.publish(lethal_cost_msg)
                self._rotation_cost_pub.publish(rotation_cost_msg)
            except rospy.ROSException as e:
                rospy.logdebug("DWAPlanner: {}".format(e))

        min_idx = np.argmin(cost)
        min_score = (cost[min_idx], _vx[min_idx], _avz[min_idx], trajectory[min_idx][:scoring_idx])
        if min_score is None:
            min_score = (10000.0, 0.0, 0.0, [])
        return min_score[1:3], min_score[3]
