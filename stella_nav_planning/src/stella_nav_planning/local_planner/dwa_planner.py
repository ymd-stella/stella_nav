import threading
import numpy as np
import time
import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from stella_nav_core.geometry_utils import GeometryUtils
from stella_nav_core.config import CostConfig, MotionConfig


class State(object):
    def __init__(self, x, y, theta, vx, avz, mask_rotation=None, cost=None, costs=None,
                 accum_cost=0.0, trajectory=None, accum_trajectory=np.array([], dtype=np.float64).reshape(0, 3), level=1, backtrace=[]):
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = vx
        self.avz = avz
        self.mask_rotation = mask_rotation
        self.cost = cost
        self.accum_cost = accum_cost
        self.costs = costs
        self.trajectory = trajectory
        self.accum_trajectory = accum_trajectory
        self.level = level
        self.backtrace = backtrace


class DWAPlanner(object):
    LETHAL_COST = 1000.0

    def __init__(
            self, costmaps, costmap, linear_motion_config, angular_motion_config,
            dt=0.1, heading_lookahead=0.1, predict_time=1.0, search_level=1,
            default_road_width=0.5, heading_lethal_angle=np.pi/4, debug_cloud=True,
            angular_speed_cost_config=CostConfig(0.01, 1.0), speed_cost_config=CostConfig(0.01, 1.0),
            heading_cost_config=CostConfig(0.01, 1.0), goal_cost_config=CostConfig(1.0, 5.0),
            obstacle_cost_config=CostConfig(100.0, 100.0)
    ):
        self._linear_motion_config = MotionConfig(**linear_motion_config)
        self._angular_motion_config = MotionConfig(**angular_motion_config)
        self._dt = dt
        self._predict_time = predict_time
        self._search_level = search_level
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
        t = np.linspace(0, self._predict_time, self._predict_time / self._dt)[np.newaxis, :, np.newaxis]
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

    def _cost(self, trajectory, costmap, goal, vx, avz):
        scoring_point = trajectory[:, -1:, :]
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
        return sum(costs), np.vstack(costs)

    def _dynamic_window(self, linear_vx, angular_vz):
        dw = [
            max(self._linear_motion_config.min_speed, min(self._linear_motion_config.max_speed, linear_vx - self._linear_motion_config.max_accel * self._dt)),
            min(self._linear_motion_config.max_speed, max(self._linear_motion_config.min_speed, linear_vx + self._linear_motion_config.max_accel * self._dt)),
            max(self._angular_motion_config.min_speed, min(self._angular_motion_config.max_speed, angular_vz - self._angular_motion_config.max_accel * self._dt)),
            min(self._angular_motion_config.max_speed, max(self._angular_motion_config.min_speed, angular_vz + self._angular_motion_config.max_accel * self._dt))
        ]
        return dw

    def _sample_v(self, dw):
        _vx = np.linspace(dw[0], dw[1], self._linear_motion_config.samples)
        _avz = np.linspace(dw[2], dw[3], self._angular_motion_config.samples)
        _avz[_avz == 0.0] = 1e-6
        vx, avz = np.meshgrid(_vx, _avz)
        vx = vx.flatten()
        avz = avz.flatten()
        mask_rotation = vx == dw[0]
        return vx, avz, mask_rotation

    def _publish_cloud(self, trajectory, cost, costs, mask_rotation):
        header = Header(frame_id="map")
        points = np.vstack((trajectory[:, -1, :2].T, np.zeros(cost.shape), costs, cost)).T
        mask = mask_rotation
        x, y = trajectory[0, 0, :2]
        theta_rot = trajectory[mask, -1, 2:3].T
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

    def plan(self, pose, goal):
        self.lock.acquire()
        twist = self._twist
        self.lock.release()
        if twist is None:
            return np.array((0.0, 0.0)), None
        self._costmap.lock.acquire()
        costmap = self._costmap.clone()
        self._costmap.lock.release()

        x = pose.pose.position.x
        y = pose.pose.position.y
        theta = GeometryUtils.get_yaw(pose.pose.orientation)
        linear_vx = twist.linear.x
        angular_vz = twist.angular.z
        states = [State(x, y, theta, linear_vx, angular_vz)]
        results = []

        while states:
            state = states.pop()
            sample_vx, sample_avz, mask_rotation = self._sample_v(dw=self._dynamic_window(state.vx, state.avz))
            trajectory = self._trajectory(state.x, state.y, state.theta, sample_vx, sample_avz)
            cost, costs = self._cost(trajectory, costmap, goal, sample_vx, sample_avz)
            for i in range(len(sample_vx)):
                _vx = sample_vx[i]
                _avz = sample_avz[i]
                _mask_rotation = mask_rotation
                _cost = cost
                _costs = costs
                _accum_cost = cost[i] + state.accum_cost
                _x = trajectory[i, -1, 0]
                _y = trajectory[i, -1, 1]
                _theta = trajectory[i, -1, 2]
                _trajectory = trajectory
                _accum_trajectory = np.vstack((trajectory[i], state.accum_trajectory))
                _backtrace = state.backtrace + [state]
                new_state = State(_x, _y, _theta, _vx, _avz, _mask_rotation, _cost, _costs, _accum_cost, _trajectory, _accum_trajectory, state.level + 1, _backtrace)
                if state.level < self._search_level:
                    states.append(new_state)
                else:
                    results.append(new_state)

        min_cost = None
        min_idx = None
        min_score = None
        for state in results:
            if min_cost is None or state.accum_cost < min_cost:
                min_cost = state.accum_cost
                min_score = (state.cost, state.vx, state.avz, state.trajectory)
                min_backtrace = state.backtrace + [state]

        if self._debug_cloud:
            for state in min_backtrace[1:]:
                self._publish_cloud(state.trajectory, state.cost, state.costs, state.mask_rotation)
        if min_score is None:
            min_score = (10000.0, 0.0, 0.0, [])
        return min_score[1:3], min_score[3]
