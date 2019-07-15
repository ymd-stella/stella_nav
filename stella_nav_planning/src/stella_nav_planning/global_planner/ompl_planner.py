import rospy
import numpy as np
from .helper import PassableChecker, WaypointGenerator
import threading
from .optimized.ompl_planner_impl import OmplPlannerImpl


class OmplPlanner(object):
    def __init__(self, costmaps, costmap, calc_time=0.1, resolution=0.001, distance=5,
                 subgoal_distance=1.0, goal_lookahead=0, default_road_width=1.0):
        self._costmaps = costmaps
        self._costmap = costmaps[costmap]
        self._impl = OmplPlannerImpl(self._costmap.shape, calc_time, resolution, distance)
        self._impl.set_weight((0.9, 0.1))
        self._subgoal_distance = subgoal_distance
        self._goal_lookahead = goal_lookahead
        self._goal_lookahead_plus = 0
        self._latest_goal_idx = None
        self._default_road_width = default_road_width
        self._result = None, None, [], None, None
        self._thread = None

    def plan(self, goal_idx, goals, subgoal_idx, subgoals, pose):
        if self._thread is None:
            self._thread = threading.Thread(target=self._plan, args=[goal_idx, goals, subgoal_idx, subgoals, pose])
            self._thread.start()
        if self._thread.is_alive():
            return None, None, [], None, None
        else:
            self._thread = None
            return self._result

    def _update_mask(self, costmap, pose, poses, goal, planning_goals):
        # get all point of map
        m = np.vstack(np.stack(np.meshgrid(np.arange(costmap.shape[0]), np.arange(costmap.shape[1]))).T)
        # convert to world point
        w = costmap.map_to_world(m)
        # remove if world point is not contained in trajectory_range
        mask = np.full(w.shape[0], True)
        p = np.array((pose.pose.position.x, pose.pose.position.y))
        distance = np.linalg.norm(p - w)
        road_width = planning_goals[0].data.get("road_width", self._default_road_width)
        mask *= distance > road_width
        for p1, p2, planning_goal in zip(poses[:-1], poses[1:], planning_goals):
            road_width = planning_goal.data.get("road_width", self._default_road_width)
            # calculate distance from pose to w
            # and if distance < trajectory_range then remove from w
            u = p1 - w
            vec = (p2 - p1)
            yaw = np.arctan2(vec[0, 1], vec[0, 0])
            v = vec / np.linalg.norm(vec)
            _mask1 = (np.matmul(-u, vec.T) <= 0.0)[:, 0]
            mask[_mask1] *= (np.linalg.norm(u, axis=1) > road_width)[_mask1]
            _mask2 = (np.matmul(w - p2, -vec.T) <= 0.0)[:, 0]
            mask[_mask2] *= (np.linalg.norm(w - p2, axis=1) > road_width)[_mask2]
            _mask = np.logical_not(_mask1) * np.logical_not(_mask2)
            mask[_mask] *= (np.abs(np.cross(u, v)) > road_width)[_mask]
        # write costmap
        costmap.write_value_from_world(w[mask], 1.0)

    def _plan(self, goal_idx, goals, subgoal_idx, subgoals, pose):
        self._costmap.lock.acquire()
        costmap = self._costmap.clone()
        self._costmap.lock.release()

        if self._latest_goal_idx is not goal_idx:
            self._goal_lookahead_plus = 0
        self._latest_goal_idx = goal_idx
        # limit motion planning space by road_width
        poses, goal, planning_goals = PassableChecker.get_planning_goals(costmap, pose, goals, goal_idx, self._goal_lookahead + self._goal_lookahead_plus)
        self._update_mask(costmap, pose, poses, goal, planning_goals)
        goal_is_passable = PassableChecker.is_passable_at_poses(costmap, poses)

        self._impl.update_costmap(costmap)

        triggers = []
        triggers.append("goalPassable" if goal_is_passable else "goalImpassable")
        need_to_plan = True
        if subgoal_idx is not None:
            subgoal_is_passable = PassableChecker.is_passable(costmap, subgoal_idx, subgoals, pose, self._goal_lookahead)
            triggers.append("subgoalPassable" if subgoal_is_passable else "subgoalImpassable")
            if subgoal_is_passable:
                need_to_plan = False

        if not need_to_plan:
            self._result = None, None, triggers, None, costmap
        else:
            w_start = np.array((pose.pose.position.x, pose.pose.position.y))
            is_contained_s, m_start = costmap.world_to_map(w_start)
            w_goal = np.array((goal.pose.position.x, goal.pose.position.y))
            is_contained_g, m_goal = costmap.world_to_map(w_goal)
            if not (is_contained_s and is_contained_g):
                rospy.logwarn("goal {} => {} (or start {} => {}) is not contained".format(m_goal, w_goal, m_start, w_start))

            # set start and goal
            self._impl.set_problem(m_start, m_goal)

            succeeded = False
            trajectory = None
            if self._impl.is_stuck():
                triggers.append("isStuck")
            elif self._impl.is_unavoidable():
                rospy.logdebug("add goal_lookahead_plus")
                self._goal_lookahead_plus += 1
                triggers.append("unavoidable")
            else:
                solution = self._impl.solve()
                if solution:
                    stetes, cost, states_np = solution
                    succeeded = True
                    trajectory = costmap.map_to_world(states_np)
                    subgoals_ret = WaypointGenerator.generate(trajectory, subgoal_distance=self._subgoal_distance)
                    global_plan = [(subgoal.pose.position.x, subgoal.pose.position.y) for subgoal in subgoals_ret]
                    triggers.append("avoidable")
                else:
                    triggers.append("unavoidable")
            rospy.logdebug(triggers)
            if not succeeded:
                subgoals_ret = None
                cost = None
            self._result = subgoals_ret, cost, triggers, trajectory, costmap
