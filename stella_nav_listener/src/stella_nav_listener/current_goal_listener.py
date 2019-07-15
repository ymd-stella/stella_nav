
class CurrentGoalListener(object):
    def __init__(self, handlers, observers, costmaps, **kwargs):
        self._update_goal_observers = [observers["force_recovery"]]
        self._costmaps = costmaps
        self._handlers = handlers

    def __call__(self, msg):
        msg = self._handlers["goal"].get_current_goal()
        for observer in self._update_goal_observers:
            observer.update_goal(msg)
        if "goal_costmap" in self._costmaps:
            origin = np.array((msg.pose.position.x, msg.pose.position.y))
            costmap = self._costmaps["goal_costmap"]
            costmap.update_origin(costmap.world_to_map_abs(origin))
