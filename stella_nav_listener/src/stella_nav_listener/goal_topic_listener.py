from stella_nav_core.goal import Goal

class GoalTopicListener(object):
    def __init__(self, handlers, handler, **kwargs):
        self._goal_handler = handlers[handler]

    def __call__(self, msg):
        assert msg.header.frame_id == "map"
        self._goal_handler.add_goal(Goal.from_msg(msg))
