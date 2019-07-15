
class OdomListener(object):
    def __init__(self, local_planners, **kwargs):
        self._local_planners = local_planners

    def __call__(self, msg):
        for planner in self._local_planners.values():
            planner.update_twist(msg.twist.twist)
