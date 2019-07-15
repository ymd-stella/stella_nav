import numpy as np

class StopPlanner(object):
    def __init__(self, *args, **kwargs):
        pass

    def update_twist(self, twist):
        pass

    def plan(self, pose, goal):
        return np.array((0.0, 0.0)), np.array(((0.0, 0.0, 0.0),))
