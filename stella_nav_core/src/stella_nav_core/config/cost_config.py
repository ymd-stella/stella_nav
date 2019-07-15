import numpy as np


class CostConfig(object):
    def __init__(self, gain, max_cost=None, min_cost=0.0):
        self._gain = gain
        if max_cost is None:
            self._max_cost = gain
        else:
            self._max_cost = max_cost
        self._min_cost = min_cost

    def get_cost(self, value):
        return np.clip(self._gain * value, self._min_cost, self._max_cost)
