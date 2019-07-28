import numpy as np
import rclpy
import threading
from .optimized.inflation import calc_inflation_as_image
from stella_nav_core.stella_nav_node import get_node


class GridMap2D(object):
    def __init__(self, size=(100, 100), resolution=10, default_value=0.0, inflation_lethal_radius_m=0.10, inflation_distance_radius_m=0.25,
                 origin=np.array((0., 0.)), cells=None, inflation_cost=None):
        self._default_value = default_value
        self._inflation_lethal_radius_px = int(round(float(inflation_lethal_radius_m) / resolution))
        self._inflation_distance_radius_px = int(round(float(inflation_distance_radius_m) / resolution))
        self._inflation_cost = inflation_cost
        if cells is None:
            self.cells = np.ones(size) * default_value
        else:
            self.cells = cells
        self.shape = self.cells.shape
        self.resolution = resolution
        self.origin = origin
        self.stamp = get_node().get_clock().now().to_msg()
        self.lock = threading.RLock()

    def clone(self):
        return GridMap2D(
            default_value=self._default_value,
            inflation_lethal_radius_m=self._inflation_lethal_radius_px,
            inflation_distance_radius_m=self._inflation_distance_radius_px,
            inflation_cost=self._inflation_cost,
            resolution=self.resolution,
            origin=self.origin,
            cells=self.cells)

    def update_origin(self, center):
        self.lock.acquire()
        self.origin = center - np.array(self.cells.shape) / 2.0
        self.lock.release()

    def is_contained(self, m):
        return np.all(m >= 0, axis=m.ndim - 1) * np.all(m < self.cells.shape, axis=m.ndim - 1)

    def world_to_map(self, w):
        m = ((np.flip(w, w.ndim - 1) / self.resolution) - self.origin).astype(int)
        return self.is_contained(m), m

    def world_to_map_abs(self, w):
        m = (np.flip(w, w.ndim - 1) / self.resolution).astype(int)
        return m

    def map_to_world(self, m):
        return (self.origin[::-1] + np.flip(m, m.ndim - 1) + 0.5) * self.resolution

    def write_from_world(self, w, stamp):
        is_contained, m = self.world_to_map(w)
        self.write(m[is_contained], stamp)

    def _calc_inflation(self, m, radius_px, lethal_radius_px):
        shape = self.shape
        search_range = np.arange(-radius_px, radius_px + 1)
        iv, jv = np.meshgrid(search_range, search_range)
        distance_array = np.hypot(iv, jv)
        mask_lethal = distance_array < lethal_radius_px
        mask_distance = distance_array < radius_px
        min_i_original = m[:, 0:1] - radius_px
        min_i = np.clip(min_i_original, 0, None)
        start_i = min_i - min_i_original
        max_i = np.min(np.hstack((np.full(m.shape[0], shape[0], dtype=np.int32).reshape(m.shape[0], 1), m[:, 0:1] + radius_px)), axis=1)
        max_i = max_i.reshape((max_i.shape[0], 1))
        min_j_original = m[:, 1:2] - radius_px
        min_j = np.clip(min_j_original, 0, None)
        start_j = min_j - min_j_original
        max_j = np.min(np.hstack((np.full(m.shape[0], shape[1], dtype=np.int32).reshape(m.shape[0], 1), m[:, 1:2] + radius_px)), axis=1)
        max_j = max_j.reshape((max_j.shape[0], 1))
        return start_i, min_i, max_i, start_j, min_j, max_j, mask_lethal, mask_distance, distance_array

    def write(self, m, stamp):
        inflation_cost = self._inflation_cost
        radius_px = self._inflation_distance_radius_px
        lethal_radius_px = self._inflation_lethal_radius_px

        start_i, min_i, max_i, start_j, min_j, max_j, mask_lethal, mask_distance, distance_array = self._calc_inflation(m, radius_px, lethal_radius_px)

        background_cells = np.full(self.cells.shape, self._default_value)
        background_cells = calc_inflation_as_image(
            background_cells,
            start_i, min_i, max_i, start_j, min_j, max_j,
            mask_lethal, mask_distance, distance_array,
            inflation_cost, radius_px, lethal_radius_px)

        self.lock.acquire()
        self.stamp = stamp
        self.cells = background_cells
        self.lock.release()

    def write_value(self, m, value):
        self.cells[tuple(m.T.tolist())] = value

    def write_value_from_world(self, w, value):
        is_contained, m = self.world_to_map(w)
        self.write_value(m[is_contained], value)

    def get_value_from_world(self, w):
        is_contained, m = self.world_to_map(w)
        result = np.full(is_contained.shape, self._default_value)
        result[is_contained] = self.cells[tuple(m[is_contained, :].T.tolist())]
        return result

    def get_value(self, m):
        return self.cells[tuple(m.T.tolist())]

    @staticmethod
    def load(msg):
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin = msg.info.origin
        data = np.array(msg.data).reshape(height, width)
        return GridMap2D(
            size=(height, width),
            resolution=resolution,
            default_value=0.0,
            origin=np.array((origin.position.x / resolution, origin.position.y / resolution)),
            cells=data
        )
