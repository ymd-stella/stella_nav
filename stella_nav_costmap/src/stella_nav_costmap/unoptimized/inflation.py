import numpy as np


def calc_inflation_as_image(
        background_cells,
        start_i,
        min_i,
        max_i,
        start_j,
        min_j,
        max_j,
        mask_lethal,
        mask_distance,
        distance_array,
        inflation_cost,
        radius_px,
        lethal_radius_px):
    for i in range(start_i.shape[0]):
        mask_i = mask_distance[start_i[i, 0]:max_i[i, 0] - min_i[i, 0] + start_i[i, 0], start_j[i, 0]:max_j[i, 0] - min_j[i, 0] + start_j[i, 0]]
        if inflation_cost is None:
            di = max_i[i, 0] - min_i[i, 0]
            dj = max_j[i, 0] - min_j[i, 0]
            if min_i[i, 0] == 0:
                dmin_i = distance_array.shape[0] - di
                dmax_i = distance_array.shape[0]
            else:
                dmin_i = 0
                dmax_i = di
            if min_j[i, 0] == 0:
                dmin_j = distance_array.shape[1] - dj
                dmax_j = distance_array.shape[1]
            else:
                dmin_j = 0
                dmax_j = dj
            background_cells[min_i[i, 0]:max_i[i, 0], min_j[i, 0]:max_j[i, 0]][mask_i] = np.maximum(
                (radius_px - distance_array[dmin_i:dmax_i, dmin_j:dmax_j][mask_i]) / (radius_px - lethal_radius_px),
                background_cells[min_i[i, 0]:max_i[i, 0], min_j[i, 0]:max_j[i, 0]][mask_i])
        else:
            background_cells[min_i[i, 0]:max_i[i, 0], min_j[i, 0]:max_j[i, 0]][mask_i] = inflation_cost
    for i in range(start_i.shape[0]):
        mask_i = mask_lethal[start_i[i, 0]:max_i[i, 0] - min_i[i, 0] + start_i[i, 0], start_j[i, 0]:max_j[i, 0] - min_j[i, 0] + start_j[i, 0]]
        background_cells[min_i[i, 0]:max_i[i, 0], min_j[i, 0]:max_j[i, 0]][mask_i] = 1.0
    return background_cells
