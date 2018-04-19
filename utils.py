import numpy as np


def clamp(value, range_min, range_max):
    return max(min(value, range_max), range_min)


def get_dist(x1, y1, x2, y2):
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
