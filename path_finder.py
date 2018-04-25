import numpy as np
import math

from utils import get_dist


class PathFinder:
    def __init__(self):
        pass

    @staticmethod
    def get_spline_points(paths, num_points: int = 100):
        # draw all bezier curves
        curves = []
        for path in paths:
            ts = np.linspace(0, 1.0, num_points)
            result = np.empty((0, 3))
            for i in range(0, path.num_segments()):
                for t in ts[:-2]:
                    s = path.eval(i, t)
                    result = np.vstack([result, s])

            curves.append(result)

        return curves

    @staticmethod
    def find_path(lines, paths):
        x = 0
        y = 0
        way_points_draw = np.empty((0, 3))
        open_list = np.arange(0, len(lines) + len(paths))
        for i in range(len(lines) + len(paths)):
            min_dist = float('inf')
            arg_min_dist = open_list[0]
            is_u = False
            is_spline = False

            for j in open_list:
                is_curr_spline = j >= len(lines)
                if is_curr_spline:
                    dist_u = get_dist(x, y, *paths[j - len(lines)][0][:2])
                    dist_v = get_dist(x, y, *paths[j - len(lines)][-1][:2])
                else:
                    dist_u = get_dist(x, y, *lines[j].u)
                    dist_v = get_dist(x, y, *lines[j].v)

                if min_dist > dist_u:
                    min_dist = dist_u
                    arg_min_dist = j
                    is_u = True
                    is_spline = is_curr_spline

                if min_dist > dist_v:
                    min_dist = dist_v
                    arg_min_dist = j
                    is_u = False
                    is_spline = is_curr_spline

            if is_spline:
                arg_min_dist -= len(lines)

                if is_u:
                    x, y = paths[arg_min_dist][-1][:2]
                else:
                    x, y = paths[arg_min_dist][0][:2]

                way_points_draw = np.vstack([way_points_draw, [arg_min_dist, is_u, is_spline]])
                open_list = open_list[open_list != (arg_min_dist + len(lines))]
            else:
                if is_u:
                    x, y = lines[arg_min_dist].v
                else:
                    x, y = lines[arg_min_dist].u

                way_points_draw = np.vstack([way_points_draw, [arg_min_dist, is_u, is_spline]])
                open_list = open_list[open_list != arg_min_dist]

        return way_points_draw
