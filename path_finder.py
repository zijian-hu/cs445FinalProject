import numpy as np
import math

from utils import get_dist


class PathFinder:
    def __init__(self):
        pass

    @staticmethod
    def get_spline_points(paths, num_points: int = 100):
        # draw all bezier curves
        paths_points = []
        paths_color = []
        for path in paths:
            ts = np.linspace(0, 1.0, num_points)
            result = np.empty((0, 3))
            for i in range(0, path.num_segments()):
                for t in ts[:-2]:
                    s = path.eval(i, t)
                    result = np.vstack([result, s])

            paths_points.append(result)
            paths_color.append(path.color)

        return paths_points, paths_color

    @staticmethod
    def find_path(lines, paths, paths_color):
        x = 0
        y = 0
        way_points_draw = np.empty((0, 3))
        open_list = np.arange(0, len(lines) + len(paths))
        prev_color = None

        for i in range(len(lines) + len(paths)):
            min_dist = float('inf')
            arg_min_dist = open_list[0]
            in_same_direction = False
            is_spline = False
            tie_list = np.empty((0, 2))

            for j in open_list:
                is_j_spline = j >= len(lines)
                if is_j_spline:
                    dist_u = get_dist(x, y, *paths[j - len(lines)][0][:2])
                    dist_v = get_dist(x, y, *paths[j - len(lines)][-1][:2])
                else:
                    dist_u = get_dist(x, y, *lines[j].u)
                    dist_v = get_dist(x, y, *lines[j].v)

                if min_dist > dist_u:
                    min_dist = dist_u
                    arg_min_dist = j
                    in_same_direction = True
                    is_spline = is_j_spline

                    if tie_list.size > 0:
                        tie_list = np.empty((0, 2))
                elif min_dist == dist_u:
                    # keep track of ties
                    tie_list = np.vstack([tie_list, [j, True]])

                if min_dist > dist_v:
                    min_dist = dist_v
                    arg_min_dist = j
                    in_same_direction = False
                    is_spline = is_j_spline

                    if tie_list.size > 0:
                        tie_list = np.empty((0, 2))
                elif min_dist == dist_v:
                    # keep track of ties
                    tie_list = np.vstack([tie_list, [j, False]])

            if tie_list.size > 0:
                # break ties

                if prev_color is not None:
                    same_color_filter = []

                    for tie_index in tie_list[:, 0]:
                        if tie_index >= len(lines):
                            curr_color = paths_color[tie_index - len(lines)]
                        else:
                            curr_color = lines[int(tie_index)].color

                        if curr_color == prev_color:
                            same_color_filter.append(int(tie_index))

                    if len(same_color_filter) > 0:
                        tie_list = tie_list[same_color_filter]

                arg_min_dist, in_same_direction = tie_list[0]
                arg_min_dist = int(arg_min_dist)
                is_spline = arg_min_dist >= len(lines)

            if is_spline:
                arg_min_dist -= len(lines)

                if in_same_direction:
                    x, y = paths[arg_min_dist][-1][:2]
                else:
                    x, y = paths[arg_min_dist][0][:2]

                open_list = open_list[open_list != (arg_min_dist + len(lines))]
                prev_color = paths_color[arg_min_dist]
            else:
                if in_same_direction:
                    x, y = lines[arg_min_dist].v
                else:
                    x, y = lines[arg_min_dist].u

                open_list = open_list[open_list != arg_min_dist]
                prev_color = lines[arg_min_dist]

            way_points_draw = np.vstack([way_points_draw, [arg_min_dist, in_same_direction, is_spline]])

        return way_points_draw
