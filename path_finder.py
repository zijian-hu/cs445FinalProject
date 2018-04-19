import numpy as np
import math

from utils import get_dist


class PathFinder:
    def __init__(self):
        pass

    @staticmethod
    def find_path(img):
        x = 0
        y = 0
        way_points = []
        way_points_draw = []
        open_list = np.arange(0, len(img.lines))
        for i in range(len(img.lines)):
            min_dist = float('inf')
            arg_min_dist = open_list[0]
            is_u = False

            for j in open_list:
                dist_u = get_dist(x, y, *img.lines[j].u)
                if min_dist > dist_u:
                    min_dist = dist_u
                    arg_min_dist = j
                    is_u = True

                dist_v = get_dist(x, y, *img.lines[j].v)
                if min_dist > dist_v:
                    min_dist = dist_v
                    arg_min_dist = j
                    is_u = False

            if is_u:
                x, y = img.lines[arg_min_dist].v
                way_points.append([
                    img.lines[arg_min_dist].u[0], img.lines[arg_min_dist].u[1],
                    arg_min_dist
                ])
                way_points.append([
                    img.lines[arg_min_dist].v[0], img.lines[arg_min_dist].v[1],
                    arg_min_dist
                ])
            else:
                x, y = img.lines[arg_min_dist].u
                way_points.append([
                    img.lines[arg_min_dist].v[0], img.lines[arg_min_dist].v[1],
                    arg_min_dist
                ])
                way_points.append([
                    img.lines[arg_min_dist].u[0], img.lines[arg_min_dist].u[1],
                    arg_min_dist
                ])

            way_points_draw.append([arg_min_dist, is_u])
            open_list = open_list[open_list != arg_min_dist]

        return np.array(way_points_draw)
