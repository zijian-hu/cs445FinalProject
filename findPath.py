import numpy as np
import matplotlib.pyplot as plt
import lab11_image
import math


def get_dist(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def break_tie(tie_list_, prev_color_):
    if prev_color_ is not None:
        same_color_filter_ = []

        for tie_index in tie_list_[:, 0]:
            if tie_index >= len(lines):
                curr_color = paths_color[tie_index - len(lines)]
            else:
                curr_color = lines[tie_index].color

            if curr_color == prev_color_:
                same_color_filter_.append(tie_index)

        if len(same_color_filter_) > 0:
            tie_list_ = tie_list_[same_color_filter_]

    return tie_list_[0]


img = lab11_image.VectorImage("lab11_img1.yaml")

lines = img.lines
paths = []
paths_color = []
for path in img.paths:
    ts = np.linspace(0, 1.0, 100)
    result = np.empty((0, 3))
    for i in range(0, path.num_segments()):
        for t in ts[:-2]:
            s = path.eval(i, t)
            result = np.vstack([result, s])

    paths.append(result)
    paths_color.append(path.color)


x = 0
y = 0
way_points_draw = np.empty((0, 3))
open_list = np.arange(0, len(lines) + len(paths))
prev_color = None

for i in range(len(lines) + len(paths)):
    min_dist = float('inf')
    arg_min_dist = open_list[0]
    is_u = False
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
            is_u = True
            is_spline = is_j_spline

            if tie_list.size > 0:
                tie_list = np.empty((0, 2))
        elif min_dist == dist_u:
            tie_list = np.vstack([tie_list, [j, True]])

        if min_dist > dist_v:
            min_dist = dist_v
            arg_min_dist = j
            is_u = False
            is_spline = is_j_spline

            if tie_list.size > 0:
                tie_list = np.empty((0, 2))
        elif min_dist == dist_v:
            tie_list = np.vstack([tie_list, [j, False]])

    if tie_list.size > 0:
        print("tie! <- {}".format(tie_list))

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

        arg_min_dist, is_u = tie_list[0]
        arg_min_dist = int(arg_min_dist)
        is_spline = arg_min_dist >= len(lines)

    if is_spline:
        arg_min_dist -= len(lines)

        if is_u:
            x, y = paths[arg_min_dist][-1][:2]
        else:
            x, y = paths[arg_min_dist][0][:2]

        open_list = open_list[open_list != (arg_min_dist + len(lines))]
        prev_color = paths_color[arg_min_dist]
    else:
        if is_u:
            x, y = lines[arg_min_dist].v
        else:
            x, y = lines[arg_min_dist].u

        open_list = open_list[open_list != arg_min_dist]
        prev_color = lines[arg_min_dist]

    way_points_draw = np.vstack([way_points_draw, [arg_min_dist, is_u, is_spline]])


way_points = np.empty((0, 2))
for draw_info in way_points_draw:
    is_spline = draw_info[2]
    is_u = draw_info[1]
    index = draw_info[0]

    if is_spline:
        spline_points = paths[int(index)]
        if not is_u:
            spline_points = reversed(spline_points)

        for point in spline_points:
            way_points = np.vstack([way_points, [point[0], point[1]]])
    else:
        line = lines[int(index)]
        if not is_u:
            way_points = np.vstack([way_points, [line.v[0], line.v[1]]])
            way_points = np.vstack([way_points, [line.u[0], line.u[1]]])
        else:
            way_points = np.vstack([way_points, [line.u[0], line.u[1]]])
            way_points = np.vstack([way_points, [line.v[0], line.v[1]]])


plt.plot(way_points[:, 0], way_points[:, 1])
plt.show()
