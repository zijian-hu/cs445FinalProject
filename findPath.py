import numpy as np
import matplotlib.pyplot as plt
import lab11_image
import math


def get_dist(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


img = lab11_image.VectorImage("lab11_img1.yaml")

lines = img.lines
paths = []
for path in img.paths:
    ts = np.linspace(0, 1.0, 100)
    result = np.empty((0, 3))
    for i in range(0, path.num_segments()):
        for t in ts[:-2]:
            s = path.eval(i, t)
            result = np.vstack([result, s])

    paths.append(result)


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

        if min_dist > dist_v:
            min_dist = dist_v
            arg_min_dist = j
            is_u = False
            is_spline = is_j_spline

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
