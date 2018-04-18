import numpy as np
import matplotlib.pyplot as plt
import lab11_image
import math


def get_dist(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


img = lab11_image.VectorImage("lab11_img1.yaml")
len(img.lines)

x = 0
y = 0
way_points = []
open_list = np.arange(0, len(img.lines))
for i in range(len(img.lines)):
    min_dist = math.inf
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

    open_list = open_list[open_list != arg_min_dist]

way_points = np.array(way_points)
for i in np.arange(0, way_points.shape[0], 2):
    plt.plot([way_points[i, 0], way_points[i + 1, 0]], [way_points[i, 1], way_points[i + 1, 1]],
             img.lines[int(way_points[i + 1, 2])].color)

plt.show()
