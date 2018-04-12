"""
Tool to plot a given vector graphics file (YAML).

Run "python3 lab11_plot.py lab11_img1.yaml" to plot the
bezier curves and lines defined in lab11_img1.yaml.
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
import math

import lab11_image

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("image", help="vector image file to plot (YAML)")
    args = parser.parse_args()

    # read file
    img = lab11_image.VectorImage(args.image)

    # setup figure
    fig = plt.figure()
    ax = fig.add_subplot(111,aspect='equal')

    # draw all bezier curves
    for path in img.paths:
        ts = np.linspace(0, 1.0, 100)
        result = np.empty((0,3))
        for i in range(0, path.num_segments()):
            for t in ts[:-2]:
                s = path.eval(i, t)
                result = np.vstack([result, s])

        ax.plot(result[:,0], result[:,1], path.color)

    # draw lines
    for line in img.lines:
        plt.plot([line.u[0], line.v[0]], [line.u[1], line.v[1]], line.color)

    # show the output
    plt.show()
