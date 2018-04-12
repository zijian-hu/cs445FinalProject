# https://codeshare.io/5wlweB
import numpy as np


class ComplementaryFilter:
    def __init__(self, odometry, tracker, alpha):
        self.alpha = np.array(alpha)
        self.odometry = odometry
        self.tracker = tracker

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self):
        odometry_list = (self.odometry.x, self.odometry.y, self.odometry.theta)
        tracker_list = (self.tracker.x, self.tracker.y, self.tracker.theta)

        self.x, self.y, self.theta = self.alpha * odometry_list + (1 - self.alpha) * tracker_list
