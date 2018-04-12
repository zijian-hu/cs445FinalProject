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

    def update(self, query=None):
        odometry_list = (self.odometry.x, self.odometry.y, self.odometry.theta)

        if query is None:
            self.x, self.y, self.theta = odometry_list
            return

        tracker_list = (
            query["position"]["x"],
            query["position"]["y"],
            query["orientation"]["y"]
        )

        self.x, self.y, self.theta = self.alpha * tracker_list + (1 - self.alpha) * odometry_list
