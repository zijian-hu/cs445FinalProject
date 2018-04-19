# https://codeshare.io/5wlweB
import numpy as np
import math

class ComplementaryFilter:
    def __init__(self, odometry, tracker, time, update_interval, alpha):
        self.alpha = np.array(alpha)
        self.odometry = odometry
        self.tracker = tracker
        self.update_interval = update_interval
        self.time = time

        # variables
        self.time_last_update = self.time.time()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self):
        # convert odometry angles to fit in the range -pi to pi
        odometry_converted = self.odometry.theta
        if self.odometry.theta > math.pi:
            odometry_converted = self.odometry.theta - 2*math.pi
        elif self.odometry.theta < -math.pi:
            odometry_converted = self.odometry.theta + 2*math.pi

        odometry_list = (self.odometry.x, self.odometry.y, odometry_converted)
        tracker_list = (self.tracker.x, self.tracker.y, self.tracker.theta)

        # update odometry in set intervals
        # TODO add low pass pass filtering
        if self.time.time() - self.time_last_update > self.update_interval:
            self.update_odometry()
            self.time_last_update = self.time.time()

        self.x, self.y, self.theta = self.alpha * tracker_list + (1 - self.alpha) * odometry_list

    def update_odometry(self):
        print('=== Odometry updated ===')
        self.odometry.x = self.tracker.x
        self.odometry.y = self.tracker.y
        self.odometry.theta = self.tracker.theta
