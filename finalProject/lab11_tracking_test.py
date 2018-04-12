"""
Example code for the external tracking.
Use "python3 run.py --sim lab11_tracking_test" to execute
"""

import math


class Run:
    def __init__(self, factory):
        self.time = factory.create_time_helper()
        # sd_{x,y,theta} and rate are only for simulation to change the noise and update rate respectively.
        # They are ignored on the robot.
        self.tracker = factory.create_tracker(1, sd_x=0.01, sd_y=0.01, sd_theta=0.01, rate=10)

    def run(self):
        while True:
            r = self.tracker.query()
            if r is not None:
                x = r["position"]["x"]
                y = r["position"]["y"]
                z = r["position"]["z"]
                yaw = r["orientation"]["y"]
                print(x,y,z,math.degrees(yaw))

            self.time.sleep(0.0)
