"""
Example to use the pen holder
Use "python3 run.py --sim lab11_penholder_test" to execute
"""
from pyCreate2 import create2
import math
import numpy as np
import odometry
import pid_controller
import matplotlib
import argparse
import lab11_image


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """

        # read file and get all paths
        # self.img = lab11_image.VectorImage(input("Enter the image name: "))
        self.img = lab11_image.VectorImage("lab11_img1.yaml")

        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.penholder = factory.create_pen_holder()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()

        self.pidTheta = pid_controller.PIDController(500, 10, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(500, 5, 50, [0, 0], [-200, 200], is_angle=False)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        for line in self.img.lines:
            self.penholder.go_to(0.0)
            for i in range(0, 2):
                # draw if heading to endpoint
                if i == 1:
                    self.penholder.go_to(-0.025)
                goal_x = line.u[i]
                goal_y = line.v[i]
                base_speed = 100
                speed_multiplier = 1

                result = np.empty((0, 5))

                finish = True
                while finish:
                    state = self.create.update()
                    if state is not None:
                        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                        goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                        theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                        # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                        new_row = [self.time.time(), math.degrees(self.odometry.theta), math.degrees(goal_theta),
                                   self.odometry.x, self.odometry.y]
                        result = np.vstack([result, new_row])

                        output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                        #print("goal_theta = {}, theta = {}".format(goal_theta, theta))
                        #print("goal x,y = {}, {}, x,y = {}, {}\n".format(goal_x, goal_y, self.odometry.x, self.odometry.y))

                        # improved version 2: fuse with velocity controller
                        distance = math.sqrt(
                            math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                        output_distance = 2 * self.pidDistance.update(0, distance, self.time.time())
                        self.create.drive_direct(speed_multiplier * int(output_theta + output_distance),
                                                 speed_multiplier * int(-output_theta + output_distance))
                        if distance < 0.1:
                            finish = False

        self.create.stop()
