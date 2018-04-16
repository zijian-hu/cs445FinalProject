"""
Example to use the pen holder
Use "python3 run.py --sim lab11_penholder_test" to execute
"""
from pyCreate2 import create2
import math
import numpy as np
import matplotlib.pyplot as plt

from pid_controller import PIDController
from odometry import Odometry
from complementary_filter import ComplementaryFilter
from tracker import Tracker

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
        self.tracker = Tracker(factory.create_tracker, 1, sd_x=0.01, sd_y=0.01, sd_theta=0.01, rate=10)

        self.servo = factory.create_servo()
        self.odometry = Odometry()

        # alpha for tracker
        self.alpha_x = 0.6
        self.alpha_y = self.alpha_x
        self.alpha_theta = 0.6

        self.pidTheta = PIDController(500, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.filter = ComplementaryFilter(self.odometry, self.tracker,
                                          (self.alpha_x, self.alpha_y, self.alpha_theta))

        # constant
        self.robot_marker_distance = 0.1906

        # debug vars
        self.debug_mode = True
        self.odo = []
        self.actual = []
        self.xi = 0
        self.yi = 0
        self.init = True


    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        base_speed = 100
        self.penholder.set_color(0.0, 1.0, 0.0)
        start_time = self.time.time()
        last_check_time = start_time

        for line in self.img.lines:
            for i in range(0, 2):
                goal_x, goal_y = self.draw_coords(line, at_start=True)

                if i == 1:
                    goal_x, goal_y = self.draw_coords(line, at_start=False)

                print("=== GOAL SET === {:.3f}, {:.3f}".format(goal_x, goal_y))

                self.tracker.update()
                self.filter.update()
                curr_x = self.filter.x
                curr_y = self.filter.y

                goal_theta = math.atan2(goal_y - curr_y, goal_x - curr_x)

                # speed_multiplier = 1

                # not drawing while turning
                self.penholder.go_to(0.0)
                self.go_to_angle(goal_theta)

                if i == 1:
                    # start drawing
                    self.penholder.go_to(-0.025)
                    print("Draw!")

                while True:
                    state = self.create.update()
                    query = self.tracker.update()

                    if state is not None:

                        if self.debug_mode:
                            self.odo.append((self.odometry.x, self.odometry.y))
                            self.actual.append(
                                (self.create.sim_get_position()[0] - self.xi,
                                 self.create.sim_get_position()[1] - self.yi))

                        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                        self.tracker.update()
                        self.filter.update()
                        curr_x = self.filter.x
                        curr_y = self.filter.y
                        curr_theta = self.filter.theta

                        theta = math.atan2(math.sin(curr_theta), math.cos(curr_theta))
                        # print("[{},{},{}]".format(self.odometry.x, self.odometry.y,
                        #                           math.degrees(self.odometry.theta)))

                        # print("goal_theta = {}, theta = {}".format(goal_theta, theta))
                        print("goal x,y = {:.3f}, {:.3f}, x,y = {:.3f}, {:.3f}".format(
                            goal_x, goal_y, curr_x, curr_y))

                        # improved version 2: fuse with velocity controller
                        distance = math.sqrt(
                            math.pow(goal_x - curr_x, 2) + math.pow(goal_y - curr_y, 2))
                        output_distance = self.pidDistance.update(0, distance, self.time.time())
                        output_theta = self.pidTheta.update(theta, curr_theta, self.time.time())
                        self.create.drive_direct(int(base_speed + output_distance - output_theta),
                                                 int(base_speed + output_distance + output_theta))
                        if distance < 0.3:
                            break

                # draw graph after every line segment
                self.draw_graph()
                self.create.drive_direct(0, 0)
                self.sleep(0.01)

        self.create.stop()

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                if self.debug_mode:
                    self.odo.append((self.odometry.x, self.odometry.y))
                    self.actual.append(
                        (self.create.sim_get_position()[0] - self.xi,
                         self.create.sim_get_position()[1] - self.yi))

            self.tracker.update()

            t = self.time.time()
            if start + time_in_sec <= t:
                break

    # gives coordinates to draw the lines correctly
    # line: segment to be drawn
    # at_start: set true to retun the first coordinate, set false for the second coordinate
    # returns the x, y coordinates offset
    def draw_coords(self, line, at_start):
        # calculate angle of the line
        theta = math.atan2(line.v[1] - line.u[1], line.v[0] - line.u[0]) + math.pi/2

        if at_start:
            return math.cos(theta)*self.robot_marker_distance + line.u[0], math.sin(theta)*self.robot_marker_distance + line.u[1]
        else:
            return math.cos(theta)*self.robot_marker_distance + line.v[0], math.sin(theta)*self.robot_marker_distance + line.v[1]

    def go_to_angle(self, goal_theta):
        while math.fabs(math.atan2(
                math.sin(goal_theta - self.odometry.theta),
                math.cos(goal_theta - self.odometry.theta))) > 0.1:
            # print("Go TO: " + str(goal_theta) + " " + str(self.odometry.theta))
            print("goal_theta = {:.2f}, theta = {:.2f}".format(math.degrees(goal_theta),
                                                               math.degrees(self.odometry.theta)))
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)

    def draw_graph(self):
        # show drawing progress after each line segment is drawn
        if self.debug_mode:
            if len(self.odo) is not 0:
                x, y = zip(*self.odo)
                a, b = zip(*self.actual)
                plt.plot(x, y, color='red')
                plt.plot(a, b, color='green')

                for line in self.img.lines:
                    # draw lines
                    plt.plot([line.u[0], line.v[0]], [line.u[1], line.v[1]], line.color)

                    # draw paths
                    theta = math.atan2(line.v[1] - line.u[1], line.v[0] - line.u[0]) + math.pi / 2
                    plt.plot([math.cos(theta) * self.robot_marker_distance + line.u[0],
                              math.cos(theta) * self.robot_marker_distance + line.v[0]],
                             [math.sin(theta) * self.robot_marker_distance + line.u[1],
                              math.sin(theta) * self.robot_marker_distance + line.v[1]],
                             'lime')
            plt.show()