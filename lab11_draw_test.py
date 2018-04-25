"""
Example to use the pen holder
Use "python3 run.py --sim lab11_penholder_test" to execute
"""
from pyCreate2 import create2
import math
import numpy as np
import matplotlib.pyplot as plt
from lab11_image import Line
from lab11_image import BezierPath
from pid_controller import PIDController
from odometry import Odometry
from complementary_filter import ComplementaryFilter
from tracker import Tracker
from path_finder import PathFinder

import lab11_image


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """

        # read file and get all paths
        self.img = lab11_image.VectorImage("lab11_img1.yaml")

        # init objects
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.penholder = factory.create_pen_holder()
        self.tracker = Tracker(factory.create_tracker, 1, sd_x=0.01, sd_y=0.01, sd_theta=0.01, rate=10)
        self.servo = factory.create_servo()
        self.odometry = Odometry()

        # alpha for tracker
        self.alpha_x = 0.35
        self.alpha_y = self.alpha_x
        self.alpha_theta = 0.4

        # init controllers
        self.pidTheta = PIDController(400, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.filter = ComplementaryFilter(self.odometry, self.tracker, self.time, 0.4,
                                          (self.alpha_x, self.alpha_y, self.alpha_theta))

        # parameters
        self.base_speed = 100

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

        self.penholder.set_color(0.0, 1.0, 0.0)

        start_time = self.time.time()
        splines, splines_color = PathFinder.get_spline_points(self.img.paths)
        
        # in format [index, is_parallel, is_spline]
        line_index_list = PathFinder.find_path(self.img.lines, splines, splines_color)

        # we know there is only 1 spline for this project
        path_points = self.draw_path_coords(splines[0], False)
        self.penholder.set_color(*get_color(splines_color[0]))

        self.draw_graph()

        # go to start of the curve and begin drawing
        for i in range(0, 2):
            # go to start of curve
            goal_x, goal_y = path_points[0, 0], path_points[0, 1]
            print("=== GOAL SET === {:.3f}, {:.3f}".format(goal_x, goal_y))

            if i == 1:
                goal_x, goal_y = path_points[1, 0], path_points[1, 1]

            # turn to goal
            self.tracker.update()
            self.filter.update()
            curr_x = self.filter.x
            curr_y = self.filter.y

            goal_theta = math.atan2(goal_y - curr_y, goal_x - curr_x)
            if i == 1:
                goal_theta = math.atan2(goal_y - path_points[0, 1], goal_x - path_points[0, 0])

            # not drawing while turning0
            self.penholder.go_to(0.0)
            self.go_to_angle(goal_theta)
            self.go_to_goal(goal_x, goal_y)

        # start drawing
        self.penholder.go_to(-0.025)
        prev_base_speed = self.base_speed
        self.filter.updateFlag = False
        self.base_speed = 25
        print("Draw!")

        # draw the rest of the curve
        for i in range(2, len(path_points), 10):
            goal_x, goal_y = path_points[i, 0], path_points[i, 1]
            print("=== GOAL SET === {:.3f}, {:.3f}".format(goal_x, goal_y))
            self.go_to_goal(goal_x, goal_y, useOdo=False)

        # stop drawing
        self.base_speed = prev_base_speed
        self.filter.updateFlag = True
        self.penholder.go_to(0.0)
        # self.draw_graph()

        prev_color = None
        for draw_info in line_index_list:
            index = int(draw_info[0])
            is_parallel = draw_info[1]
            is_spline = draw_info[2]

            if is_spline:
                path_points = self.draw_path_coords(splines[index], is_parallel)
                curr_color = self.img.paths[index].color
                continue

            line = self.img.lines[index]
            curr_color = self.img.lines[index].color

            if curr_color != prev_color:
                prev_color = curr_color
                self.penholder.set_color(*get_color(curr_color))

            for i in range(0, 2):
                goal_x, goal_y = self.draw_coords(line, is_parallel=is_parallel, at_start=True)

                if i == 1:
                    goal_x, goal_y = self.draw_coords(line, is_parallel=is_parallel, at_start=False)

                print("=== GOAL SET === {:.3f}, {:.3f}".format(goal_x, goal_y))

                self.tracker.update()
                self.filter.update()
                curr_x = self.filter.x
                curr_y = self.filter.y

                goal_theta = math.atan2(goal_y - curr_y, goal_x - curr_x)

                # not drawing while turning
                self.penholder.go_to(0.0)
                self.go_to_angle(goal_theta)

                if i == 1:
                    # start drawing
                    self.penholder.go_to(-0.025)
                    print("Draw!")

                self.go_to_goal(goal_x, goal_y)

        self.draw_graph()
        self.create.stop()

    def drive(self, theta, distance, speed):
        # Sum all controllers and clamp
        self.create.drive_direct(max(min(int(theta + distance + speed), 500), -500),
                                 max(min(int(-theta + distance + speed), 500), -500))

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            self.update()

            t = self.time.time()
            if start + time_in_sec <= t:
                break

    # gives coordinates to draw the lines correctly
    # line: segment to be drawn
    # at_start: set true to retun the first coordinate, set false for the second coordinate
    # returns the x, y coordinates offset
    def draw_coords(self, line, at_start=True, is_parallel=True):
            if is_parallel:
                theta = math.atan2(line.v[1] - line.u[1], line.v[0] - line.u[0]) + math.pi / 2
                if at_start:
                    return math.cos(theta) * self.robot_marker_distance + line.u[0], \
                           math.sin(theta) * self.robot_marker_distance + line.u[1]
                else:
                    return math.cos(theta) * self.robot_marker_distance + line.v[0], \
                           math.sin(theta) * self.robot_marker_distance + line.v[1]
            else:
                theta = math.atan2(line.v[1] - line.u[1], line.v[0] - line.u[0]) - math.pi / 2
                if at_start:
                    return math.cos(theta) * self.robot_marker_distance + line.v[0], \
                           math.sin(theta) * self.robot_marker_distance + line.v[1]
                else:
                    return math.cos(theta) * self.robot_marker_distance + line.u[0], \
                           math.sin(theta) * self.robot_marker_distance + line.u[1]

    def draw_path_coords(self, result, is_parallel):
        final_result = np.empty((0, 2))
        if is_parallel:
            for i in range(0, len(result)-1):
                theta = math.atan2(result[i, 1] - result[i + 1, 1],
                                   result[i, 0] - result[i + 1, 0]) - math.pi/2
                s = math.cos(theta) * self.robot_marker_distance + result[i, 0], \
                    math.sin(theta) * self.robot_marker_distance + result[i, 1]
                final_result = np.vstack([final_result, s])
        else:
            for i in range(len(result)-1, 1, -1):
                theta = math.atan2(result[i, 1] - result[i - 1, 1],
                                   result[i, 0] - result[i - 1, 0]) - math.pi/2
                s = math.cos(theta) * self.robot_marker_distance + result[i, 0], \
                    math.sin(theta) * self.robot_marker_distance + result[i, 1]
                final_result = np.vstack([final_result, s])
        return final_result

    def go_to_goal(self, goal_x, goal_y, useOdo = False):
        while True:
            state = self.update()

            if state is not None:
                if useOdo:
                    curr_x = self.odometry.x
                    curr_y = self.odometry.y
                    curr_theta = self.odometry.theta
                else:
                    curr_x = self.filter.x
                    curr_y = self.filter.y
                    curr_theta = self.filter.theta

                distance = math.sqrt(math.pow(goal_x - curr_x, 2) + math.pow(goal_y - curr_y, 2))
                output_distance = self.pidDistance.update(0, distance, self.time.time())

                theta = math.atan2(goal_y - curr_y, goal_x - curr_x)
                output_theta = self.pidTheta.update(curr_theta, theta, self.time.time())

                print("goal x,y = {:.3f}, {:.3f}, x,y = {:.3f}, {:.3f}".format(
                    goal_x, goal_y, curr_x, curr_y))

                self.drive(output_theta, output_distance, self.base_speed)

                if distance < 0.05:
                    self.create.drive_direct(0, 0)
                    break

                self.sleep(0.01)

    def go_to_angle(self, goal_theta):
        curr_theta = self.filter.theta

        while abs(-math.degrees(math.atan2(math.sin(curr_theta - goal_theta),
                                           math.cos(curr_theta - goal_theta)))) > 8:
            curr_theta = self.filter.theta

            print("goal_theta = {:.2f}, theta = {:.2f}".format(math.degrees(goal_theta),
                                                               math.degrees(curr_theta)))
            output_theta = self.pidTheta.update(curr_theta, goal_theta, self.time.time())

            self.drive(output_theta, 0, 0)
            self.sleep(0.01)
        self.create.drive_direct(0, 0)

    # debug function. Draws robot paths
    def draw_graph(self):
        # show drawing progress after each line segment is drawn
        if self.debug_mode:
            if len(self.odo) is not 0 and len(self.actual) is not 0:
                x, y = zip(*self.odo)
                a, b = zip(*self.actual)
                plt.plot(x, y, color='red', label='Sensor path')
                plt.plot(a, b, color='green', label='Actual path', linewidth=1.4)
                self.odo = []
                self.actual = []

            ts = np.linspace(0, 1.0, 100)
            result = np.empty((0, 3))
            for i in range(0, self.img.paths[0].num_segments()):
                for t in ts[:-2]:
                    s = self.img.paths[0].eval(i, t)
                    result = np.vstack([result, s])

            plt.plot(result[:, 0], result[:, 1], self.img.paths[0].color)

            path_points = self.draw_path_coords(result, True)
            plt.plot(path_points[:, 0], path_points[:, 1], color='aqua')
            path_points = self.draw_path_coords(result, False)
            plt.plot(path_points[:, 0], path_points[:, 1], color='aqua')

            line_num = 0

            for line in self.img.lines:
                # draw lines
                plt.plot([line.u[0], line.v[0]], [line.u[1], line.v[1]], line.color)
                plt.annotate(s=line_num, xy=(line.v[0], line.v[1]), xytext=(line.u[0], line.u[1]),
                             arrowprops=dict(arrowstyle='-|>'))

                # draw paths
                theta = math.atan2(line.v[1] - line.u[1], line.v[0] - line.u[0]) + math.pi / 2
                plt.plot([math.cos(theta) * self.robot_marker_distance + line.u[0],
                          math.cos(theta) * self.robot_marker_distance + line.v[0]],
                         [math.sin(theta) * self.robot_marker_distance + line.u[1],
                          math.sin(theta) * self.robot_marker_distance + line.v[1]],
                         'aqua')

                theta = math.atan2(line.v[1] - line.u[1], line.v[0] - line.u[0]) - math.pi / 2
                plt.plot([math.cos(theta) * self.robot_marker_distance + line.u[0],
                          math.cos(theta) * self.robot_marker_distance + line.v[0]],
                         [math.sin(theta) * self.robot_marker_distance + line.u[1],
                          math.sin(theta) * self.robot_marker_distance + line.v[1]],
                         'aqua')

                line_num += 1
            plt.legend()
            plt.show()

    # updates odometry, filter, and tracker
    def update(self):
        state = self.create.update()
        self.filter.update()
        self.tracker.update()

        if state is not None:
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

            if self.debug_mode:
                self.odo.append((self.filter.x, self.filter.y))
                self.actual.append(
                    (self.create.sim_get_position()[0] - self.xi,
                     self.create.sim_get_position()[1] - self.yi))

        return state


def get_color(color):
    if color == "red":
        return 1.0, 0.0, 0.0
    elif color == "blue":
        return 0.0, 0.0, 1.0
    elif color == "green":
        return 0.0, 1.0, 0.0
    else:
        # black is default color
        return 0.0, 0.0, 0.0
