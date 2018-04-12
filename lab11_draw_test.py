"""
Example to use the pen holder
Use "python3 run.py --sim lab11_penholder_test" to execute
"""
from pyCreate2 import create2
import math
import numpy as np
import odometry
import pid_controller
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

        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        base_speed = 100

        for line in self.img.lines:
            for i in range(0, 2):
                goal_x = line.u[i]
                goal_y = line.v[i]
                goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)

                speed_multiplier = 1

                # not drawing while turning
                self.penholder.go_to(0.0)
                self.go_to_angle(goal_theta)

                print("goal x,y = {:.3f}, {:.3f}".format(goal_x, goal_y))
                if i == 1:
                    # start drawing
                    self.penholder.go_to(-0.025)
                    print("draw!")

                while True:
                    state = self.create.update()
                    if state is not None:
                        self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                        theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                        # print("[{},{},{}]".format(self.odometry.x, self.odometry.y,
                        #                           math.degrees(self.odometry.theta)))

                        # print("goal_theta = {}, theta = {}".format(goal_theta, theta))
                        print("goal x,y = {:.3f}, {:.3f}, x,y = {:.3f}, {:.3f}".format(
                            goal_x, goal_y, self.odometry.x, self.odometry.y))

                        # improved version 2: fuse with velocity controller
                        distance = math.sqrt(
                            math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                        output_distance = 2 * self.pidDistance.update(0, distance, self.time.time())
                        self.create.drive_direct(int(base_speed + output_distance), int(base_speed + output_distance))
                        if distance < 0.15:
                            break

                self.create.drive_direct(0, 0)
                self.sleep(0.01)
                print()

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
            t = self.time.time()
            if start + time_in_sec <= t:
                break

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
