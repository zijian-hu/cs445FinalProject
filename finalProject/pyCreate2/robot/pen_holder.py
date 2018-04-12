"""
Module to control a pen holder.
"""

from . import Servo
import math


class PenHolder:
    def __init__(self, number):
        self.servo = Servo(number)

    def go_to(self, height):
        """Go to specified target height.

        Args:
            height (float): target height in cm
        """
        self.servo.go_to(height * 1000)

    def set_color(self, r, g, b):
        """Set pen color (RGB).

        Args:
            r (float): red component (0 to 1)
            g (float): green component (0 to 1)
            b (float): blue component (0 to 1)
        """
        input('Press [Enter] after you changed the pen color to ({},{},{})'.format(r, g, b))
