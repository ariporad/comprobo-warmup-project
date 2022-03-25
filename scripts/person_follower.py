#!/usr/bin/env python3
from email.errors import NonASCIILocalPartDefect
import math
import numpy as np
import rospy
from typing import Optional, Tuple, List
from geometry_msgs.msg import Quaternion, Point, Vector3, Twist, Pose
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from random import choices

from helpers import State, Node, QuaternionMath, linear_regression


def clip(min: float, max: float, val: float) -> float:
    if val < min:
        return min
    elif val > max:
        return max
    else:
        return val


class PersonFollower(State):
    default_params = {
        # Minimum depths prevents running into things
        'min_depth': 0.1,
        'max_depth': 2,
        'width': 2,

        'forward_vel': 0.1,
        'angular_speed_coeff': 3,
    }

    def activate(self, node: Node):
        super().activate(node)

    def update(self):
        target = self.detect_target()

        if target is None:
            self.node.set_speed(0, 0)
            return

        self.node.mark_target(Point(*target), scale=(0.25, 0.25, 0.25))

        heading_angle = self.calculate_heading_angle(target)
        self.node.set_speed(0.1, -heading_angle * self.params)

    def detect_target(self) -> Optional[Point]:
        """
        Detect the center position of the target.

        This is currently the geometric mean of all laser scan data in the x-axis range
        [min_depth, max_depth] relative to the robot, and the y-axis range [-width / 2, width / 2]
        (also relative to the robot).

        Returns None if no target can be found.
        """
        ranges = self.node.laser_points

        # Ranges relative to robot
        # This could probably be accomplished with TF instead
        ranges_rel = ranges - np.array([
            self.node.position.x,
            self.node.position.y,
            self.node.position.z
        ])

        relevant_points = ranges[
            (ranges_rel[:, 0] >= self.params['min_depth']) &
            (ranges_rel[:, 0] <= self.params['max_depth']) &
            (np.abs(ranges_rel)[:, 1] <= self.params['width'] / 2)
        ]

        if len(relevant_points) == 0:
            return None
        else:
            return np.mean(relevant_points, axis=0)

    def calculate_heading_angle(self, target: Point) -> float:
        angle = math.atan2(target[1], target[0])

        cur_angle = euler_from_quaternion(
            QuaternionMath.rospy_to_tf(self.node.orientation))[2]

        # FIXME: this is not how you subtract angles
        return cur_angle - angle


if __name__ == '__main__':
    PersonFollower().run()
