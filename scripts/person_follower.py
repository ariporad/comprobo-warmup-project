#!/usr/bin/env python3
import math
import numpy as np
from typing import Optional
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from helpers import State, Node, q_math


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
        self.node.set_speed(
            self.params['forward_vel'],
            -heading_angle * self.params['angular_speed_coeff']
        )

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
            q_math.rospy_to_tf(self.node.orientation))[2]

        # FIXME: this is not how you subtract angles
        return cur_angle - angle


if __name__ == '__main__':
    PersonFollower().run()
