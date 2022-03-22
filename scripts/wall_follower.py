#!/usr/bin/env python3
from cmath import pi
from math import cos, radians, sin, inf
import numpy as np
import rospy
import ros_numpy.point_cloud2 as numpy_point_cloud2
from typing import Optional, Tuple, List
from geometry_msgs.msg import Quaternion, Point, Vector3, Twist, Pose
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from random import choices

from follow import make_marker
from helpers import State, Node


# Adapted from https://numpy.org/doc/stable/reference/generated/numpy.linalg.lstsq.html
def linear_regression(x: np.array, y: np.array) -> Tuple[float, float]:
    """ Conduct a least squares regression for y = m*x + b, returning m, b. """
    A = np.vstack([x, np.ones(len(x))]).T
    m, b = np.linalg.lstsq(A, y, rcond=None)[0]
    return m, b


class WallFollower(State):
    def __init__(self):
        self.range_pub = rospy.Publisher('/points', Marker, queue_size=10)

    def activate(self, node: Node):
        super().activate(node)

    def update(self):
        self.ransac()

    def ransac(
        self,
        threshold: float = 0.0005,
        min_matches: int = 20,
        points_per_attempt: int = 5,
        max_iters: int = 1000
    ):
        """
        Perform a modified RANSAC against the laser scan data to detect the wall.

        This cuts a few corners compared to a true RANSAC, but it's plenty good for this use case.

        Default parameters were determined by experimentation.
        """
        ranges = numpy_point_cloud2.pointcloud2_to_xyz_array(
            self.node.laser_data)

        for _ in range(max_iters):
            points = np.array(choices(ranges, k=points_per_attempt))

            m, b = linear_regression(points[:, 0], points[:, 1])

            matches = 0

            for x, y, z in ranges:
                y_predicted = (m * x) + b
                y_err = abs(y - y_predicted)
                if y_err <= threshold:
                    matches += 1

            if matches > min_matches:  # Success!
                self.node.mark_target(
                    [
                        Point(-10, (m * -10) + b, 0),
                        Point(20, (m * 20) + b, 0)
                    ],
                    shape=Marker.LINE_STRIP,
                    scale=(0.1, 0.1, 0.1),
                    color=(0, 1, 0, 0.25)
                )

                return


if __name__ == '__main__':
    WallFollower().run()
