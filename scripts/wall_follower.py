#!/usr/bin/env python3
"""
Wall Follower

Use the LIDAR (and RANSAC) to detect a wall, then drive parallel to it.

This script predates my understanding of how to use TF, and more broadly how to write good ROS code.
"""
import math
import numpy as np
import rospy
from typing import Optional, Tuple, List
from geometry_msgs.msg import Quaternion, Point, Vector3, Twist, Pose
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from random import choices

from helpers import State, Node, q_math, linear_regression


class WallFollower(State):
    def activate(self, node: Node):
        super().activate(node)

    def update(self):
        orientation = self.detect_wall_orientation()

        # RANSAC Failed
        if orientation is None:
            print("RANSAC Failed!")
            self.node.set_speed(0, 0)
            return

        diff = q_math.ijk_magnitude(
            q_math.difference(orientation, self.node.orientation)
        )

        self.node.set_speed(0.1, diff)

    def detect_wall_orientation(
        self,
        threshold: float = 0.01,
        min_matches: int = 20,
        points_per_attempt: int = 5,
        max_iters: int = 1000
    ) -> Optional[Quaternion]:
        """
        Perform a modified RANSAC against the laser scan data to detect the wall.

        This cuts a few corners compared to a true RANSAC, but it's plenty good for this use case.

        Default parameters were determined by experimentation.
        """
        ranges = self.node.laser_points

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
                angle = math.asin(m)
                orientation = q_math.tf_to_rospy(
                    quaternion_from_euler(0, 0, angle)
                )
                print("RANSAC, m = ", m, "angle =", angle * (180 / math.pi),)
                self.node.mark_target(
                    # Marker Option 1: Vector

                    # self.node.position,
                    # orientation=orientation,
                    # shape=Marker.ARROW,
                    # scale=(1, 0.1, 0.1),

                    # Marker Option 2: Line
                    [
                        Point(-10, (m * -10) + b, 0),
                        Point(20, (m * 20) + b, 0)
                    ],
                    shape=Marker.LINE_STRIP,
                    scale=(0.1, 0.1, 0.1),
                    color=(0, 1, 0, 0.25)
                )
                return orientation

        return None


if __name__ == '__main__':
    WallFollower().run()
