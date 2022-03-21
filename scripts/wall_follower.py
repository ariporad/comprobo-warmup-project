#!/usr/bin/env python3
from math import cos, radians, sin, inf
import numpy as np
from typing import Optional, Tuple, List
from geometry_msgs.msg import Quaternion, Point, Vector3, Twist, Pose
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from random import choices

from follow import make_marker
from helpers import State, Node


class WallFollower(State):
    def __init__(self):
        pass

    def activate(self, node: Node):
        super().activate(node)

    def update(self):
        print("Updating...")
        self.ransac()

    @property
    def laser_ranges_cart(self) -> List[Tuple[float, float]]:
        raw_ranges = self.node.laser_ranges[:-1]
        ranges: list[Tuple[float, float]] = zip(
            [float(x) for x in range(len(raw_ranges))],
            raw_ranges
        )

        return [
            (
                -rho * cos(radians(phi)),
                -rho * sin(radians(phi)),
            )
            for phi, rho in ranges
            if rho != inf
        ]

    def ransac(self, threshold: float = 0.01, min_matches: int = 50, max_iters: int = 1000):
        print("RANSAC1")
        ranges = self.laser_ranges_cart

        print("RANSAC2")
        for _ in range(max_iters):
            points = choices(ranges, k=2)
            print(
                (points[1][1] - points[0][1]),
                (points[1][0] - points[0][0])
            )
            try:
                m = (points[1][1] - points[0][1]) / \
                    (points[1][0] - points[0][0])
            except ZeroDivisionError:
                continue

            b = (-m * points[0][0]) + points[0][1]

            matches = 0

            for x, y in ranges:
                y_predicted = (m * x) + b
                y_err = abs(y - y_predicted)
                if y_err <= threshold:
                    matches += 1

            if matches > min_matches:
                print("RANSACED!")
                marker = make_marker(
                    Point(0, 0, 0),
                    shape=Marker.LINE_STRIP,
                    scale=(0.1, 0.1, 0.1),
                    frame_id='laser_link'
                )
                marker.points = [
                    Point(-10, (m * -10) + b, 0),
                    Point(20, (m * 20) + b, 0)
                ]
                self.node.target_pub.publish(marker)
                return


if __name__ == '__main__':
    WallFollower().run()
