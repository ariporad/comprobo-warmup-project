"""
Helper methods for doing math with Point message objects.
"""
import math
from typing import List
from geometry_msgs.msg import Point


def add(*points: List[Point]) -> Point:
    """ a + b + ... + c """
    x, y, z = 0, 0, 0
    for point in points:
        x += point.x
        y += point.y
        z += point.z
    return Point(x, y, z)


def subtract(a: Point, b: Point) -> Point:
    """ a - b """
    return Point(
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
    )


def magnitude(point: Point) -> float:
    """ Distance from the origin to point. """
    return math.sqrt((point.x ** 2) + (point.y ** 2) + (point.z ** 2))


def distance(a: Point, b: Point) -> float:
    """ Distance between two points. """
    return magnitude(subtract(a, b))
