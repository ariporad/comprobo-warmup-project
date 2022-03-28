"""
Miscellaneous helper methods
"""
from __future__ import annotations
import numpy as np
import rospy
from typing import Union, Iterable, Tuple
from std_srvs.srv import Empty
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker


def linear_regression(x: np.array, y: np.array) -> Tuple[float, float]:
    """
    Conduct a least squares regression for y = m*x + b, returning m, b.

    Adapted from https://numpy.org/doc/stable/reference/generated/numpy.linalg.lstsq.html
    """
    A = np.vstack([x, np.ones(len(x))]).T
    m, b = np.linalg.lstsq(A, y, rcond=None)[0]
    return m, b


def make_marker(point: Union[Point, Iterable[Point]] = Point(0, 0, 0),
                orientation: Quaternion = Quaternion(0, 0, 0, 1),
                id: int = 0,
                ns: str = "aporad",
                shape=Marker.SPHERE,
                action=Marker.ADD,
                scale=(1.0, 0.1, 0.1),
                color=(0.0, 1.0, 0.0, 1.0),
                lifetime=1,
                frame_id: str = 'odom'):
    """
    Generate a Marker message, with some reasonable defaults.

    Adapted from: https://comprobo20.github.io/Sample_code/marker_sample
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = shape
    marker.action = action
    marker.pose.orientation = orientation
    if isinstance(point, Point):
        marker.pose.position = point
    else:
        marker.pose.position = Point(0, 0, 0)
        marker.points = point

    marker.lifetime = rospy.Duration(lifetime)

    if isinstance(scale, int) or isinstance(scale, float):
        scale = (scale, scale, scale)
    x, y, z = scale
    marker.scale.x = x
    marker.scale.y = y
    marker.scale.z = z

    r, g, b, a = color
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a

    return marker


def clip(min: float, max: float, val: float) -> float:
    if val < min:
        return min
    elif val > max:
        return max
    else:
        return val
