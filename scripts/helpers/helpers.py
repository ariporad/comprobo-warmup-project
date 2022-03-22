#!/usr/bin/env python3
from __future__ import annotations
import numpy as np
import rospy
import math
import tf
import ros_numpy.point_cloud2 as np_point_cloud2
from math import pi
from typing import Optional, List, Type, Union, Iterable, Tuple
from std_srvs.srv import Empty
from geometry_msgs.msg import Quaternion, Point, Vector3, Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2

from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler, quaternion_conjugate


def linear_regression(x: np.array, y: np.array) -> Tuple[float, float]:
    """
    Conduct a least squares regression for y = m*x + b, returning m, b.

    Adapted from https://numpy.org/doc/stable/reference/generated/numpy.linalg.lstsq.html
    """
    A = np.vstack([x, np.ones(len(x))]).T
    m, b = np.linalg.lstsq(A, y, rcond=None)[0]
    return m, b


def reset_gazebo(wait: bool = True):
    # Reset Gazebo
    rospy.wait_for_service('/gazebo/reset_world')
    rospy.ServiceProxy('/gazebo/reset_world', Empty)()

    # Let it settle
    if wait:
        rospy.sleep(3)


def make_marker(point: Union[Point, Iterable[Point]],
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
    Generate a Marker message, with some sane defaults.

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


class QuaternionMath:
    @staticmethod
    def multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
        """ q1 * q2 """
        return QuaternionMath.tf_to_rospy(
            quaternion_multiply(
                QuaternionMath.rospy_to_tf(q1),
                QuaternionMath.rospy_to_tf(q2)
            )
        )

    @staticmethod
    def difference(q1: Quaternion, q2: Quaternion) -> Quaternion:
        """ q2 = x * q1, returns x """
        return QuaternionMath.tf_to_rospy(
            quaternion_multiply(
                QuaternionMath.rospy_to_tf(q2),
                QuaternionMath.rospy_to_tf(QuaternionMath.inverse(q1))
            )
        )

    @staticmethod
    def inverse(q: Quaternion) -> Quaternion:
        return Quaternion(q.x, q.y, q.z, -q.w)

    @staticmethod
    def ijk_magnitude(q: Quaternion) -> Quaternion:
        return math.sqrt(
            (q.x ** 2) +
            (q.y ** 2) +
            (q.z ** 2)
        )

    @staticmethod
    def rospy_to_tf(q: Quaternion) -> np.array:
        if isinstance(q, list):
            q = np.array(q)
        if isinstance(q, np.ndarray):
            return q
        return np.array([q.x, q.y, q.z, q.w])

    @staticmethod
    def tf_to_rospy(q: np.array) -> Quaternion:
        if isinstance(q, Quaternion):
            return q
        return Quaternion(q[0], q[1], q[2], q[3])


class PointMath:
    @staticmethod
    def add(*points: List[Point]) -> Point:
        """ a + b + ... + c """
        x, y, z = 0, 0, 0
        for point in points:
            x += point.x
            y += point.y
            z += point.z
        return Point(x, y, z)

    @staticmethod
    def subtract(a: Point, b: Point) -> Point:
        """ a - b """
        return Point(
            a.x - b.x,
            a.y - b.y,
            a.z - b.z
        )

    @staticmethod
    def magnitude(point: Point) -> float:
        return math.sqrt((point.x ** 2) + (point.y ** 2) + (point.z ** 2))

    @staticmethod
    def distance(a: Point, b: Point) -> float:
        return PointMath.magnitude(PointMath.subtract(a, b))
