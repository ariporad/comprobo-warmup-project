#!/usr/bin/env python3
from __future__ import annotations
import numpy
import rospy
import math
from math import pi
from typing import Optional, List, Type
from std_srvs.srv import Empty
from geometry_msgs.msg import Quaternion, Point, Vector3, Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler, quaternion_conjugate


# Adapted from: https://comprobo20.github.io/Sample_code/marker_sample
def make_marker(position: Point,
                orientation: Quaternion = Quaternion(0, 0, 0, 1),
                id: int = 0,
                ns: str = "aporad",
                shape=Marker.SPHERE,
                action=Marker.ADD,
                scale=(1.0, 0.1, 0.1),
                color=(0.0, 1.0, 0.0, 1.0),
                frame_id: str = 'odom'):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = id
    marker.type = shape
    marker.action = action
    marker.pose.position = position
    marker.pose.orientation = orientation

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
        """ q2 = x * q1, returns x"""
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
    def rospy_to_tf(q: Quaternion) -> numpy.array:
        if isinstance(q, list):
            q = numpy.array(q)
        if isinstance(q, numpy.ndarray):
            return q
        return numpy.array([q.x, q.y, q.z, q.w])

    @staticmethod
    def tf_to_rospy(q: numpy.array) -> Quaternion:
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


class State:
    node: Optional[Node] = None

    def activate(self, node: Node):
        self.node = node

    def deactivate(self):
        self.node = None

    def update(self):
        raise NotImplementedError('Override me!')

    def run(self, name: Optional[str] = None, gazebo_reset: bool = False):
        Node.run_node(
            self,
            name=name or self.__class__.__name__,
            gazebo_reset=gazebo_reset
        )


class WaitingForData(Exception):
    """ Early exit exception for when data hasn't been received yet. """


class Node:
    active_state: State

    _odom: Optional[Odometry] = None
    _scan: Optional[LaserScan] = None

    velocity_pub: rospy.Publisher
    target_pub: rospy.Publisher

    def __init__(self, starting_state: State, name: str):
        rospy.init_node(name)
        self.active_state = starting_state
        self.active_state.activate(self)

        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.target_pub = rospy.Publisher('/target', Marker, queue_size=10)

        self._subscribe('/odom', Odometry, '_odom')
        self._subscribe('/stable_scan', LaserScan, '_scan')

    def _subscribe(self, topic: str, msg_type: Type, attr_name: str, *args, **kwargs):
        def _handler(msg):
            self.__setattr__(attr_name, msg)
            self.update()
        rospy.Subscriber(topic, msg_type, _handler, *args, **kwargs)

    def update(self):
        try:
            self.active_state.update()
        except WaitingForData:
            # If we're waiting for data, then we can't update yet (but it isn't an error!)
            pass

    def transition(self, to_node):
        print("Transitioning to:", to_node)
        self.active_state.deactivate()
        self.active_state = to_node
        self.active_state.activate(self)

    def run(self):
        rospy.spin()

    @classmethod
    def run_node(cls, state: State, name: Optional[str] = None, gazebo_reset: bool = False):
        if gazebo_reset:
            reset_gazebo(wait=True)
        cls(state, name or state.__class__.__name__).run()

    def set_speed(self, forward, angular):
        twist = Twist(
            linear=Vector3(forward, 0, 0),
            angular=Vector3(0, 0, angular)
        )
        self.velocity_pub.publish(twist)

    def mark_target(self, *args, **kwargs):
        self.target_pub.publish(make_marker(*args, **kwargs))

    @property
    def odom(self) -> Odometry:
        if self._odom is None:
            raise WaitingForData()
        return self._odom

    @property
    def scan(self) -> LaserScan:
        if self._scan is None:
            raise WaitingForData()
        return self._scan

    @property
    def position(self) -> Point:
        return self.odom.pose.pose.position

    @property
    def orientation(self) -> Quaternion:
        return self.odom.pose.pose.orientation

    @property
    def linear_vel(self) -> Vector3:
        return self.odom.twist.twist.linear

    @property
    def angular_vel(self) -> Vector3:
        return self.odom.twist.twist.angular

    @property
    def laser_ranges(self) -> List[float]:
        return self.scan.ranges


def reset_gazebo(wait: bool = True):
    # Reset Gazebo
    rospy.wait_for_service('/gazebo/reset_world')
    rospy.ServiceProxy('/gazebo/reset_world', Empty)()

    # Let it settle
    if wait:
        rospy.sleep(3)
