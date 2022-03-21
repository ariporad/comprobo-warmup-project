#!/usr/bin/env python3
import numpy
import rospy
import math
from math import pi
from typing import Optional, List
from std_srvs.srv import Empty
from geometry_msgs.msg import Quaternion, Point, Vector3, Twist, Pose
from nav_msgs.msg import Odometry
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


class NodeState:
    node: Optional['StateMachineNode'] = None

    def activate(self, node: 'StateMachineNode'):
        self.node = node

    def deactivate(self):
        self.node = None

    def update(self):
        raise NotImplementedError('Override me!')


class DummyState(NodeState):
    """
    Helper state that waits for an update to be received, then transitions to another state.

    Used to ensure that the first state is activated after an update has been received.
    """
    next: NodeState

    def __init__(self, next: NodeState):
        self.next = next

    def update(self):
        self.node.transition(self.next)


class StateMachineNode:
    active_state: NodeState
    position: Point = Point(0, 0, 0)

    orientation: Quaternion = Quaternion(0, 0, 0, 0)
    linear_vel: Vector3 = Vector3(0, 0, 0)
    angular_vel: Vector3 = Vector3(0, 0, 0)

    def __init__(self, starting_state: NodeState, name: str = 'square'):
        rospy.init_node(name)
        self.active_state = DummyState(starting_state)
        self.active_state.activate(self)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.process_odom)

    def transition(self, to_node):
        print("Transitioning to:", to_node)
        self.active_state.deactivate()
        self.active_state = to_node
        self.active_state.activate(self)

    def set_speed(self, forward, angular):
        twist = Twist(
            linear=Vector3(forward, 0, 0),
            angular=Vector3(0, 0, angular)
        )
        self.cmd_vel.publish(twist)

    def process_odom(self, odom: Odometry):
        self.position = odom.pose.pose.position
        self.orientation = odom.pose.pose.orientation
        self.linear_vel = odom.twist.twist.linear
        self.angular_vel = odom.twist.twist.angular
        self.active_state.update()

    def run(self):
        rospy.spin()


class MoveForwardState(NodeState):
    speed: float
    distance: float
    tolerance: float
    target: Point = Point(0, 0, 0)
    info_target: rospy.Publisher

    def __init__(self, speed: float = 0.2, distance: float = 2, tolerance=0.1):
        self.speed = speed
        self.distance = distance
        self.tolerance = tolerance
        self.info_target = rospy.Publisher('/target', Marker, queue_size=10)

    def activate(self, node: StateMachineNode):
        super().activate(node)
        distance = [-self.distance, 0, 0, 0]  # w is always 0 for vectors
        target_rotated_q = QuaternionMath.multiply(
            QuaternionMath.multiply(self.node.orientation, distance),
            QuaternionMath.inverse(self.node.orientation)
        )
        target_rotated = Point(
            target_rotated_q.x, target_rotated_q.y, target_rotated_q.z)
        self.target = PointMath.add(self.node.position, target_rotated)

    def update(self):
        self.info_target.publish(
            make_marker(
                position=self.target,
                scale=(0.25, 0.25, 0.25),
                color=(0, 1, 0, 1)
            )
        )
        if PointMath.distance(self.node.position, self.target) < self.tolerance:
            self.node.set_speed(0, 0)
            self.node.transition(RotatingLeftState())
        else:
            self.node.set_speed(self.speed, 0)


class RotatingLeftState(NodeState):
    speed: float
    angle: float

    target: Quaternion = Quaternion(0, 0, 0, 0)
    info_target: rospy.Publisher

    def __init__(self, speed: float = 0.3, angle: float = pi / 2.0):
        self.speed = speed
        self.angle = angle
        self.info_target = rospy.Publisher('/target', Marker, queue_size=10)

    def activate(self, node: StateMachineNode):
        super().activate(node)
        self.target = QuaternionMath.multiply(
            quaternion_from_euler(0, 0, self.angle),
            self.node.orientation
        )

    def update(self):
        self.info_target.publish(
            make_marker(
                position=self.node.position,
                orientation=self.target,
                shape=Marker.ARROW,
                color=(0, 1, 0, 1)
            )
        )
        if QuaternionMath.ijk_magnitude(
            QuaternionMath.difference(self.target, self.node.orientation)
        ) < 0.01:
            self.node.set_speed(0, 0)
            self.node.transition(MoveForwardState())
        else:
            self.node.set_speed(0, self.speed)


if __name__ == '__main__':
    # Reset Gazebo
    rospy.wait_for_service('/gazebo/reset_world')
    rospy.ServiceProxy('/gazebo/reset_world', Empty)()

    # Let it settle
    rospy.sleep(3)

    # Run
    node = StateMachineNode(MoveForwardState())
    node.run()
