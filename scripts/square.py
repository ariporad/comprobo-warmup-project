#!/usr/bin/env python3
""" 
Drive in a Square

Uses odometry to turn precisely. Technically capable of driving in any regular polygon.

This script predates my understanding of how to use TF, and more broadly how to write good ROS code.

Parameters:

linear_vel: linear velocity when driving straight
distance: length of each side
distance_tolerance: tolerance for when the target distance has been hit
angular_vel: angular velocity (when turning)
angle: angle to turn for each corner
angle_tolerance: tolerance for when the target angle has been reached

Reasonable defaults, determined through experimentation in the simulator, have been set for all parameters.
"""
import rospy
from math import pi
from geometry_msgs.msg import Quaternion, Point
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler

from helpers import State, Node, q_math, point_math, make_marker


class MoveForwardState(State):
    target: Point = Point(0, 0, 0)

    default_params = {
        'linear_vel': 0.2,
        'distance': 1,
        'distance_tolerance': 0.25
    }

    def activate(self, node: Node):
        super().activate(node)
        # w is always 0 for vectors
        distance = [-self.params['distance'], 0, 0, 0]
        target_rotated_q = q_math.multiply(
            q_math.multiply(self.node.orientation, distance),
            q_math.inverse(self.node.orientation)
        )
        target_rotated = Point(
            target_rotated_q.x, target_rotated_q.y, target_rotated_q.z)
        self.target = point_math.add(self.node.position, target_rotated)

    def update(self):
        self.node.mark_target(
            point=self.target,
            scale=(0.25, 0.25, 0.25),
            color=(0, 1, 0, 1)
        )
        if point_math.distance(self.node.position, self.target) < self.params['distance_tolerance']:
            self.node.set_speed(0, 0)
            self.node.transition(RotatingLeftState())
        else:
            self.node.set_speed(self.params['linear_vel'], 0)


class RotatingLeftState(State):
    target: Quaternion = Quaternion(0, 0, 0, 0)
    info_target: rospy.Publisher

    default_params = {
        'angular_vel': 0.3,
        'angle': pi / 2.0,
        'angle_tolerance': 0.025,
    }

    def activate(self, node: Node):
        super().activate(node)
        self.target = q_math.multiply(
            quaternion_from_euler(0, 0, self.params['angle']),
            self.node.orientation
        )

    def update(self):
        self.node.mark_target(
            point=self.node.position,
            orientation=self.target,
            shape=Marker.ARROW,
            color=(0, 1, 0, 1)
        )
        if q_math.ijk_magnitude(
            q_math.difference(self.target, self.node.orientation)
        ) < self.params['angle_tolerance']:
            self.node.set_speed(0, 0)
            self.node.transition(MoveForwardState())
        else:
            self.node.set_speed(0, self.params['angular_vel'])


class DummyState(State):
    """
    KLUDGE: State which, on the first update, transitions to another state.

    Used because our states need data in their activate() method, which they won't have till the
    first update.
    """
    next_state: State

    def __init__(self, next_state: State):
        self.next_state = next_state

    def update(self):
        self.node.transition(self.next_state)


if __name__ == '__main__':
    DummyState(MoveForwardState()).run()
