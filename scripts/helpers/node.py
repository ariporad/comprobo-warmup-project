"""
Helper classes for encapsulating state-machine-capable nodes with various housekeeping functions.

This was an example of premature optimization (the root of all evil). It was written well before I
understood what functionality was actually common between different behaviors, and how to use TF.
There are (possibly) some decent ideas here, but they're overkill and need some serious rethinking.
"""
from __future__ import annotations
from cmath import isnan
import numpy as np
import rospy
import math
import tf
import tf2_ros
import ros_numpy.point_cloud2 as np_point_cloud2
from math import pi
from typing import Any, Generic, Optional, List, Type, Union, Iterable, Tuple, TypeVar, Dict
from std_srvs.srv import Empty
from geometry_msgs.msg import Quaternion, Point, Vector3, Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2

from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler, quaternion_conjugate

from .helpers import make_marker

T = TypeVar('T')


class WaitingForData(Exception):
    """ Early exit exception for when data hasn't been received yet. """


class State:
    """
    A specific behavioral state for a Node.

    Don't use me directly, subclass me!

    State objects are single-use, and cannot be activated twice.
    """
    node: Optional[Node] = None

    default_params: dict[str, Any] = {}
    """
    Default values for ROS parameters. Actual values from ROS will be populated in self.params.

    A concise way to list all the parameters that a State will use at the top.
    """

    def activate(self, node: Node):
        """ Method invoked when a State is activated by a Node. """
        self.node = node

    def deactivate(self):
        """ Method invoked when a State is deactivated by a Node. """
        self.node = None

    def update(self):
        """ Method invoked whenever new data is received. """
        raise NotImplementedError('Override me!')

    def run(self, name: Optional[str] = None):
        """
        Helper method to create and run a Node, starting with this state.
        """
        Node.run_node(self, name or self.__class__.__name__,)

    @property
    def params(self) -> dict[str, Any]:
        """ Get ROS parameters (but only those defined in default_params). """
        return {
            (name): self.node.param(name, val)
            for name, val in self.default_params.items()
        }


class Node:
    """
    A helper class representing a finite-state-machine ROS Node. Also useful for single-state programs.

    You likely won't need to use this much directly. Subclass State instead.
    """
    active_state: Optional[State] = None
    _next_active_state: Optional[State] = None

    # Access these via the getters below
    _odom: Optional[Odometry] = None
    _laser_point_cloud: Optional[PointCloud2] = None

    velocity_pub: rospy.Publisher
    target_pub: rospy.Publisher

    def __init__(self, starting_state: State, name: str):
        rospy.init_node(name)
        self._next_active_state = starting_state

        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.target_pub = rospy.Publisher('/target', Marker, queue_size=10)

        self._subscribe('/odom', Odometry, '_odom')
        self._subscribe('/projected_stable_scan',
                        PointCloud2, '_laser_point_cloud')

    def _subscribe(self, topic: str, msg_type: Type, attr_name: str, *args, **kwargs):
        def _handler(msg):
            self.__setattr__(attr_name, msg)
            self.update()
        rospy.Subscriber(topic, msg_type, _handler, *args, **kwargs)

    def update(self):
        try:
            # If we have a new state to transition to, try that
            # This is carefully structured so that (de)activate can throw a WaitingForData, and
            # abort, and the next update call will pick up from the right place.
            if self._next_active_state is not None:
                print("Transitioning to:", self._next_active_state)
                if self.active_state is not None:
                    self.active_state.deactivate()
                    self.active_state = None
                self.active_state = self._next_active_state
                self.active_state.activate(self)
                self._next_active_state = None

            # Update either way
            self.active_state.update()
        except (WaitingForData, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # If we're waiting for data, then we can't update yet (but it isn't an error!)
            pass

    def transition(self, to_node):
        """
        Transition to another state. Note that transitions are asynchronous.
        """
        print("Scheduling transition to:", to_node)
        self._next_active_state = to_node

    def run(self):
        rospy.spin()

    def param(self, name: str, default_value: T) -> T:
        return rospy.get_param('~' + name, default_value)

    @classmethod
    def run_node(cls, state: State, name: Optional[str] = None):
        cls(state, name or state.__class__.__name__).run()

    def set_speed(self, forward, angular):
        # Publishing any NaN values crashes Gazebo
        if isnan(forward):
            forward = 0
        if isnan(angular):
            angular = 0
        twist = Twist(
            linear=Vector3(forward, 0, 0),
            angular=Vector3(0, 0, angular)
        )
        self.velocity_pub.publish(twist)

    def mark_target(self, *args, **kwargs):
        """
        Helper method to publish a marker as the target.

        Same arguments as helpers.make_marker
        """
        self.target_pub.publish(make_marker(*args, **kwargs))

    # These helper methods all throw a WaitingForData if there's no data yet, which is silently
    # caught by update(), so States can just always assume there will be data and silently fail if
    # not.
    @property
    def odom(self) -> Odometry:
        if self._odom is None:
            raise WaitingForData()
        return self._odom

    @property
    def laser_point_cloud(self) -> PointCloud2:
        if self._laser_point_cloud is None:
            raise WaitingForData()
        return self._laser_point_cloud

    @property
    def laser_points(self) -> np.array:
        return np_point_cloud2.pointcloud2_to_xyz_array(
            self.laser_point_cloud
        )

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
