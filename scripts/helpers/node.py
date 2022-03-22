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

from helpers import reset_gazebo, make_marker


class WaitingForData(Exception):
    """ Early exit exception for when data hasn't been received yet. """


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


class Node:
    active_state: State

    _odom: Optional[Odometry] = None
    _laser_point_cloud: Optional[PointCloud2] = None

    velocity_pub: rospy.Publisher
    target_pub: rospy.Publisher

    def __init__(self, starting_state: State, name: str):
        rospy.init_node(name)
        self.active_state = starting_state
        self.active_state.activate(self)

        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.target_pub = rospy.Publisher('/target', Marker, queue_size=10)

        self.transform = tf.TransformListener()

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
