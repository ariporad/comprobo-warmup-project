#!/usr/bin/env python3
"""
Person Follower: Follow a person in front of the robot.

Where a "person" is simply any object in front of the robot. Specifically, the geometric mean of all
LIDAR points in the 2x2 meter detection zone in front of the robot.

Parameters:

width: width of the detection zone
min_depth: the close edge of the detection zone is this distance from the center of the robot.
max_depth: the far edge of the detection zone is this distance from the center of the robot.

forward_vel: controls the forward speed of the robot, but not a maximum speed
angular_vel: controls the angular speed of the robot, but not a maximum speed

Sane defaults, determined through experimentation in the simulator, have been set for all parameters.
"""
import math
import numpy as np
import rospy
import tf2_ros
from typing import Optional
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist, Vector3
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from ros_numpy import point_cloud2 as np_point_cloud2
from helpers import make_marker


class PersonFollower:
    def __init__(self):
        rospy.init_node('PersonFollower')

        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.target_pub = rospy.Publisher('/target', Marker, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.laser_scan_sub = rospy.Subscriber(
            '/projected_stable_scan', PointCloud2, self.process_laser_scan)

        self.width = rospy.get_param('~width', 2)
        self.min_depth = rospy.get_param('~min_depth', 0)
        self.max_depth = rospy.get_param('~max_depth', 2)
        self.forward_vel = rospy.get_param('~forward_vel', 0.1)
        self.angular_vel = rospy.get_param('~angular_vel', 0.5)

    def set_speed(self, forward, angular):
        if math.isnan(forward):
            forward = 0
        if math.isnan(angular):
            angular = 0
        twist = Twist(
            linear=Vector3(forward, 0, 0),
            angular=Vector3(0, 0, angular)
        )
        self.velocity_pub.publish(twist)

    def process_laser_scan(self, point_cloud: PointCloud2):
        """ Process new laser scan data, by detecting the target and moving accordingly. """

        # Visualize the target search area
        height = (self.max_depth - self.min_depth)
        self.target_pub.publish(make_marker(
            Point((height / 2) + self.min_depth, 0, 0),
            id=0,
            shape=Marker.CUBE,
            scale=(height, self.width, 0.01),
            color=(0, 0, 1, 0.25),
            frame_id='base_link'
        ))

        target = self.detect_target(point_cloud)

        if target is None:
            self.set_speed(0, 0)
            return

        self.target_pub.publish(
            make_marker(Point(*target), frame_id='base_link', scale=0.25, id=1)
        )

        distance = min(1, math.sqrt((target[0] ** 2) + (target[1] ** 2)))
        self.set_speed(
            self.forward_vel * distance,
            self.angular_vel * math.atan2(target[1], target[0])
        )

    def detect_target(self, point_cloud: PointCloud2) -> Optional[np.array]:
        """
        Detect the center position of the target.

        This is currently the geometric mean of all laser scan data in the x-axis range
        [min_depth, max_depth] relative to the robot, and the y-axis range [-width / 2, width / 2]
        (also relative to the robot).

        Returns the x, y, z coordinates of the target in the base_link frame, or None if the target
        can't be found.
        """
        point_cloud_rel = do_transform_cloud(
            point_cloud,
            self.tf_buffer.lookup_transform(
                'base_link',
                point_cloud.header.frame_id,
                rospy.Time.now(),
                rospy.Duration(0.5)
            )
        )
        points = np_point_cloud2.pointcloud2_to_xyz_array(point_cloud_rel)

        relevant_points = points[
            (points[:, 0] >= self.min_depth) &
            (points[:, 0] <= self.max_depth) &
            (np.abs(points)[:, 1] <= self.width / 2)
        ]

        if len(relevant_points) == 0:
            return None
        else:
            return np.mean(relevant_points, axis=0)

    def run(self):
        rospy.on_shutdown(lambda: self.set_speed(0, 0))
        rospy.spin()


if __name__ == '__main__':
    PersonFollower().run()
