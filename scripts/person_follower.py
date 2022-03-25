#!/usr/bin/env python3
import math
import numpy as np
from typing import Optional
from geometry_msgs.msg import Point, TransformStamped, Quaternion, Twist, Vector3
import ros_numpy.point_cloud2 as np_point_cloud2
from scripts.helpers.helpers import make_marker
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
import rospy
from tf.transformations import euler_from_quaternion
from helpers import State, Node, point_math
from tf2_msgs.msg import TFMessage
import tf2_ros


class PersonFollower:
    def __init__(self):
        rospy.init_node('PersonFollower')

        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.target_pub = rospy.Publisher('/target', Marker, queue_size=10)

        self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.laser_scan_sub = rospy.SubscribeListener(
            '/projected_stable_scan', PointCloud2, self.handle_laser_scan)

    def drive(self):
        trans = self.tf_buffer.lookup_transform(
            'base_link', 'target', rospy.Time.now(), rospy.Duration(0.5)
        )

        self.target_pub.publish(make_marker(frame_id='target'))

        self.set_speed(
            rospy.get_param('~forward_vel', 0.1) *
            min(1, point_math.magnitude(trans.transform.translation)),
            rospy.get_param('~angular_vel', 0.5) *
            math.atan2(trans.transform.translation.y,
                       trans.transform.translation.x)
        )

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

    def handle_laser_scan(self, point_cloud: PointCloud2):
        target = self.detect_target(point_cloud)
        self.publish_frame(target)
        self.drive()

    def publish_frame(self, target: np.array):
        t = TransformStamped()

        t.child_frame_id = 'target'
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.node.laser_point_cloud.header.frame_id

        t.transform.translation.x = target[0]
        t.transform.translation.y = target[1]
        t.transform.translation.z = target[2]

        t.transform.rotation = Quaternion(0, 0, 0, 1)

        self.tf_pub.publish(TFMessage([t]))

    def detect_target(self, point_cloud: PointCloud2) -> Optional[np.array]:
        """
        Detect the center position of the target.

        This is currently the geometric mean of all laser scan data in the x-axis range
        [min_depth, max_depth] relative to the robot, and the y-axis range [-width / 2, width / 2]
        (also relative to the robot).

        Returns None if no target can be found.
        """
        point_cloud_rel = self.tf_buffer.transform(
            point_cloud, 'base_link', rospy.Duration(0.5))

        points = np_point_cloud2.pointcloud2_to_xyz_array(point_cloud_rel)

        relevant_points = points[
            (points[:, 0] >= rospy.get_param('~min_depth', 0.2)) &
            (points[:, 0] <= rospy.get_param('~max_depth', 2)) &
            (np.abs(points)[:, 1] <= rospy.get_param('width', 2) / 2)
        ]

        if len(relevant_points) == 0:
            return None
        else:
            return np.mean(relevant_points, axis=0)

    def run():
        rospy.spin()


if __name__ == '__main__':
    PersonFollower().run()
