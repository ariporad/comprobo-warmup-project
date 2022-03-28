#!/usr/bin/env python3
"""
Obstacle Avoider: Move to a specified goal, avoiding obstacles.

Uses a potential field to simulate the goal and obstacles, inspired by [0].

Parameters:

goal_x: x coordinate of the target
goal_y: y coordinate of the target

forward_vel: maximum forwards velocity
angular_vel: angular velocity (may be exceeded, but used for setting the speed)
angle_preference_factor: a huristic value, higher means the robot (which can only translate forward)
                         will be more picky about being aligned with the proper heading before
                         moving forward.

goal_intensity: potential field intensity of the goal
obstacle_intensity: potential field intensity of obstacles (one obstacle per LIDAR point) 
obstacle_decay_factor: higher values result in farther-away obstacles having less force

goal_tolerance: how close to the goal need the robot be to stop moving?

Reasonable defaults, determined through experimentation in the simulator, have been set for all parameters.

[0]: https://phoenix.goucher.edu/~jillz/cs325_robotics/goodrich_potential_fields.pdf
"""
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from typing import Tuple, Optional
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Twist, Vector3, Quaternion
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf.transformations import quaternion_from_euler
from ros_numpy import point_cloud2 as np_point_cloud2
from helpers import make_marker


class ObstacleAvoider:
    def __init__(self):
        rospy.init_node('ObstacleAvoider')

        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.target_pub = rospy.Publisher('/target', Marker, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.laser_scan_sub = rospy.Subscriber(
            '/projected_stable_scan', PointCloud2, self.process_laser_scan)

        self.goal = PointStamped()
        self.goal.header.frame_id = 'odom'
        # Default goal is a good place for the default gauntlet sim world.
        self.goal.point.x = rospy.get_param('~goal_x', 0)
        self.goal.point.y = rospy.get_param('~goal_y', -1.875)
        self.goal.point.z = 0

        # Higher values means the robot will want to be more directionally aligned before moving
        # forward.
        self.angle_preference_factor = rospy.get_param(
            '~angle_preference_factor', 5)
        self.forward_vel = rospy.get_param('~forward_vel', 0.1)
        self.angular_vel = rospy.get_param('~angular_vel', 0.15)

        self.goal_intensity = rospy.get_param('~goal_intensity', 1000)
        self.obstacle_intensity = rospy.get_param('~obstacle_intensity', 2)
        self.obstacle_decay_factor = rospy.get_param(
            '~obstacle_decay_factor', 100)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.1)

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
        """ Process new laser scan data, by calculating the direction to move in and doing so. """

        target = self.calculate_direction(point_cloud)

        if target is None:
            print("At target!")
            self.set_speed(0, 0)
            return

        angle = math.atan2(target[1], target[0])
        magnitude = min(1, math.sqrt((target[0] ** 2) + (target[1] ** 2)))

        # Visualize the target
        orientation = Quaternion(*quaternion_from_euler(0, 0, angle))
        self.target_pub.publish(
            make_marker(
                frame_id='base_link',
                orientation=orientation,
                scale=(magnitude, 0.1, 0.1),
                shape=Marker.ARROW,
                id=1
            )
        )

        # This logic ensures that the robot (which can only translate in one direction), is
        # sufficiently aligned with the direction we want to go before it starts translating at all.
        # In other words, if the current heading is *very* wrong, the robot will rotate in place.
        # If we're pretty close to the right heading, we'll rotate and translate at the same time.
        speed_scalar = max(0,
                           1 - ((self.angle_preference_factor * abs(angle)) / math.pi)
                           )

        self.set_speed(
            self.forward_vel * speed_scalar,
            self.angular_vel * angle
        )

    def calculate_direction(self, point_cloud: PointCloud2) -> Optional[Tuple[int, int]]:
        """
        Calculate the direction that the robot should move, by treating obstacles (from LIDAR) as
        repellant forces and the goal as an attractive force.

        Returns the x and y components of the vector representing the direction.
        """

        # Calculate the goal, relative to the robot
        self.goal.header.stamp = rospy.Time.now()
        goal_rel = self.tf_buffer.transform(
            self.goal, 'base_link', rospy.Duration(0.5)
        )

        goal_dist = math.sqrt((goal_rel.point.x ** 2) + (goal_rel.point.y**2))

        # We hit the target!
        if goal_dist < self.goal_tolerance:
            return None

        net_x = self.goal_intensity * (goal_rel.point.x / goal_dist)
        net_y = self.goal_intensity * (goal_rel.point.y / goal_dist)

        self.target_pub.publish(
            make_marker(
                goal_rel.point,
                frame_id='base_link',
                scale=0.25,
                id=2
            )
        )

        # Calculate obstacles
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

        # Note that the robot is at (0, 0, 0) in this reference frame
        for x, y, z in points:
            net_x += self.obstacle_intensity / \
                ((x * self.obstacle_decay_factor) ** 2)
            net_y += self.obstacle_intensity / \
                ((y * self.obstacle_decay_factor) ** 2)

        return net_x, net_y

    def run(self):
        rospy.on_shutdown(lambda: self.set_speed(0, 0))
        rospy.spin()


if __name__ == '__main__':
    ObstacleAvoider().run()
