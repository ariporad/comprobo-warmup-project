#!/usr/bin/env python3
"""
Teleop: Human control of the robot

A simple CLI-based remote control for the robot.

Controls (from stdin) are:
- w/a/s/d = forward/left/backward/right (Dvorak bindings of ,/a/o/e are accepted too).
- [Space] = stop the robot
- q = quit
- ? = show this help message

Parameters:

- forward_vel: forward (and backward) speed of the robot
- angular_vel: angular speed of the robot

Sane defaults, determined through experimentation in the simulator, have been set for all parameters.
"""
from geometry_msgs.msg import Twist, Vector3
import rospy
import tty
import select
import sys
import termios


class AsyncInputReader:
    """
    Get non-blocking input from stdin. Based on code provided by the teaching team
    """

    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)

    def read(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        if key == '\x03':
            raise KeyboardInterrupt()

        return key


class TeleopNode:
    def __init__(self, name='teleop'):
        rospy.init_node(name)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.input = AsyncInputReader()
        self.forward_vel = rospy.get_param('~forward_vel', 0.5)
        self.angular_vel = rospy.get_param('~angular_vel', 1)

    def set_speed(self, forward, angular):
        twist = Twist(
            linear=Vector3(forward, 0, 0),
            angular=Vector3(0, 0, angular)
        )
        self.cmd_vel.publish(twist)

    def handle_input(self, key: str):
        if key == ' ':
            self.set_speed(0, 0)
        elif key == 'q':
            self.set_speed(0, 0)
            raise KeyboardInterrupt("Exit requested!")
        elif key == '?':
            print("""
Neato Teleop:
w/a/s/d = forward/left/backward/right (Dvorak bindings of ,/a/o/e are accepted too).
[Space] = stop the robot
q = quit
? = show this help message
			""")

        elif key in [',', 'w']:
            self.set_speed(self.forward_vel, 0)
        elif key in ['o', 's']:
            self.set_speed(-self.forward_vel, 0)
        elif key in ['a']:
            self.set_speed(0, self.angular_vel)
        elif key in ['e', 'd']:
            self.set_speed(0, -self.angular_vel)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.handle_input(self.input.read())
                r.sleep()
            except KeyboardInterrupt:
                if not rospy.is_shutdown():
                    self.set_speed(0, 0)
                raise


if __name__ == '__main__':
    TeleopNode().run()
