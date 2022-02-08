#!/usr/bin/env python3
""" Drive forward until the Neato hits something. """
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int8MultiArray
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
			self.set_speed(0.5, 0)
		elif key in ['o', 's']:
			self.set_speed(-0.5, 0)
		elif key in ['a']:
			self.set_speed(0, 1)
		elif key in ['e', 'd']:
			self.set_speed(0, -1)

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
	node = TeleopNode()
	node.run()
