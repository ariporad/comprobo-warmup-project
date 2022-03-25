import math
import numpy as np
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler, quaternion_conjugate


def multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """ q1 * q2 """
    return tf_to_rospy(
        quaternion_multiply(rospy_to_tf(q1), rospy_to_tf(q2))
    )


def difference(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """ q2 = x * q1, returns x """
    return tf_to_rospy(
        quaternion_multiply(rospy_to_tf(q2), rospy_to_tf(inverse(q1)))
    )


def inverse(q: Quaternion) -> Quaternion:
    return Quaternion(q.x, q.y, q.z, -q.w)


def ijk_magnitude(q: Quaternion) -> Quaternion:
    return math.sqrt((q.x ** 2) + (q.y ** 2) + (q.z ** 2))


def rospy_to_tf(q: Quaternion) -> np.array:
    if isinstance(q, list):
        q = np.array(q)
    if isinstance(q, np.ndarray):
        return q
    return np.array([q.x, q.y, q.z, q.w])


def tf_to_rospy(q: np.array) -> Quaternion:
    if isinstance(q, Quaternion):
        return q
    return Quaternion(q[0], q[1], q[2], q[3])
