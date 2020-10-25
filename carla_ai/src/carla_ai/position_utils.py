import time
from typing import Tuple

import carla
import rospy
import tf


def get_cur_location(tf_listener: tf.TransformListener,
                     role_name: str,
                     max_attempts: int = 1) -> Tuple[carla.Location, float]:
    for attempt in range(1, max_attempts + 1):
        try:
            (position, quaternion) = tf_listener.lookupTransform(
                '/map', role_name, rospy.Time())
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
            center_pos = carla.Location(position[0], -position[1], position[2])
            return center_pos, -yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn(f"Unable to retrieve vehicle location (attempt {attempt}/{max_attempts}")
            if attempt < max_attempts:
                time.sleep(0.1)
    raise Exception(f"Unable to retrieve vehicle location after {max_attempts} attempts")
