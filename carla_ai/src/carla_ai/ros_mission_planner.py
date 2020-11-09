import carla
import math
import random
import rospy
from carla_msgs.msg import CarlaWorldInfo
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


class RosMissionPlanner(object):
    def __init__(self, role_name: str):
        self._goal_pub = rospy.Publisher(f"/carla/{role_name}/goal", PoseStamped, queue_size=1)

        rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)

        host = rospy.get_param("/carla/host", "127.0.0.1")
        port = rospy.get_param("/carla/port", 2000)
        timeout = rospy.get_param("/carla/timeout", 10)

        rospy.loginfo(f"CARLA world available. Trying to connect to {host}:{port}")

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)
        carla_world = carla_client.get_world()
        rospy.loginfo("Connected to Carla.")

        spawn_points = carla_world.get_map().get_spawn_points()
        spawn_point = random.choice(spawn_points)

        x = spawn_point.location.x
        y = spawn_point.location.y
        z = spawn_point.location.z
        yaw = spawn_point.rotation.yaw
        rospy.loginfo(f"Setting goal to (x={x}, y={y}, z={z}, yaw={yaw})")
        goal = self._carla_tranform_to_pose_stamped(spawn_point)
        self._goal_pub.publish(goal)

    def _carla_tranform_to_pose_stamped(self, t: carla.Transform) -> PoseStamped:
        pose = PoseStamped()
        pose.pose.position.x = t.location.x
        pose.pose.position.y = -t.location.y
        pose.pose.position.z = t.location.z  # move it toward the ground

        yaw = -math.radians(t.rotation.yaw)
        x, y, z, w = quaternion_from_euler(0, 0, yaw)

        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        return pose

    def run(self) -> None:
        pass
