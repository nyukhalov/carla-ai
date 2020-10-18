import time
import traceback
from typing import List

import carla
import rospy
from carla_msgs.msg import CarlaWorldInfo

from carla_ai import msg
from carla_ai.av import Planner
from carla_ai.av.model import WaypointWithSpeedLimit


class RosPlanner:
    def __init__(self, role_name: str):
        self._role_name = role_name

        self._path_pub = rospy.Publisher(f"/carla_ai/{role_name}/planner/path", msg.PlannerPath, queue_size=1)

        rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)

        host = rospy.get_param("/carla/host", "127.0.0.1")
        port = rospy.get_param("/carla/port", 2000)
        timeout = rospy.get_param("/carla/timeout", 10)

        rospy.loginfo(f"CARLA world available. Trying to connect to {host}:{port}")

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)
        carla_world = carla_client.get_world()
        rospy.loginfo("Connected to Carla.")

        ego_actor = None
        while ego_actor is None:
            for actor in carla_world.get_actors():
                if actor.type_id.startswith("vehicle"):
                    ego_actor = actor
                    print("Found ego actor")
                    break
            print("Ego actor not found...")
            time.sleep(0.5)
        self._planner = Planner(carla_world.get_map(), ego_actor)

    def run(self) -> None:
        try:
            rate = rospy.Rate(1)  # ROS Rate at 1Hz
            while not rospy.is_shutdown():
                self._tick()
                rate.sleep()
        except Exception as e:
            print('RosPlanner node has been interrupted by exception', e)
            traceback.print_exc()
        finally:
            self._destroy()

    def _tick(self) -> None:
        self._planner.plan()
        path = self._planner.path
        self._publish_path(path)

    def _publish_path(self, path: List[WaypointWithSpeedLimit]):
        path_msg = msg.PlannerPath()
        path_msg.path = [self._to_msg_wp(wp) for wp in path]
        self._path_pub.publish(path_msg)

    def _to_msg_wp(self, wp: WaypointWithSpeedLimit) -> msg.WaypointWithSpeedLimit:
        wp_msg = msg.WaypointWithSpeedLimit()
        wp_msg.x = wp.x
        wp_msg.y = wp.y
        wp_msg.z = wp.z
        wp_msg.speed_limit = wp.speed_limit
        return wp_msg

    def _destroy(self) -> None:
        self._path_pub.unregister()
