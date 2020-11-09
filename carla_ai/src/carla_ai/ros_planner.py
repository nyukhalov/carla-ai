import time
import traceback
from typing import List

import carla
import rospy
from carla_msgs.msg import CarlaWorldInfo

from nav_msgs.msg import Path
from carla_ai import msg


class RosPlanner:
    def __init__(self, role_name: str):
        self._role_name = role_name
        self._path_pub = rospy.Publisher(f"/carla_ai/{role_name}/planner/path", msg.PlannerPath, queue_size=1)
        self._raw_path_sub = rospy.Subscriber(f"/carla/{role_name}/waypoints", Path, self._on_raw_path)

    def run(self) -> None:
        try:
            rate = rospy.Rate(1)  # ROS Rate at 1Hz
            while not rospy.is_shutdown():
                rate.sleep()
        except Exception as e:
            print('RosPlanner node has been interrupted by exception', e)
            traceback.print_exc()
        finally:
            self._destroy()

    def _on_raw_path(self, path: Path) -> None:
        print(f"Got path of {len(path.poses)} poses")
        path_msg = self._nav_path_to_planner_path(path)
        self._path_pub.publish(path_msg)

    def _nav_path_to_planner_path(self, path: Path) -> msg.PlannerPath:
        path_msg = msg.PlannerPath()
        path_msg.path = []
        for pose in path.poses:
            wp_msg = msg.WaypointWithSpeedLimit()
            wp_msg.x = pose.pose.position.x
            wp_msg.y = -pose.pose.position.y
            wp_msg.z = pose.pose.position.z
            wp_msg.speed_limit = 3
            path_msg.path.append(wp_msg)
        return path_msg

    def _destroy(self) -> None:
        self._path_pub.unregister()
        self._raw_path_sub.unregister()
