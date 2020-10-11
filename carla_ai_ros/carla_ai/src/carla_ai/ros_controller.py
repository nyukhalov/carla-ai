from typing import Tuple

import carla
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus, CarlaWorldInfo
from carla_ai.msg import ControllerDebugInfo
import rospy
import time

from shapely.geometry import Point, LineString

from carla_ai.av.control import PID
from carla_ai.av.model import WaypointWithSpeedLimit
from carla_ai.av.planner import Planner
from carla_ai.util.math import clamp


class RosController(object):
    def __init__(self, role_name: str):
        self.steer_pid = PID(0.03, 0, 0.02)
        self.throttle_pid = PID(0.05, 0.00022, 0.008)
        self.prev_steer = 0.0
        self.speed = 0

        self._vehicle_status_subscriber = rospy.Subscriber(
            f"/carla/{role_name}/vehicle_status",
            CarlaEgoVehicleStatus,
            self._on_vehicle_status
        )
        self._vehicle_control_publisher = rospy.Publisher(f"/carla/{role_name}/vehicle_control_cmd",
                                                          CarlaEgoVehicleControl, queue_size=1)

        self._debug_info_publisher = rospy.Publisher(f"/carla_ai/{role_name}/controller/debug_info",
                                                     ControllerDebugInfo, queue_size=1)

        rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)

        host = rospy.get_param("/carla/host", "127.0.0.1")
        port = rospy.get_param("/carla/port", 2000)
        timeout = rospy.get_param("/carla/timeout", 10)

        rospy.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
            host=host, port=port))

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
        self.planner = Planner(carla_world.get_map(), ego_actor)

    def tick(self):
        self.planner.plan()

        target_speed = self._get_target_speed()
        speed_err = target_speed - self.speed
        cte = self._calc_lateral_error()  # cross-track error

        max_delta = 100
        steer = clamp(-1, self.steer_pid.update(cte), 1)
        steer = clamp(self.prev_steer - max_delta, steer, self.prev_steer + max_delta)
        self.prev_steer = steer
        brake = 0
        throttle = clamp(-1, self.throttle_pid.update(speed_err), 1)
        if throttle < 0:
            throttle = 0
            brake = -throttle

        # public control command
        control = CarlaEgoVehicleControl()
        control.throttle = throttle
        control.steer = steer
        control.brake = brake
        control.hand_brake = False
        self._vehicle_control_publisher.publish(control)

        # publish debug info
        debug_info_msg = ControllerDebugInfo()
        debug_info_msg.cross_track_error = cte
        debug_info_msg.speed_error = speed_err
        debug_info_msg.target_speed = target_speed
        self._debug_info_publisher.publish(debug_info_msg)

    def destroy(self):
        self._vehicle_control_publisher.unregister()
        self._vehicle_status_subscriber.unregister()
        self._debug_info_publisher.unregister()

    def _on_vehicle_status(self, msg: CarlaEgoVehicleStatus) -> None:
        self.speed = msg.velocity

    def _get_target_speed(self) -> float:
        cur_pose = self.planner.ego_location
        closest_node = None
        min_distance = float('inf')
        for node in self.planner.path:
            dist = node.waypoint.transform.location.distance(cur_pose)
            if dist < min_distance:
                min_distance = dist
                closest_node = node
        if closest_node is None:
            print('[WARN] The path is empty')
            return 0
        return closest_node.speed_limit

    def _calc_lateral_error(self) -> float:
        # use the center position intead of the rear axle center position
        # as it works better with PID steering controller
        cur_pose = self.planner.ego_location
        path = self.planner.path

        closest_node_idx = None
        min_distance = float('inf')
        for idx, node in enumerate(path):
            dist = node.waypoint.transform.location.distance(cur_pose)
            if dist < min_distance:
                min_distance = dist
                closest_node_idx = idx

        if closest_node_idx is None:
            print('[WARN] Could not find the closest node')
            return 0.0

        cur_node = path[closest_node_idx]
        next_node = path[closest_node_idx + 1] if closest_node_idx < len(path) - 1 else None
        prev_node = path[closest_node_idx - 1] if closest_node_idx > 0 else None

        closest_edge = None
        if prev_node is None:
            closest_edge = (cur_node, next_node)
        elif next_node is None:
            closest_edge = (prev_node, cur_node)
        elif next_node.distance(cur_pose) < prev_node.distance(cur_pose):
            closest_edge = (cur_node, next_node)
        else:
            closest_edge = (prev_node, cur_node)
        sign = -1 if self._is_left(closest_edge, cur_pose) else 1

        cur_pose_p = Point(cur_pose.x, cur_pose.y)
        path_ls = LineString([(node.x, node.y) for node in closest_edge])
        try:
            return sign * path_ls.distance(cur_pose_p)
        except Exception as e:
            print(f'[ERROR] Failed to calculate the distance to the closest path edge: {e}')
            return sign * 999

    def _is_left(self, edge: Tuple[WaypointWithSpeedLimit, WaypointWithSpeedLimit], p: carla.Location):
        a, b = edge
        return ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)) > 0
