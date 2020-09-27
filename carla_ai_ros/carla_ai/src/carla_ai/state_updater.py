import math
from typing import Tuple, Optional
import time

import carla
import rospy
import tf
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaEgoVehicleInfo, CarlaEgoVehicleInfoWheel, CarlaWorldInfo
from shapely.geometry import Point, LineString

from carla_ai.av.model import WaypointWithSpeedLimit, VehicleInfo


class StateUpdater(object):
    def __init__(self, role_name: str):
        self._role_name = role_name
        self.steer = 0.0
        self.steering_wheel_angle = 0.0
        self.speed = 0.0
        self.target_speed = 0.0
        self.speed_err = 0.0
        self.cte = 0.0
        self.ego_location = carla.Location(0, 0, 0)
        self.ego_heading = 0.0
        self.ego_vel: carla.Vector3D = carla.Vector3D(0, 0, 0)
        self.ego_acc: carla.Vector3D = carla.Vector3D(0, 0, 0)
        self.veh_info: Optional[VehicleInfo] = None
        self.map_name: str = "Unknown"

        self._tf_listener = tf.TransformListener()
        self._vehicle_info_subscriber = rospy.Subscriber(
            f"/carla/{role_name}/vehicle_info",
            CarlaEgoVehicleInfo,
            self._on_vehicle_info,
        )
        self._vehicle_status_subscriber = rospy.Subscriber(
            f"/carla/{role_name}/vehicle_status",
            CarlaEgoVehicleStatus,
            self._on_vehicle_status
        )
        self._world_info_subscriber = rospy.Subscriber(
            "/carla/world_info",
            CarlaWorldInfo,
            self._on_world_info
        )

    def _on_vehicle_info(self, msg: CarlaEgoVehicleInfo) -> None:
        wheels = msg.wheels

        max_steer_angle = wheels[0].max_steer_angle
        wheel_base = 0.0  # TODO: calc

        rlw: CarlaEgoVehicleInfoWheel = wheels[2]  # rear left wheel
        rrw: CarlaEgoVehicleInfoWheel = wheels[3]  # rear right wheel

        center_pos, _ = self._get_cur_location(max_attempts=10)

        l_wheel_pos = carla.Location(rlw.position.x / 100, rlw.position.y / 100, rlw.position.z / 100)
        r_wheel_pos = carla.Location(rrw.position.x / 100, rrw.position.y / 100, rrw.position.z / 100)

        dist_between_rear_wheels = l_wheel_pos.distance(r_wheel_pos)
        rear_axle_pos_offset_1 = math.sqrt(l_wheel_pos.distance(center_pos)**2 - (dist_between_rear_wheels/2)**2)
        rear_axle_pos_offset_2 = math.sqrt(r_wheel_pos.distance(center_pos)**2 - (dist_between_rear_wheels/2)**2)
        rear_axle_pos_offset = (rear_axle_pos_offset_1 + rear_axle_pos_offset_2) / 2

        self.veh_info = VehicleInfo(
            max_steer_angle,
            wheel_base,
            rear_axle_pos_offset
        )

    def _on_vehicle_status(self, msg: CarlaEgoVehicleStatus) -> None:
        if self.veh_info is None:
            return
        self.speed = msg.velocity
        self.ego_acc.x = msg.acceleration.linear.x
        self.ego_acc.y = msg.acceleration.linear.y
        self.ego_acc.z = msg.acceleration.linear.z

        steer_norm = msg.control.steer
        self.steer = steer_norm * self.veh_info.max_steer_angle
        self.steering_wheel_angle = steer_norm * math.radians(450)  # 450 degrees is chosen randomly

        center_pos, yaw = self._get_cur_location()
        self.ego_heading = yaw
        self.ego_location: carla.Location = self._get_veh_pos(center_pos, self.ego_heading)

    def _on_world_info(self, msg: CarlaWorldInfo) -> None:
        self.map_name = msg.map_name

    def _get_cur_location(self, max_attempts: int = 1) -> Tuple[carla.Location, float]:
        for attempt in range(1, max_attempts + 1):
            try:
                (position, quaternion) = self._tf_listener.lookupTransform(
                    '/map', self._role_name, rospy.Time())
                _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
                center_pos = carla.Location(position[0], -position[1], position[2])
                return center_pos, -yaw
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn(f"Unable to retrieve vehicle location (attempt {attempt}/{max_attempts}")
                if attempt < max_attempts:
                    time.sleep(0.1)
        raise Exception(f"Unable to retrieve vehicle location after {max_attempts} attempts")

    def update(self) -> None:
        self.target_speed = self._get_target_speed()
        self.speed_err = self.target_speed - self.speed
        self.cte = self._calc_lateral_error()  # cross-track error

    def destroy(self) -> None:
        self._vehicle_status_subscriber.unregister()
        self._vehicle_info_subscriber.unregister()
        self._world_info_subscriber.unregister()

    def _get_veh_pos(self, center_pos: carla.Location, heading: float) -> carla.Location:
        rear_axle_pos_offset = self.veh_info.rear_axle_pos_offset
        x = center_pos.x - rear_axle_pos_offset * math.cos(heading)
        y = center_pos.y - rear_axle_pos_offset * math.sin(heading)
        return carla.Location(x, y, center_pos.z)

    def _get_target_speed(self) -> float:
        cur_pose = self.ego_location
        closest_node = None
        min_distance = float('inf')
        # for node in self.planner.path:
        #     dist = node.waypoint.transform.location.distance(cur_pose)
        #     if dist < min_distance:
        #         min_distance = dist
        #         closest_node = node
        if closest_node is None:
            # print('[WARN] The path is empty')
            return 0
        return closest_node.speed_limit

    def _calc_lateral_error(self) -> float:
        return 0
        # use the center position intead of the rear axle center position
        # as it works better with PID steering controller
        cur_pose = self.sim.ego_car.get_transform().location
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
        return ((b.x - a.x)*(p.y - a.y) - (b.y - a.y)*(p.x - a.x)) > 0
