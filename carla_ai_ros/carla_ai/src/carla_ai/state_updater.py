import math
from typing import Tuple

from shapely.geometry import Point, LineString
import carla

from carla_ai.av.model import WaypointWithSpeedLimit, VehicleInfo


class StateUpdater(object):
    def __init__(self):
        self.steer = 0.0
        self.steering_wheel_angle = 0.0
        self.speed = 0.0
        self.target_speed = 0.0
        self.speed_err = 0.0
        self.cte = 0.0
        self.ego_location = None
        self.ego_heading = 0.0
        self.ego_vel: carla.Vector3D = carla.Vector3D(0, 0, 0)
        self.ego_acc: carla.Vector3D = carla.Vector3D(0, 0, 0)

        self.veh_info = self._init_vehicle_info()

        self.update()

    def _init_vehicle_info(self):
        # wheels = self.sim.ego_car.get_physics_control().wheels
        # max_steer_angle = math.radians(wheels[0].max_steer_angle)
        wheel_base = 0.0  # TODO: calc
        max_steer_angle = math.radians(40)
        rear_axle_pos_offset = 1

        # rlw: carla.WheelPhysicsControl = wheels[2]  # rear left wheel
        # rrw: carla.WheelPhysicsControl = wheels[3]  # rear right wheel

        # center_pos: carla.Location = self.sim.ego_car.get_transform().location

        # l_wheel_pos = carla.Location(rlw.position.x / 100, rlw.position.y / 100, rlw.position.z / 100)
        # r_wheel_pos = carla.Location(rrw.position.x / 100, rrw.position.y / 100, rrw.position.z / 100)

        # dist_between_rear_wheels = l_wheel_pos.distance(r_wheel_pos)
        # rear_axle_pos_offset_1 = math.sqrt(l_wheel_pos.distance(center_pos)**2 - (dist_between_rear_wheels/2)**2)
        # rear_axle_pos_offset_2 = math.sqrt(r_wheel_pos.distance(center_pos)**2 - (dist_between_rear_wheels/2)**2)
        # rear_axle_pos_offset = (rear_axle_pos_offset_1 + rear_axle_pos_offset_2) / 2

        return VehicleInfo(
            max_steer_angle,
            wheel_base,
            rear_axle_pos_offset
        )

    def update(self) -> None:
        # self.ego_heading = math.radians(self.sim.ego_car.get_transform().rotation.yaw)
        self.ego_heading: float = 0
        self.ego_location: carla.Location = self._calc_ego_location(self.ego_heading)
        # self.ego_vel = self.sim.ego_car.get_velocity()
        # self.ego_acc = self.sim.ego_car.get_acceleration()

        self.steer = self._calc_steer_angle()
        self.steering_wheel_angle = self._calc_steering_wheel_angle()
        # self.speed = 3.6 * math.sqrt(self.ego_vel.x ** 2 + self.ego_vel.y ** 2)
        self.speed = 0
        self.target_speed = self._get_target_speed()
        self.speed_err = self.target_speed - self.speed
        self.cte = self._calc_lateral_error()  # cross-track error

    def _calc_ego_location(self, heading: float) -> carla.Location:
        # return self._get_veh_pos(self.sim.ego_car.get_transform().location, heading)
        return carla.Location(0, 0, 0)

    def _get_veh_pos(self, center_pos: carla.Location, heading: float) -> carla.Location:
        rear_axle_pos_offset = self.veh_info.rear_axle_pos_offset
        x = center_pos.x - rear_axle_pos_offset * math.cos(heading)
        y = center_pos.y - rear_axle_pos_offset * math.sin(heading)
        return carla.Location(x, y, center_pos.z)

    def _calc_steering_wheel_angle(self):
        # steer_norm = self.sim.ego_car.get_control().steer
        steer_norm = 0
        max_angle = math.radians(450)  # a random number
        return steer_norm * max_angle

    def _calc_steer_angle(self):
        # steer_norm = self.sim.ego_car.get_control().steer
        steer_norm = 0
        return steer_norm * self.veh_info.max_steer_angle

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
            print('[WARN] The path is empty')
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
