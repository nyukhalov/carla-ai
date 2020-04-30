import math
from typing import Tuple

from shapely.geometry import Point, LineString
import carla

from carla_ai.av import Planner
from carla_ai.av.model import WaypointWithSpeedLimit
from carla_ai.sim import Simulation


class StateUpdater(object):
    def __init__(self, sim: Simulation, planner: Planner):
        self.sim = sim
        self.planner = planner
        self.steer = 0.0
        self.steering_wheel_angle = 0.0
        self.speed = 0.0
        self.target_speed = 0.0
        self.speed_err = 0.0
        self.cte = 0.0
        self.ego_location = None
        self.ego_heading = 0.0
        self.ego_vel = 0.0
        self.ego_acc = 0.0

        self.max_steer_deg = self.sim.ego_car.get_physics_control().wheels[0].max_steer_angle

        self.update()

    def update(self):
        ego_transform = self.sim.ego_car.get_transform()
        self.ego_location = self._calc_ego_location()
        self.planner.ego_location = self.ego_location  # a hack to push ego location to planner
        self.ego_heading = ego_transform.rotation.yaw
        self.ego_vel = self.sim.ego_car.get_velocity()
        self.ego_acc = self.sim.ego_car.get_acceleration()

        self.steer = self._calc_steer_angle()
        self.steering_wheel_angle = self._calc_steering_wheel_angle()
        self.speed = 3.6 * math.sqrt(self.ego_vel.x ** 2 + self.ego_vel.y ** 2)
        self.target_speed = self._get_target_speed()
        self.speed_err = self.target_speed - self.speed
        self.cte = self._calc_lateral_error()  # cross-track error

    def _calc_steering_wheel_angle(self):
        steer_norm = self.sim.ego_car.get_control().steer
        max_angle = math.radians(450)  # a random number
        return steer_norm * max_angle

    def _calc_steer_angle(self):
        steer_norm = self.sim.ego_car.get_control().steer
        return steer_norm * math.radians(self.max_steer_deg)

    def _calc_ego_location(self):
        # This list should have 4 elements, where
        # index 0 corresponds to the front left wheel,
        # index 1 corresponds to the front right wheel,
        # index 2 corresponds to the back left wheel and
        # index 3 corresponds to the back right wheel.
        wheels = self.sim.ego_car.get_physics_control().wheels
        rlw: carla.WheelPhysicsControl = wheels[2]  # rear left wheel
        rrw: carla.WheelPhysicsControl = wheels[3]  # rear right wheel

        # divide by 100 because the wheels position and radius are in cm
        wheel_radius = rrw.radius / 100
        rear_axle_center_pos = (rlw.position + rrw.position) / (2 * 100)

        return carla.Location(rear_axle_center_pos.x, rear_axle_center_pos.y, rear_axle_center_pos.z - wheel_radius)

    def _get_target_speed(self) -> float:
        cur_pose = self.ego_location
        closest_node = None
        min_distance = float('inf')
        for node in self.planner.path:
            dist = node.waypoint.transform.location.distance(cur_pose)
            if dist < min_distance:
                min_distance = dist
                closest_node = node
        return closest_node.speed_limit if closest_node is not None else 0

    def _calc_lateral_error(self) -> float:
        cur_pose = self.ego_location
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
