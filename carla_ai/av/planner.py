from typing import List

import carla

from carla_ai.sim import Simulation
from carla_ai.av.model import WaypointWithSpeedLimit


class Planner(object):
    def __init__(self, sim: Simulation):
        self.sim = sim
        self.path: List[WaypointWithSpeedLimit] = []
        self.num_waypoints = 20
        self.speed_limit = 20  # 20 km/h

    def plan(self) -> None:
        if not self.path:
            closest_wp = self.sim.map.get_waypoint(self.sim.ego_car.get_location(), lane_type=carla.LaneType.Driving)
            self.path = [self._make_wp_with_speed_limit(closest_wp)]
        self._update_path()

    def _make_wp_with_speed_limit(self, wp: carla.Waypoint) -> WaypointWithSpeedLimit:
        return WaypointWithSpeedLimit(wp, self.speed_limit)

    def _update_path(self) -> None:
        cur_location = self.sim.ego_car.get_location()
        closest_wp_idx = None
        closest_distance = float('inf')
        for idx, node in enumerate(self.path):
            dist = node.waypoint.transform.location.distance(cur_location)
            if dist < closest_distance:
                closest_distance = dist
                closest_wp_idx = idx

        # if the car goes off the track, then reset the path
        if closest_distance > 5:
            self.path = []
            return

        # otherwise remove waypoints behind the car
        self.path = self.path[closest_wp_idx:]

        # add waypoints ahead the car
        within_distance = 2
        while len(self.path) < self.num_waypoints:
            next_wp = self.path[-1].waypoint.next(within_distance)[-1]
            self.path.append(self._make_wp_with_speed_limit(next_wp))
