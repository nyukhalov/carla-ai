from typing import List

import carla

from ..sim import Simulation


class Planner(object):
    def __init__(self, sim: Simulation):
        self.sim = sim
        self.path: List[carla.Waypoint] = []
        self.num_waypoints = 20

    def plan(self) -> None:
        if not self.path:
            self.path = [self.sim.map.get_waypoint(self.sim.ego_car.get_location(), lane_type=carla.LaneType.Driving)]
        self._update_path()

    def _update_path(self) -> None:
        cur_location = self.sim.ego_car.get_location()
        closest_wp_idx = None
        closest_distance = float('inf')
        for idx, wp in enumerate(self.path):
            dist = wp.transform.location.distance(cur_location)
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
            next_wp = self.path[-1].next(within_distance)[-1]
            self.path.append(next_wp)
