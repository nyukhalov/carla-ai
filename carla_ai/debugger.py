import time

import carla
import pygame as pg

from .av.planner import Planner
from .sim import Simulation
from carla_ai.state_updater import StateUpdater


class Debugger(object):
    def __init__(self, sim: Simulation, planner: Planner, state_updater: StateUpdater):
        self.sim = sim
        self.planner = planner
        self.state_updater = state_updater
        self.debug: carla.DebugHelper = self.sim.world.debug
        self.last_update_time = self._timestamp_now_ms()

    def tick(self, clock: pg.time.Clock):
        lifetime = 0.1
        now = self._timestamp_now_ms()
        if now > self.last_update_time + lifetime * 1000:
            color = carla.Color(0, 0, 200, 0)
            self.last_update_time = now
            # visualize a few nearest waypoints
            path = self.planner.path or []
            for node in path:
                self._draw_waypoint(node.waypoint, color, lifetime=lifetime)
            # draw_bbox(self.debug, veh)
            self._draw_position_vray(lifetime=lifetime)

    def _timestamp_now_ms(self) -> int:
        return time.monotonic_ns() // 1000000

    def _draw_bbox(self, obj):
        bbox_location = obj.get_transform().location + obj.bounding_box.location
        self.debug.draw_box(
            carla.BoundingBox(bbox_location, obj.bounding_box.extent),
            obj.get_transform().rotation,
            0.05,  # thickness
            carla.Color(255, 0, 0, 0),
            5  # lifetime [sec]
        )

    def _draw_position_vray(self, lifetime: float = 0):
        from_loc = self.state_updater.ego_location
        to_loc = carla.Location(from_loc.x, from_loc.y, from_loc.z + 3)
        color = carla.Color(0, 200, 0)
        self.debug.draw_line(from_loc, to_loc, thickness=0.01, color=color, life_time=lifetime)

    def _draw_waypoint(self, wp: carla.Waypoint, color: carla.Color, lifetime: float = 0):
        loc = wp.transform.location
        loc.z += 0.5
        inner_size = 0.04
        outer_size = 0.068
        rotation = wp.transform.rotation
        self.debug.draw_box(
            carla.BoundingBox(loc, carla.Vector3D(inner_size, inner_size, inner_size)),
            rotation,
            0.06,
            color,
            lifetime
        )
        self.debug.draw_box(
            carla.BoundingBox(loc, carla.Vector3D(outer_size, outer_size, outer_size)),
            rotation,
            0.01,
            carla.Color(0, 0, 0, 0),
            lifetime
        )
