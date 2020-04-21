import time
import random
from typing import List
import pygame as pg
import carla
from .sim import Simulation

class Debugger(object):
    def __init__(self, sim: Simulation):
        self.sim = sim
        self.last_update_time = self._timestamp_now_ms()

    def tick(self, clock: pg.time.Clock):
        lifetime = 0.2
        now = self._timestamp_now_ms()
        if now > self.last_update_time + lifetime * 1000:
            color = carla.Color(0, 0, 200, 0)
            self.last_update_time = now
            # visualize a few nearest waypoints
            wps = self._get_next_wps(20)
            for wp in wps:
                self._draw_waypoint(wp, color, life_time=lifetime)
            #draw_bbox(self.debug, veh)

    def _get_next_wps(self, num_waypoints: int) -> List[carla.Waypoint]:
        within_distance = 2
        nearest_wp = self.sim.map.get_waypoint(self.sim.ego_car.get_location(), lane_type=carla.LaneType.Driving)
        wps = [nearest_wp]
        while len(wps) < num_waypoints:
            next_wp = wps[-1].next(within_distance)[-1]
            wps.append(next_wp)
        return wps

    def _timestamp_now_ms(self) -> int:
        return time.monotonic_ns() / 1000000

    def _draw_bbox(self, obj):
        bbox_location = obj.get_transform().location + obj.bounding_box.location
        self.sim.world.debug.draw_box(
            carla.BoundingBox(bbox_location, obj.bounding_box.extent),
            obj.get_transform().rotation,
            0.05, # thickness
            carla.Color(255, 0, 0, 0),
            5 # lifetime [sec]
        )

    def _draw_waypoint(self, wp: carla.Waypoint, color: carla.Color, life_time: float = 0):
        loc = wp.transform.location
        loc.z += 0.5
        inner_size = 0.04
        outer_size = 0.068
        rotation = wp.transform.rotation
        self.sim.world.debug.draw_box(
            carla.BoundingBox(loc, carla.Vector3D(inner_size, inner_size, inner_size)),
            rotation,
            0.06,
            color,
            life_time
        )
        self.sim.world.debug.draw_box(
            carla.BoundingBox(loc, carla.Vector3D(outer_size, outer_size, outer_size)),
            rotation,
            0.01,
            carla.Color(0, 0, 0, 0),
            life_time
        )