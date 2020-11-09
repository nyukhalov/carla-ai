import time

import carla
import pygame as pg

from carla_ai.state_updater import StateUpdater
from carla_ai import msg


class Debugger(object):
    def __init__(self, debug_helper: carla.DebugHelper, state_updater: StateUpdater):
        self._state_updater = state_updater
        self._debug_helper: carla.DebugHelper = debug_helper
        self._last_update_time = self._timestamp_now_ms()

    def tick(self, clock: pg.time.Clock):
        lifetime = 5
        now = self._timestamp_now_ms()
        if now > self._last_update_time + (lifetime * 1000) - 50:
            color = carla.Color(0, 0, 200, 0)
            self._last_update_time = now
            # draw_bbox(self.debug, veh)
            # self._draw_position_vray(lifetime=lifetime)

            if self._state_updater.goal:
                g = self._state_updater.goal.pose
                from_loc = carla.Location(g.position.x, -g.position.y, g.position.z)
                to_loc = carla.Location(from_loc.x, from_loc.y, from_loc.z + 100)
                self._debug_helper.draw_line(
                    from_loc, to_loc, thickness=1.0, color=carla.Color(40, 255, 0), life_time=lifetime
                )

            path = self._state_updater.path
            for idx, wp in enumerate(path):
                if idx % 10 == 0:
                    self._draw_waypoint(wp, color, lifetime=lifetime)

    def _timestamp_now_ms(self) -> int:
        return time.monotonic_ns() // 1000000

    def _draw_bbox(self, obj):
        bbox_location = obj.get_transform().location + obj.bounding_box.location
        self._debug_helper.draw_box(
            carla.BoundingBox(bbox_location, obj.bounding_box.extent),
            obj.get_transform().rotation,
            0.05,  # thickness
            carla.Color(255, 0, 0, 0),
            5  # lifetime [sec]
        )

    def _draw_position_vray(self, lifetime: float = 0):
        from_loc = self._state_updater.ego_location
        to_loc = carla.Location(from_loc.x, from_loc.y, from_loc.z + 3)
        color = carla.Color(0, 200, 0)
        self._debug_helper.draw_line(from_loc, to_loc, thickness=0.01, color=color, life_time=lifetime)

    def _draw_waypoint(self, wp: msg.WaypointWithSpeedLimit, color: carla.Color, lifetime: float = 0):
        loc = carla.Location(wp.x, wp.y, wp.z + 0.5)
        rotation = carla.Rotation(0, 0, 0)
        inner_size = 0.04
        outer_size = 0.068
        self._debug_helper.draw_box(
            carla.BoundingBox(loc, carla.Vector3D(inner_size, inner_size, inner_size)),
            rotation,
            thickness=0.06,
            color=color,
            life_time=lifetime
        )
        self._debug_helper.draw_box(
            carla.BoundingBox(loc, carla.Vector3D(outer_size, outer_size, outer_size)),
            rotation,
            thickness=0.01,
            color=carla.Color(0, 0, 0, 0),
            life_time=lifetime
        )
