from typing import Tuple
import math

import carla
import pygame as pg

from carla_ai.draw import round_rect


class WheelsIndicator(object):
    def __init__(self, pos: Tuple[int, int], size: Tuple[int, int]):
        self.pos = pos
        self.size = size
        self.surface = pg.Surface(size)
        self.border_color = (200, 200, 200)
        self.inner_color = (30, 30, 30)
        self.radius = 6
        self.border = 1
        self.h_offset = 30
        self.v_offset = 50

    def render(self, display: pg.Surface, turn_angle: float):
        """
        Render the indicator
        @param: turn_angle wheels turn angle in radians
        """
        # background w/ border
        round_rect(
            self.surface,
            pg.Rect(0, 0, self.size[0], self.size[1]),
            self.border_color,
            self.radius,
            self.border,
            self.inner_color
        )

        line_width = 1
        axle_width = self.size[0] - (2 * self.h_offset)
        frame_height = self.size[1] - (2 * self.v_offset)
        front_axle_y = self.v_offset
        rear_axle_y = self.v_offset + frame_height
        wheel_half_size = 18

        # front axle
        pg.draw.line(
            self.surface,
            self.border_color,
            (self.h_offset, front_axle_y),
            (self.h_offset + axle_width, self.v_offset),
            line_width
        )

        # rear axle
        pg.draw.line(
            self.surface,
            self.border_color,
            (self.h_offset, rear_axle_y),
            (self.h_offset + axle_width, rear_axle_y),
            line_width
        )

        # frame center
        pg.draw.line(
            self.surface,
            self.border_color,
            (self.h_offset + axle_width // 2, front_axle_y),
            (self.h_offset + axle_width // 2, rear_axle_y),
            line_width
        )

        for x_offset in [self.h_offset, self.h_offset + axle_width]:
            # front wheels
            pivot_p = carla.Vector2D(x_offset, front_axle_y)
            from_p = carla.Vector2D(x_offset, front_axle_y + wheel_half_size)
            to_p = carla.Vector2D(x_offset, front_axle_y - wheel_half_size)
            from_p = self._rotate_point(pivot_p, from_p, turn_angle)
            to_p = self._rotate_point(pivot_p, to_p, turn_angle)
            pg.draw.line(
                self.surface,
                self.border_color,
                (from_p.x, from_p.y),
                (to_p.x, to_p.y),
                10
            )

            # bottom wheels
            pg.draw.line(
                self.surface,
                self.border_color,
                (x_offset, rear_axle_y + wheel_half_size),
                (x_offset, rear_axle_y - wheel_half_size),
                10
            )

        # update display
        display.blit(self.surface, self.pos)

    def _rotate_point(self, center: carla.Vector2D, point: carla.Vector2D, angle: float):
        return carla.Vector2D(
            math.cos(angle) * (point.x - center.x) - math.sin(angle) * (point.y - center.y) + center.x,
            math.sin(angle) * (point.x - center.x) + math.cos(angle) * (point.y - center.y) + center.y
        )
