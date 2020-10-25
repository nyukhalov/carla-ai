from typing import Tuple
import math

import pygame as pg

from carla_ai.draw import round_rect


class SteeringWheel(object):
    def __init__(self, pos: Tuple[int, int], size: Tuple[int, int]):
        self.pos = pos
        self.size = size
        self.surface = pg.Surface(self.size, pg.SRCALPHA)
        self.border_color = (200, 200, 200, 255)
        self.inner_color = (30, 30, 30, 255)
        self.radius = 6
        self.border = 1
        self.r = int(0.8 * min(self.size[0], self.size[1]) // 2)
        #self.wheel_surface = pg.Surface((self.r, self.r), pg.SRCALPHA)
        self.wheel_surface = pg.Surface(self.size, pg.SRCALPHA)

    def render(self, display: pg.Surface, steer_angle: float):
        # background w/ border
        round_rect(
            self.surface,
            pg.Rect(0, 0, self.size[0], self.size[1]),
            self.border_color,
            self.radius,
            self.border,
            self.inner_color
        )

        angle = -math.degrees(steer_angle)
        center = self.wheel_surface.get_rect().center

        pg.draw.circle(self.wheel_surface, self.border_color, center, self.r, 5)
        pg.draw.line(self.wheel_surface, self.border_color, (center[0] - self.r, center[1]), (center[0] + self.r, center[1]), 5)

        # rotate the surface
        surface_center = self.wheel_surface.get_rect().center
        rotated_surface = pg.transform.rotate(self.wheel_surface, angle)
        rect = rotated_surface.get_rect(center=surface_center)

        # update display
        self.surface.blit(rotated_surface, rect.topleft)
        display.blit(self.surface, self.pos)
