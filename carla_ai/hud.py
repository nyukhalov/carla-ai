import collections
import math
import time
from typing import Tuple

import pygame as pg

from carla_ai.ui import font, Graph
from .measurement import Measurement
from .sim import Simulation


class HUD(object):
    def __init__(self, display_size: Tuple[int, int], sim: Simulation):
        self.display_size = display_size
        self.sim = sim
        self.world = sim.world

        # cache the map, as calling the method inside the tick/render method significantly reduces FPS
        self.map = self.world.get_map()

        self.text = None
        self._server_clock = pg.time.Clock()

        self.world.on_tick(self.on_world_tick)

        panel_width = 300
        self.background_surface = pg.Surface((panel_width, self.display_size[1]))
        self.background_surface.set_alpha(100)

        # initializing fonts
        self._font_mono = font.make_mono_font(14)

        # init history circular buffers
        self._history_time = 10 # seconds
        self._history_samples_per_sec = 10
        self.measurement_history = collections.deque(maxlen=self._history_time * self._history_samples_per_sec)

        # init speed graph
        speed_graph_size = (300, 200)
        speed_graph_pos_y = self.display_size[1] - speed_graph_size[1]
        self.speed_graph = Graph((0, speed_graph_pos_y), speed_graph_size, (6, 5))
        self.speed_graph.set_title('Speed')
        self.speed_graph.set_xlabel('Time (sec)')
        self.speed_graph.set_ylabel('km/h')
        self.speed_graph.set_xlim((-10, 0))
        self.speed_graph.set_ylim((0, 40))
        self.speed_graph.set_line_size(1)

    def on_world_tick(self, timestamp):
        self._server_clock.tick()

    def tick(self, clock: pg.time.Clock):
        max_len = 18
        ego_transform = self.sim.ego_car.get_transform()
        ego_location = ego_transform.location
        ego_heading = ego_transform.rotation.yaw
        ego_vel = self.sim.ego_car.get_velocity()
        ego_acc = self.sim.ego_car.get_acceleration()

        speed = 3.6 * math.sqrt(ego_vel.x**2 + ego_vel.y**2)

        timestamp = self._timestamp_now_ms()
        threshold = (1000 / self._history_samples_per_sec)
        if not self.measurement_history or self.measurement_history[-1].timestamp < timestamp - threshold:
            m = Measurement(timestamp, speed)
            self.measurement_history.append(m)

        self.text = [
            'Simulation',
            f'sFPS: {self._server_clock.get_fps():.0f}',
            f'cFPS: {clock.get_fps():.0f}',
            f'Map:  {self.map.name}',
            '',
            'Vehicle State',
            self._format_text_item(f'speed: {speed:.3f}', 'km/h', max_len),
            self._format_text_item(f'vx:    {ego_vel.x:.3f}', 'm/s', max_len),
            self._format_text_item(f'vy:    {ego_vel.y:.3f}', 'm/s', max_len),
            self._format_text_item(f'ax:    {ego_acc.x:.3f}', 'm/s2', max_len),
            self._format_text_item(f'ay:    {ego_acc.y:.3f}', 'm/s2', max_len),
            '',
            'Localization:',
            self._format_text_item(f'x:   {ego_location.x:.3f}', 'm', max_len),
            self._format_text_item(f'y:   {ego_location.y:.3f}', 'm', max_len),
            self._format_text_item(f'yaw: {ego_heading:.3f}', 'deg', max_len),
            ''
        ]

    def _timestamp_now_ms(self):
        return time.time_ns() / 1000000

    def _format_text_item(self, text: str, r_suffix: str, max_len: int):
        to_fill = max_len - len(text) - len(r_suffix)
        return text + ' '*to_fill + r_suffix


    def render(self, display):
        # tinted background
        display.blit(self.background_surface, (0,0))

        # text items
        v_offset = 4
        for text_item in self.text:
            surface = self._font_mono.render(text_item, True, (255,255,255))
            display.blit(surface, (8, v_offset))
            v_offset += 18

        # speed graph
        now = self._timestamp_now_ms()
        xs = []
        ys = []
        for m in self.measurement_history:
            t = (m.timestamp - now) / 1000 # seconds
            if t < -10:
                continue
            xs.append(t)
            ys.append(m.speed)
        self.speed_graph.render(display, xs, ys, pg.Color(0,200,0))