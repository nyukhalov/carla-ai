import collections
import time
from typing import Tuple

import pygame as pg

from carla_ai.measurement import Measurement
from carla_ai.state_updater import StateUpdater
from carla_ai.ui import font, Graph, WheelsIndicator, SteeringWheel


class HUD(object):
    def __init__(self, display_size: Tuple[int, int], state_updater: StateUpdater):
        self.display_size = display_size
        self.state_updater = state_updater

        self.text = None

        panel_width = 300
        self.background_surface = pg.Surface((panel_width, self.display_size[1]))
        self.background_surface.set_alpha(100)

        # initializing fonts
        self._font_mono = font.make_mono_font(14)

        # init history circular buffers
        self._history_time = 10  # seconds
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
        # self.speed_graph.set_ylim((0, 40))
        self.speed_graph.set_line_size(1)

        # init CTE graph (lateral error)
        cte_graph_size = (300, 200)
        cte_graph_pos_y = self.display_size[1] - cte_graph_size[1] - speed_graph_size[1]
        self.cte_graph = Graph((0, cte_graph_pos_y), cte_graph_size, (6, 5))
        self.cte_graph.set_title('CTE & SPD Error')
        self.cte_graph.set_xlabel('Time (sec)')
        self.cte_graph.set_ylabel('meters / km/h')
        self.cte_graph.set_xlim((-10, 0))
        #self.cte_graph.set_ylim((0, 5))
        self.cte_graph.set_line_size(1)

        # init controls graph
        controls_graph_size = (300, 200)
        controls_graph_pos_x = self.display_size[0] - controls_graph_size[0]
        controls_graph_pos_y = self.display_size[1] - controls_graph_size[1]
        self.controls_graph = Graph((controls_graph_pos_x, controls_graph_pos_y), controls_graph_size, (6, 5))
        self.controls_graph.set_title('Controls')
        self.controls_graph.set_xlabel('Time (sec)')
        self.controls_graph.set_ylabel('percentage')
        self.controls_graph.set_xlim((-10, 0))
        self.controls_graph.set_ylim((-1.1, 1.1))
        self.controls_graph.set_line_size(1)

        # init wheels indicator
        wheels_ind_size = (130, 200)
        wheels_ind_pos_x = self.display_size[0] - wheels_ind_size[0]
        wheels_ind_pos_y = self.display_size[1] - wheels_ind_size[1] - controls_graph_size[1]
        self.wheels_ind = WheelsIndicator((wheels_ind_pos_x, wheels_ind_pos_y), wheels_ind_size)

        # init steering wheel
        steering_wheel_size = (170, 200)
        steering_wheel_pos = (self.display_size[0] - steering_wheel_size[0] - wheels_ind_size[0], wheels_ind_pos_y)
        self.steering_wheel = SteeringWheel(steering_wheel_pos, steering_wheel_size)

    def tick(self, clock: pg.time.Clock):
        max_len = 20

        speed = self.state_updater.speed
        target_speed = self.state_updater.target_speed
        cte = self.state_updater.cte
        speed_err = self.state_updater.speed_err
        ego_location = self.state_updater.ego_location
        ego_heading = self.state_updater.ego_heading
        ego_vel = self.state_updater.ego_vel
        ego_acc = self.state_updater.ego_acc
        steer = self.state_updater.steer
        map_name = self.state_updater.map_name

        throttle_cmd = self.state_updater.throttle_cmd
        steer_cmd = self.state_updater.steer_cmd

        timestamp = self._timestamp_now_ms()
        threshold = (1000 / self._history_samples_per_sec)
        if not self.measurement_history or self.measurement_history[-1].timestamp < timestamp - threshold:
            m = Measurement(timestamp, speed, target_speed, cte, speed_err, throttle_cmd, steer_cmd)
            self.measurement_history.append(m)

        self.text = [
            'Simulation',
            f'cFPS: {clock.get_fps():.0f}',
            f'Map:  {map_name}',
            '',
            'Vehicle State',
            self._format_text_item(f'cur_spd: {speed:.3f}', 'km/h', max_len)        + '  ' + self._format_text_item(f'vx: {ego_vel.x:.3f}', 'm/s', max_len),
            self._format_text_item(f'tar_spd: {target_speed:.3f}', 'km/h', max_len) + '  ' + self._format_text_item(f'vy: {ego_vel.y:.3f}', 'm/s', max_len),
            self._format_text_item(f'spd_err: {speed_err:.3f}', 'km/h', max_len)    + '  ' + self._format_text_item(f'ax: {ego_acc.x:.3f}', 'm/s2', max_len),
            self._format_text_item(f'cte:     {cte:.3f}', 'm', max_len)             + '  ' + self._format_text_item(f'ay: {ego_acc.y:.3f}', 'm/s2', max_len),
            self._format_text_item(f'steer:   {steer:.3f}', 'rad', max_len),
            '',
            'Localization:',
            self._format_text_item(f'x:   {ego_location.x:.3f}', 'm', max_len),
            self._format_text_item(f'y:   {ego_location.y:.3f}', 'm', max_len),
            self._format_text_item(f'yaw: {ego_heading:.3f}', 'rad', max_len),
            ''
        ]

    def _timestamp_now_ms(self) -> int:
        return time.time_ns() // 1000000

    def _format_text_item(self, text: str, r_suffix: str, max_len: int):
        to_fill = max_len - len(text) - len(r_suffix)
        return text + ' ' * to_fill + r_suffix

    def render(self, display):
        # tinted background
        display.blit(self.background_surface, (0, 0))

        # text items
        v_offset = 4
        for text_item in self.text:
            surface = self._font_mono.render(text_item, True, (255, 255, 255))
            display.blit(surface, (8, v_offset))
            v_offset += 18

        # speed graph
        now = self._timestamp_now_ms()
        xs = []
        speed_hist = []
        target_speed_hist = []
        cte_hist = []
        speed_err_hist = []
        throttle_hist = []
        steer_hist = []
        for m in self.measurement_history:
            t = (m.timestamp - now) / 1000  # seconds
            if t < -10:
                continue
            xs.append(t)
            speed_hist.append(m.speed)
            target_speed_hist.append(m.target_speed)
            cte_hist.append(m.lateral_error)
            speed_err_hist.append(m.speed_error)
            throttle_hist.append(m.throttle_cmd)
            steer_hist.append(m.steer_cmd)
        self.speed_graph.render(display, xs, speed_hist, pg.Color(0, 200, 0), target_speed_hist, pg.Color(200, 0, 0))
        self.cte_graph.render(display, xs, cte_hist, pg.Color(0, 200, 0), speed_err_hist, pg.Color(200, 0, 0))
        self.controls_graph.render(display, xs, steer_hist, pg.Color(0, 200, 0), throttle_hist, pg.Color(200, 0, 0))

        # wheels indicator
        turn_angle = self.state_updater.steer
        self.wheels_ind.render(display, turn_angle)

        # steering wheel
        self.steering_wheel.render(display, self.state_updater.steering_wheel_angle)
