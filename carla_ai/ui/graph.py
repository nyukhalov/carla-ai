from typing import Tuple
from collections import deque
import time
import pygame as pg
import numpy as np
from ..draw import round_rect
from ..ui import font

class Graph(object):
    def __init__(self, pos: Tuple[int, int], size: Tuple[int, int], lookback_time: float):
        """
        @param: lookback_time: max time in sec that will be displayed on the x-axis
        """
        self.pos = pos
        self.size = size
        self.lookback_time = lookback_time
        self.surface = pg.Surface(size)
        self.border_color = (200, 200, 200)
        self.inner_color = (30, 30, 30)
        self.radius = 6
        self.border = 1
        self._font_mono = font.make_mono_font(13)

    def render(self, display, data: deque):
        """
        Render the provided timestamped data on the given display
        @param: display: a display the data will be rendered on
        @param: data: a deque containing timestamped data to display.
                      the deque's item is a tuple of a timestamp (ms) and the value.
        """

        if len(data) < 2:
            return

        # background w/ border
        round_rect(
            self.surface,
            pg.Rect(0, 0, self.size[0], self.size[1]),
            self.border_color,
            self.radius,
            self.border,
            self.inner_color
        )

        # axis
        grid = (7, 5)
        offset = 32

        cut_len = 4
        axis_height = self.size[1] - offset - offset
        axis_width = self.size[0] - offset - offset
        axis_y = offset + axis_height

        # x-axis
        pg.draw.line(
            self.surface,
            self.border_color,
            (offset, axis_y),
            (offset + axis_width, axis_y),
            1
        )
        for cut_no in range(grid[0]):
            # render axis
            num_intervals = grid[0] - 1
            x = offset + (axis_width / num_intervals) * cut_no
            pg.draw.line(self.surface, self.border_color, (x, axis_y), (x, axis_y + cut_len), 1)
            # render labels
            t = (-self.lookback_time / num_intervals) * (grid[0] - cut_no - 1)
            label = f'{t:.1f}'
            fnt_surface = self._font_mono.render(label, True, self.border_color)
            fnt_hwidth = fnt_surface.get_width() // 2
            self.surface.blit(fnt_surface, (x - fnt_hwidth, axis_y + 10))

        # y-axis
        pg.draw.line(self.surface, self.border_color, (offset, offset), (offset, axis_y), 1)
        for cut_no in range(grid[1]):
            y = offset + (axis_height / (grid[1]-1)) * cut_no
            pg.draw.line(self.surface, self.border_color, (offset-cut_len, y), (offset, y), 1)


        # find min/max data values
        min_val = data[0][1]
        max_val = data[0][1]
        for ts, val in data:
            min_val = min(min_val, val)
            max_val = max(max_val, val)
        y_hi = max_val * 1.2
        y_lo = min_val * 1.2

        # x-labels
        # y-labels

        # data
        loopback_ms = int(self.lookback_time * 1000)

        h_res =  loopback_ms / axis_width # horizontal resolution
        v_res = abs(y_hi - y_lo) / axis_height # vertical resolution

        now_ms = time.time_ns() // 1000000 # ms
        last_ts = now_ms - loopback_ms # ms

        r_data = list(reversed(data))
        for (ts1, val1), (ts2, val2) in zip(r_data[:-1], r_data[1:]):
            if ts1 > now_ms or ts2 > now_ms:
                continue
            if ts1 < last_ts or ts2 < last_ts:
                break

            x1 = offset + int((ts1 - last_ts) / h_res)
            x2 = offset + int((ts2 - last_ts) / h_res)
            y1 = offset + axis_height - int((val1 - y_lo) / v_res)
            y2 = offset + axis_height - int((val2 - y_lo) / v_res)

            pg.draw.line(self.surface, (0,255,0), (x1,y1), (x2,y2), 1)

        # update display
        display.blit(self.surface, self.pos)

