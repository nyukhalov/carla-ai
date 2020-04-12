from typing import Tuple, Optional
from collections import deque
import time
import pygame as pg
import numpy as np
from ..draw import round_rect
from ..ui import font

class Graph(object):
    def __init__(self,
            pos: Tuple[int, int],
            size: Tuple[int, int],
            lookback_time: float,
            title: Optional[str] = None,
            xlabel: Optional[str] = None,
            ylabel: Optional[str] = None,
            xlim: Optional[Tuple[int, int]] = None,
            ylim: Optional[Tuple[int, int]] = None,
        ):
        """
        @param: lookback_time: max time in sec that will be displayed on the x-axis
        """
        self.title = title
        self.pos = pos
        self.size = size
        self.lookback_time = lookback_time
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.xlim = xlim
        self.ylim = ylim
        self.surface = pg.Surface(size)
        self.border_color = (200, 200, 200)
        self.inner_color = (30, 30, 30)
        self.radius = 6
        self.border = 1
        self._font_title = font.make_mono_font(20)
        self._font_label = font.make_mono_font(13)
        self._font_mono = font.make_mono_font(13)

    def set_xlabel(self, xlabel: str):
        self.xlabel = xlabel

    def set_ylabel(self, ylabel: str):
        self.ylabel = ylabel

    def set_title(self, title: str):
        self.title = title

    def set_xlim(self, xlim: Tuple[int, int]):
        self.xlim = xlim

    def set_ylim(self, ylim: Tuple[int, int]):
        self.ylim = ylim

    def _get_ylim(self, data) -> Tuple[int, int]:
        if self.ylim:
            return self.ylim
        # else find min/max data values
        min_val = data[0][1]
        max_val = data[0][1]
        for ts, val in data:
            min_val = min(min_val, val)
            max_val = max(max_val, val)
        y_hi = max_val * 1.2
        y_lo = min_val * 1.2
        return (y_lo, y_hi)


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

        # title
        if self.title:
            title_surface = self._font_title.render(self.title, True, self.border_color)
            title_x = self.size[0] // 2 - title_surface.get_width() // 2
            title_y = 8
            self.surface.blit(title_surface, (title_x, title_y))

        # xlabel
        if self.xlabel:
            fnt_surface = self._font_label.render(self.xlabel, True, self.border_color)
            xlabel_x = self.size[0] // 2 - fnt_surface.get_width() // 2
            xlabel_y = self.size[1] - fnt_surface.get_height() - 8
            self.surface.blit(fnt_surface, (xlabel_x, xlabel_y))

        # ylabel
        if self.ylabel:
            fnt_surface = self._font_label.render(self.ylabel, True, self.border_color)
            fnt_surface = pg.transform.rotate(fnt_surface, 90)
            ylabel_x = 6
            ylabel_y = self.size[1] // 2 - fnt_surface.get_height() // 2
            self.surface.blit(fnt_surface, (ylabel_x, ylabel_y))

        y_lo, y_hi = self._get_ylim(data)

        # axis
        grid = (7, 5)
        left_offset = 70
        right_offset = 20
        top_offset = 40
        bottom_offset = 50

        cut_len = 4
        axis_height = self.size[1] - top_offset - bottom_offset
        axis_width = self.size[0] - left_offset - right_offset
        axis_y = top_offset + axis_height

        # x-axis
        pg.draw.line(
            self.surface,
            self.border_color,
            (left_offset, axis_y),
            (left_offset + axis_width, axis_y),
            1
        )
        for cut_no in range(grid[0]):
            # render axis
            num_intervals = grid[0] - 1
            x = left_offset + (axis_width / num_intervals) * cut_no
            pg.draw.line(self.surface, self.border_color, (x, axis_y), (x, axis_y + cut_len), 1)
            # render labels
            t = (-self.lookback_time / num_intervals) * (grid[0] - cut_no - 1)
            label = f'{t:.1f}'
            fnt_surface = self._font_mono.render(label, True, self.border_color)
            fnt_hwidth = fnt_surface.get_width() // 2
            self.surface.blit(fnt_surface, (x - fnt_hwidth, axis_y + 10))

        # y-axis
        pg.draw.line(self.surface, self.border_color, (left_offset, top_offset), (left_offset, axis_y), 1)
        for cut_no in range(grid[1]):
            # render axis
            num_intervals = grid[1] - 1
            y = top_offset + (axis_height / num_intervals) * cut_no
            pg.draw.line(self.surface, self.border_color, (left_offset-cut_len, y), (left_offset, y), 1)
            # render labels
            v = y_hi - (cut_no * (y_hi - y_lo) / num_intervals)
            label = f'{v:.1f}'
            fnt_surface = self._font_mono.render(label, True, self.border_color)
            fnt_hheight = fnt_surface.get_height() // 2
            fnt_width = fnt_surface.get_width()
            self.surface.blit(fnt_surface, (left_offset - 10 - fnt_width, y - fnt_hheight))

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

            x1 = left_offset + int((ts1 - last_ts) / h_res)
            x2 = left_offset + int((ts2 - last_ts) / h_res)
            y1 = top_offset + axis_height - int((val1 - y_lo) / v_res)
            y2 = top_offset + axis_height - int((val2 - y_lo) / v_res)

            pg.draw.line(self.surface, (0,255,0), (x1,y1), (x2,y2), 1)

        # update display
        display.blit(self.surface, self.pos)

