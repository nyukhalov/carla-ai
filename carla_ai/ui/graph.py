from typing import Tuple, Optional

import numpy as np
import pygame as pg

from ..draw import round_rect
from ..ui import font


class Graph(object):
    def __init__(self,
            pos: Tuple[int, int],
            size: Tuple[int, int],
            grid: Tuple[int, int],
            title: Optional[str] = None,
            xlabel: Optional[str] = None,
            ylabel: Optional[str] = None,
            xlim: Optional[Tuple[int, int]] = None,
            ylim: Optional[Tuple[int, int]] = None,
            line_size: Optional[int] = None
        ):
        """
        @param: lookback_time: max time in sec that will be displayed on the x-axis
        """
        self.title = title
        self.pos = pos
        self.size = size
        self.grid = grid
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.xlim = xlim
        self.ylim = ylim
        self.line_size = line_size
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

    def set_line_size(self, line_size: int):
        self.line_size = line_size

    def _get_xlim(self, xs) -> Tuple[int, int]:
        if self.xlim:
            return self.xlim
        return (np.amin(xs), np.amax(xs))

    def _get_ylim(self, yys) -> Tuple[int, int]:
        if self.ylim:
            return self.ylim
        # else find min/max data values
        min_val = float('inf')
        max_val = -float('inf')
        for ys in yys:
            for y in ys:
                min_val = min(min_val, y)
                max_val = max(max_val, y)

        diff = max_val - min_val
        offset = diff * 0.1 if diff == 0 else 0.1
        y_hi = max_val
        y_lo = min_val
        return (y_lo - offset, y_hi + offset)


    def render(self, display, *data):
        """
        Render the provided timestamped data on the given display
        @param: display: a display the data will be rendered on
        @param: data: a sequence of xs, ys1, color1, [ys2, color2]
        """

        if len(data) < 2:
            return

        data = np.asarray(data)
        xs = data[0]
        yss = data[1::2]
        colors = data[2::2]

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

        x_lo, x_hi = self._get_xlim(xs)
        y_lo, y_hi = self._get_ylim(yss)

        # axes
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
        for cut_no in range(self.grid[0]):
            # render axis cuts
            num_intervals = self.grid[0] - 1
            x = left_offset + (axis_width / num_intervals) * cut_no
            pg.draw.line(self.surface, self.border_color, (x, axis_y), (x, axis_y + cut_len), 1)
            # render labels
            v = x_lo + cut_no * (x_hi - x_lo) / num_intervals
            label = f'{v:.1f}'
            fnt_surface = self._font_mono.render(label, True, self.border_color)
            fnt_hwidth = fnt_surface.get_width() // 2
            self.surface.blit(fnt_surface, (x - fnt_hwidth, axis_y + 10))

        # y-axis
        pg.draw.line(self.surface, self.border_color, (left_offset, top_offset), (left_offset, axis_y), 1)
        for cut_no in range(self.grid[1]):
            # render axis cuts
            num_intervals = self.grid[1] - 1
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

        h_res = abs(x_hi - x_lo) / axis_width # horizontal resolution
        v_res = abs(y_hi - y_lo) / axis_height # vertical resolution

        for ys, color in zip(yss, colors):
            for x1_val, x2_val, y1_val, y2_val in zip(xs[:-1], xs[1:], ys[:-1], ys[1:]):
                x1 = left_offset + int((x1_val - x_lo) / h_res)
                x2 = left_offset + int((x2_val - x_lo) / h_res)
                y1 = top_offset + axis_height - int((y1_val - y_lo) / v_res)
                y2 = top_offset + axis_height - int((y2_val - y_lo) / v_res)

                pg.draw.line(self.surface, color, (x1,y1), (x2,y2), self.line_size or 2)

        # update display
        display.blit(self.surface, self.pos)

