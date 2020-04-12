from typing import Tuple
import pygame as pg
from ..draw import round_rect

class Graph(object):
    def __init__(self, pos: Tuple[int, int], size: Tuple[int, int]):
        self.pos = pos
        self.size = size
        self.surface = pg.Surface(size)

        #self.surface.set_alpha(100)
        #self.surface.fill((200, 200, 200))

    def render(self, display):
        #round_rect(
        #    self.surface,
        #    pg.Rect(0, 0, self.size[0], self.size[1]),
        #    (200,200,200),
        #    rad=6, border=1,
        #    inside=(30,30,30)
        #)
        pg.draw.rect(self.surface, (0,255,0), pg.Rect(0, 0, 100, 100))
        display.blit(self.surface, self.pos)

