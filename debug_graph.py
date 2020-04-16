import time
import math
from collections import deque
import pygame as pg
from carla_ai.ui import Graph

if __name__=='__main__':
    pg.init()
    pg.font.init()

    W=1280
    H=720
    display = pg.display.set_mode((W,H), pg.HWSURFACE | pg.DOUBLEBUF)

    pos = (100, 100)
    size = (400, 300)
    grid = (5, 5)
    g = Graph(pos, size, grid, 10.0)
    g.set_title('Title')
    g.set_xlabel('Time (sec)')
    g.set_ylabel('meters')
    #g.set_xlim((-30, 30))
    #g.set_ylim((-30, 30))

    clock = pg.time.Clock()
    offset = 0
    while True:
        clock.tick_busy_loop(60)

        offset -= 0.02
        now = time.time_ns() // 1000000
        x = []
        y1 = []
        y2 = []
        for i in range(55):
            x.append(now - (i*200))
            y1.append(math.sin(offset+i/5))
            y2.append(math.cos(offset+i/5))

        g.render(display, x, y1, pg.Color(150,0,0), y2, pg.Color(0,0,150))
        pg.display.flip()

