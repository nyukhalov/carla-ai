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

    g = Graph((100, 100), (400, 300), 10.0)
    g.set_title('Title')
    g.set_xlabel('Time (sec)')
    g.set_ylabel('meters')

    clock = pg.time.Clock()
    offset = 0
    while True:
        clock.tick_busy_loop(60)

        offset -= 0.02
        now = time.time_ns() // 1000000
        q = deque()
        for i in range(55):
            q.append((now - (i*200), 20*math.sin(offset+i/5)))
        q.reverse()

        g.render(display, q)
        pg.display.flip()

