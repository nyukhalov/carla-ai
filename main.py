import traceback
from typing import Tuple

import carla
import pygame as pg

from carla_ai.av.control import Controller
from carla_ai.av.planner import Planner
from carla_ai.debugger import Debugger
from carla_ai.hud import HUD
from carla_ai.sim import Simulation
from carla_ai.state_updater import StateUpdater


class Game(object):
    def __init__(self, world: carla.World, display_size: Tuple[int, int]):
        # set weather
        world.set_weather(carla.WeatherParameters.ClearNoon)

        self.simulation = Simulation(display_size, world)
        self.planner = Planner(self.simulation)
        self.state_updater = StateUpdater(self.simulation, self.planner)
        self.controller = Controller(self.simulation, self.state_updater)
        self.hud = HUD(display_size, self.simulation, self.planner, self.state_updater)
        self.debugger = Debugger(self.simulation, self.planner, self.state_updater)

    def tick(self, clock: pg.time.Clock):
        self.simulation.tick(clock)
        self.state_updater.update()
        self.planner.plan()
        self.controller.tick()
        self.hud.tick(clock)
        self.debugger.tick(clock)

    def render(self, display):
        self.simulation.render(display)
        self.hud.render(display)

    def destroy(self):
        self.simulation.destroy()


def main():
    W, H = [1280, 720]
    map_name = 'Town06'

    pg.init()
    pg.font.init()

    game = None
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)  # seconds
        client.load_world(map_name)

        game = Game(client.get_world(), (W, H))
        clock = pg.time.Clock()
        display = pg.display.set_mode((W, H), pg.HWSURFACE | pg.DOUBLEBUF)
        while True:
            clock.tick_busy_loop(60)
            game.tick(clock)
            game.render(display)
            pg.display.flip()
    except Exception as e:
        print('Game loop has been interrupted by exception', e)
        traceback.print_exc()
    finally:
        if game is not None:
            game.destroy()
        pg.quit()


if __name__ == '__main__':
    main()
