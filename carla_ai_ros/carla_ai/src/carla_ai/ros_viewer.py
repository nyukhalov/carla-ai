import traceback

import rospy
import pygame as pg
import carla
from carla_msgs.msg import CarlaWorldInfo
from carla_ai import CamViewRenderer, HUD, StateUpdater
from carla_ai.debugger import Debugger


class RosViewer(object):
    def __init__(self, role_name: str):
        # resolution should be similar to spawned camera with role-name 'view'
        W, H = [1920, 1080]

        pg.init()
        pg.font.init()

        self._display_size = (W, H)
        self._role_name = role_name
        self._renderer = CamViewRenderer(role_name)
        self._state_updater = StateUpdater(role_name)
        self._hud = HUD(self._display_size, self._state_updater)

        rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
        host = rospy.get_param("/carla/host", "127.0.0.1")
        port = rospy.get_param("/carla/port", 2000)
        timeout = rospy.get_param("/carla/timeout", 10)
        rospy.loginfo(f"CARLA world available. Trying to connect to {host}:{port}")
        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)
        carla_world = carla_client.get_world()
        rospy.loginfo("Connected to Carla.")
        self._debugger = Debugger(carla_world.debug, self._state_updater)

    def run(self) -> None:
        try:
            pg.display.set_caption("Carla AI Viewer")
            display = pg.display.set_mode(self._display_size, pg.HWSURFACE | pg.DOUBLEBUF)

            clock = pg.time.Clock()
            while not rospy.is_shutdown():
                clock.tick_busy_loop(60)
                self._tick(clock)
                self._render(display)
                pg.display.flip()
        except Exception as e:
            print('Game loop has been interrupted by exception', e)
            traceback.print_exc()
        finally:
            self._destroy()

    def _tick(self, clock: pg.time.Clock):
        self._renderer.tick(clock)
        self._hud.tick(clock)
        self._debugger.tick(clock)

    def _render(self, display: pg.display):
        self._renderer.render(display)
        self._hud.render(display)

    def _destroy(self):
        self._renderer.destroy()
        self._state_updater.destroy()
        pg.quit()
