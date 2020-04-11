import random
from typing import Tuple
import weakref
import time
import carla
from carla import ColorConverter as cc
import pygame
import numpy as np

def draw_bbox(debug, obj):
    bbox_location = obj.get_transform().location + obj.bounding_box.location
    debug.draw_box(
        carla.BoundingBox(bbox_location, obj.bounding_box.extent),
        obj.get_transform().rotation,
        0.05, # thickness
        carla.Color(255, 0, 0, 0),
        5 # lifetime [sec]
    )

def draw_waypoint(debug, wp: carla.Waypoint, color: carla.Color, life_time: float = 0):
    loc = wp.transform.location
    loc.z += 0.5
    inner_size = 0.04
    outer_size = 0.068
    rotation = wp.transform.rotation
    debug.draw_box(
        carla.BoundingBox(loc, carla.Vector3D(inner_size, inner_size, inner_size)),
        rotation,
        0.06,
        color,
        life_time
    )
    debug.draw_box(
        carla.BoundingBox(loc, carla.Vector3D(outer_size, outer_size, outer_size)),
        rotation,
        0.01,
        carla.Color(0, 0, 0, 0),
        life_time
    )

class HUD(object):
    def __init__(self, display_size: Tuple[int, int], world: carla.World):
        self.display_size = display_size
        self.world = world

        # cache the map, as calling the method inside the tick/render method significantly reduces FPS
        self.map = world.get_map()

        self.text = None
        self._server_clock = pygame.time.Clock()

        self.world.on_tick(self.on_world_tick)

        self.background_surface = pygame.Surface((220, self.display_size[1]))
        self.background_surface.set_alpha(100)

        # initializing fonts
        font_name = 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)

    def on_world_tick(self, timestamp):
        self._server_clock.tick()

    def tick(self, clock: pygame.time.Clock):
        self.text = [
            f'Server FPS: {self._server_clock.get_fps()}',
            f'Client FPS: {clock.get_fps()}',
            f'Map:        {self.map.name}'
        ]

    def render(self, display):
        # tinted background
        display.blit(self.background_surface, (0,0))

        # text items
        v_offset = 4
        for text_item in self.text:
            surface = self._font_mono.render(text_item, True, (255,255,255))
            display.blit(surface, (8, v_offset))
            v_offset += 18


class Game(object):
    def __init__(self, world: carla.World, display_size: Tuple[int, int]):
        self.surface = None
        self.world = world
        self.debug = world.debug
        self.display_size = display_size

        self.hud = HUD(display_size, world)

        # set weather
        world.set_weather(carla.WeatherParameters.ClearNoon)

        self.cur_map = world.get_map()

        bp_lib = world.get_blueprint_library()
        veh_bp = random.choice(bp_lib.filter('vehicle.toyota.prius'))
        transform = random.choice(self.cur_map.get_spawn_points())

        self.veh = world.spawn_actor(veh_bp, transform)
        self.veh.set_autopilot(True)

        # add 3rd-person view camera
        weak_self = weakref.ref(self)
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(self.display_size[0]))
        cam_bp.set_attribute('image_size_y', str(self.display_size[1]))
        self.sensor = world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)),
            attach_to=self.veh,
            attachment_type=carla.AttachmentType.SpringArm
        )
        self.sensor.listen(lambda image: Game.parse_image(weak_self, image))

    def tick(self, clock: pygame.time.Clock):
        self.hud.tick(clock)

    def render(self, display):
        # visualize a few nearest waypoints
        #nearest_wp = self.cur_map.get_waypoint(self.veh.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        #draw_waypoint(self.debug, nearest_wp, carla.Color(255, 0, 0, 0), life_time=-1)
        #next_wps = nearest_wp.next_until_lane_end(1)
        #for wp in next_wps:
        #    draw_waypoint(self.debug, wp, carla.Color(0, 0, 200, 0), life_time=-1)

        #draw_bbox(self.debug, veh)

        if self.surface is not None:
            display.blit(self.surface, (0,0))
        self.hud.render(display)

    def destroy(self):
        actors = [
            self.sensor,
            self.veh
        ]
        for actor in actors:
            actor.destroy()


    @staticmethod
    def parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(cc.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))


def main():
    W, H = [1280, 720]
    map_name = 'Town02'

    pygame.init()
    pygame.font.init()

    game = None
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0) # seconds
        client.load_world(map_name)

        game = Game(client.get_world(), (W,H))
        clock = pygame.time.Clock()
        display = pygame.display.set_mode((W,H), pygame.HWSURFACE | pygame.DOUBLEBUF)
        while True:
            clock.tick_busy_loop(60)
            game.tick(clock)
            game.render(display)
            pygame.display.flip()
    except Exception as e:
        print('Game loop has been interrupted by exception', e)
    finally:
        if game is not None:
            game.destroy()
        pygame.quit()


if __name__=='__main__':
    main()
