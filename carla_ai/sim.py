import random
import weakref
from typing import Tuple

import carla
import numpy as np
import pygame as pg
from carla import ColorConverter as cc


class Simulation(object):
    def __init__(self, display_size: Tuple[int, int], world: carla.World):
        self.world = world
        self.map = world.get_map()
        self.surface = None

        bp_lib = world.get_blueprint_library()

        ego_car_bp = random.choice(bp_lib.filter('vehicle.toyota.prius'))
        ego_car_transform = random.choice(world.get_map().get_spawn_points())

        self.ego_car = world.spawn_actor(ego_car_bp, ego_car_transform)
        #self.ego_car.set_autopilot(True)

        # add 3rd-person view camera
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(display_size[0]))
        cam_bp.set_attribute('image_size_y', str(display_size[1]))
        self.sensor = world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)),
            attach_to=self.ego_car,
            attachment_type=carla.AttachmentType.SpringArm
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: Simulation.parse_image(weak_self, image))

    def tick(self, clock: pg.time.Clock):
        pass

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    def destroy(self):
        actors = [
            self.sensor,
            self.ego_car
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
        self.surface = pg.surfarray.make_surface(array.swapaxes(0, 1))
