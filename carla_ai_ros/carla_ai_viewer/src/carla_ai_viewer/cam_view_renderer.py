from typing import Optional
import numpy as np
import pygame as pg
import rospy
from sensor_msgs import msg


class CamViewRenderer(object):
    def __init__(self, role_name: str):
        self._surface: Optional[pg.Surface] = None
        self._image_subscriber = rospy.Subscriber(
            f"/carla/{role_name}/camera/rgb/view/image_color",
            msg.Image,
            self._on_view_image
        )

    def tick(self, clock: pg.time.Clock) -> None:
        pass

    def render(self, display: pg.display) -> None:
        if self._surface is not None:
            display.blit(self._surface, (0, 0))

    def destroy(self):
        self._image_subscriber.unregister()

    def _on_view_image(self, image: msg.Image):
        """
        Callback when receiving a camera image
        """
        array = np.frombuffer(image.data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self._surface = pg.surfarray.make_surface(array.swapaxes(0, 1))
