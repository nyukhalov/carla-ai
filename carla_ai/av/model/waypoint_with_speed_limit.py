from carla import Waypoint


class WaypointWithSpeedLimit(object):
    def __init__(self, waypoint: Waypoint, speed_limit: float):
        assert speed_limit >= 0, f'Speed limit must be positive, but got {speed_limit}'
        self.waypoint = waypoint
        self.speed_limit = speed_limit  # km / h

    @property
    def x(self):
        return self.waypoint.transform.location.x

    @property
    def y(self):
        return self.waypoint.transform.location.y