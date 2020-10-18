from carla import Waypoint, Location


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

    @property
    def z(self):
        return self.waypoint.transform.location.z

    def distance(self, other) -> float:
        if type(other) == Location:
            return self.waypoint.transform.location.distance(other)
        if type(other) == Waypoint:
            return self.distance(other.transform.location)
        if type(other) == WaypointWithSpeedLimit:
            return self.distance(other.waypoint)
        raise ValueError(f'Unsupported type {type(other)}')
