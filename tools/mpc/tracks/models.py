class Waypoint(object):
    def __init__(self, s: float, x: float, y: float, heading: float):
        assert(s >= 0, f"s must be positive, but got {s}")
        self.s = s
        self.x = x
        self.y = y
        self.heading = heading  # heading angle [in radians]
