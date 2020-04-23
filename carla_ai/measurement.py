class Measurement(object):
    def __init__(self, timestamp: int, speed: float, lateral_error: float):
        self.timestamp = timestamp
        self.speed = speed
        self.lateral_error = lateral_error
