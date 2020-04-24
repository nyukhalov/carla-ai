class Measurement(object):
    def __init__(self, timestamp: int, speed: float, target_speed: float, lateral_error: float, speed_error: float, throttle: float, steer: float):
        self.timestamp = timestamp
        self.speed = speed
        self.target_speed = target_speed
        self.lateral_error = lateral_error
        self.speed_error = speed_error
        self.throttle = throttle
        self.steer = steer
