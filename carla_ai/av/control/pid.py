class PID(object):
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = None
        self.total_error = 0.0

    def update(self, error: float) -> float:
        error_dot = error - (self.prev_error or error)
        self.prev_error = error
        self.total_error += error
        return (self.kp * error) + (self.ki * self.total_error) + (self.kd * error_dot)
