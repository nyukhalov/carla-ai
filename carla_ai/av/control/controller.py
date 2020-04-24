from carla import VehicleControl

from .pid import PID
from carla_ai.sim import Simulation
from carla_ai.state_updater import StateUpdater
from carla_ai.util.math import clamp


class Controller(object):
    def __init__(self, sim: Simulation, state_updater: StateUpdater):
        self.sim = sim
        self.state_updater = state_updater
        self.steer_pid = PID(0.3, 0.0001, 3.0)
        self.throttle_pid = PID(0.2, 0.0001, 2.0)
        self.prev_steer = 0.0

    def tick(self) -> None:
        cte = self.state_updater.cte
        speed_err = self.state_updater.speed_err

        gain = 0.1
        steer = clamp(-1, self.steer_pid.update(cte), 1)
        steer = gain * steer + (1 - gain) * self.prev_steer
        self.prev_steer = steer
        brake = 0
        throttle = clamp(-1, self.throttle_pid.update(speed_err), 1)
        if throttle < 0:
            throttle = 0
            brake = -throttle

        vehicle_control = VehicleControl(throttle=throttle, steer=steer, brake=brake)
        self.sim.ego_car.apply_control(vehicle_control)
