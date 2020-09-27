from carla_msgs.msg import CarlaEgoVehicleControl
import rospy
import random


class RosController(object):
    def __init__(self, role_name: str):
        self._vehicle_control_publisher = rospy.Publisher(f"/carla/{role_name}/vehicle_control_cmd",
                                                          CarlaEgoVehicleControl, queue_size=1)

    def tick(self):
        control = CarlaEgoVehicleControl()
        control.throttle = 0
        control.steer = random.random()
        control.brake = 0
        control.hand_brake = False
        self._vehicle_control_publisher.publish(control)

    def destroy(self):
        self._vehicle_control_publisher.unregister()
