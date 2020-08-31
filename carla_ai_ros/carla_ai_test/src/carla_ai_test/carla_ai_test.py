import rospy
from carla_ego_vehicle.carla_ego_vehicle import CarlaEgoVehicle


def main():
    """
    main function
    """
    rospy.loginfo("Creating ego vehicle")
    ego_vehicle = CarlaEgoVehicle()
    try:
        ego_vehicle.run()
    finally:
        rospy.loginfo("Shutting down")
        if ego_vehicle is not None:
            ego_vehicle.destroy()


if __name__ == "__main__":
    main()
