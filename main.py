import random
import time
import carla

def draw_bbox(debug, obj):
    bbox_location = obj.get_transform().location + obj.bounding_box.location
    debug.draw_box(
        carla.BoundingBox(bbox_location, obj.bounding_box.extent),
        obj.get_transform().rotation,
        0.05, # thickness
        carla.Color(255, 0, 0, 0),
        5 # lifetime [sec]
    )


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0) # seconds

    available_maps = client.get_available_maps()
    print('Available maps')
    for avail_map in available_maps:
        print(f'- {avail_map}')

    # set map
    world = client.load_world('Town02')

    # set weather
    world.set_weather(carla.WeatherParameters.ClearNoon)

    bp_lib = world.get_blueprint_library()
    cur_map = world.get_map()

    veh_bp = random.choice(bp_lib.filter('vehicle'))
    transform = random.choice(cur_map.get_spawn_points())

    veh = world.spawn_actor(veh_bp, transform)

    veh.set_autopilot(True)

    print(type(veh))
    print(dir(veh))

    spectator = world.get_spectator()
    transform = veh.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50), carla.Rotation(pitch=-90)))

    debug = world.debug
    while(True):
        draw_bbox(debug, veh)
        time.sleep(1)


if __name__=='__main__':
    main()
