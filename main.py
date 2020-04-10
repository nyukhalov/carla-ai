import random
import time
import carla

def draw_bbox(debug, obj):
    bbox_location = obj.get_transform().location
    bbox_location.z += obj.bounding_box.extent.z
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

    world = client.load_world('Town01')

    bp_lib = world.get_blueprint_library()
    cur_map = world.get_map()

    veh_bp = random.choice(bp_lib.filter('vehicle'))
    transform = random.choice(cur_map.get_spawn_points())

    veh = world.spawn_actor(veh_bp, transform)

    veh.set_autopilot(True)

    print(type(veh))
    print(dir(veh))

    debug = world.debug
    while(True):
        draw_bbox(debug, veh)
        time.sleep(1)


if __name__=='__main__':
    main()
