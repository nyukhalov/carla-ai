# Alpha 2

Comfortable driving along an infinite route.

## Scene Setup

- Map: `Town06`
- Weather: default
- Vehicle: `Toyota Prius`

## Road Traffic

The ego car it the only car on the map.

## Perception

The car does not have any sensors.
The localization, planner and controller modules can use any information provided by Carla.

## Localization

The localization module uses the ground truth car pose provided by Carla.

## Planner

**Path**

- Initial pose: a random spawn point
- Goal pose: another random spawn point
- Path: shortest path, driving along the waypoints
- Obstacle avoidance: N/A

**Velocity**

- Target speed is 40 km/h
- Ignore traffic lights

## Controller

- Control type: throttle and steering
- Control constraints
    - max acceleration: 1.0 m/s2
    - max deceleration: 3.5 m/s2
    - max steering angle: 40.0 deg
    - max steering rate: 20.0 deg/s
