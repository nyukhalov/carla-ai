# Alpha 1

Driving around a block using the simplest approach.

## Scene Setup

- Map: `Town06`
- Weather: default
- Vehicle: `Toyota Prius`

## Road Traffic

The ego car is the only car on the map.

## Perception

The car does not have any sensors.
The localization, planner and controller modules can use any information provided by Carla.

## Localization

The localization module uses the ground truth car pose provided by Carla.

## Planner

**Path**

- Initial pose: a random spawn point
- Goal pose: N/A
- Path: driving along the waypoints, taking right turns at all forks
- Obstacle avoidance: N/A

**Velocity**

- Target speed is 20 km/h
- Ignore traffic lights

## Controller

- Control type: throttle and steering
- Acceleration and jerk constraints should be ignored
