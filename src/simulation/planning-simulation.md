# Planning Simulation

The planning simulator runs the Autoware planning and control stack without any sensor data. You set a destination on the map, and the vehicle plans a route and drives to it. This is the simplest way to explore AutoSDV and does not require a GPU or any hardware.

**Prerequisites**: Complete the [Recommended Installation](../getting-started/installation/recommended.md) before proceeding.

## Launch

```bash
just launch-sim-planning
```

This starts the Autoware planning simulator with the AutoSDV vehicle model and the COSS Park map. Open the web UI at [http://localhost:8081](http://localhost:8081).

## Set a Destination

1. In the RViz toolbar, click **2D Pose Estimate** and click on the map to set the vehicle's initial position.
2. Click **2D Goal Pose** and click a location on the map.
3. The planner computes a lane-level route and the vehicle begins driving.

## What You Are Seeing

The planning simulator exercises these Autoware modules:

- **Mission planner** — computes a global route along the lane graph
- **Behavior planner** — generates a local trajectory (lane following, lane changes, turns)
- **Motion planner** — smooths the trajectory for the controller
- **Control** — follows the trajectory using a model-predictive or PID controller

Perception and localization are not involved. The vehicle's position is provided directly by the simulator.

## The COSS Park Map

The default map is located at `data/COSS-map-planning/`. It is a point cloud map and Lanelet2 vector map of the NTU COSS Park area. To use a different map, pass the path as an argument:

```bash
play_launch launch \
    --web-addr 0.0.0.0:8081 \
    autoware_launch planning_simulator.launch.xml \
    map_path:=/path/to/your/map \
    vehicle_model:=autosdv_vehicle \
    sensor_model:=autosdv_sensor_kit
```

## Adding Obstacles

In the RViz toolbar, use the **Dummy Object** tools to place static or moving obstacles on the map. The planner will route around them or wait for them to pass.

## Common Scenarios

- **Lane driving** — set a goal along the road and observe lane-following behavior
- **U-turn** — set a goal behind the vehicle to trigger a U-turn maneuver
- **Obstacle avoidance** — place a dummy object in the lane and watch the planner reroute

## Troubleshooting

**Vehicle does not move after setting a goal**

- Make sure you set an initial pose first (2D Pose Estimate).
- Check that the goal is on a drivable lane (not on a sidewalk or off-road area).

**Map does not load**

- Verify the map directory exists: `ls data/COSS-map-planning/`
- Check that it contains both `pointcloud_map.pcd` and `lanelet2_map.osm`.

**Web UI not accessible**

- Confirm the URL is `http://localhost:8081`.
- If running on a remote machine, use the machine's IP address instead of `localhost`.

## Next Steps

- [Logging Simulation](./logging-simulation.md) — replay real sensor data through the full stack
- [Operating the Vehicle](../getting-started/usage.md) — launch parameters and tools
