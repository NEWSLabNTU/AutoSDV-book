# Logging Simulation

The logging simulation replays recorded sensor data through the full Autoware stack — localization, perception, planning, and control. This tests the same code that runs on the physical vehicle, using real LiDAR, IMU, and GNSS data.

**Prerequisites**:

- Complete the [Recommended Installation](../getting-started/installation/recommended.md)
- An NVIDIA GPU (required for perception and localization)

## Download Test Data

Download the COSS Park outdoor recording:

```bash
just download-data
```

This downloads the `outdoor_20251226_153115` rosbag (~2.8 GB) to `data/rosbags/`. The download tool (`synology-dl`) is installed automatically if not present.

See [Datasets and Rosbags](./datasets.md) for details about available recordings.

## Launch

Start the logging simulation in one terminal:

```bash
just launch-sim-logging
```

In a second terminal, play the recorded data:

```bash
ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock
```

Open the web UI at [http://localhost:8081](http://localhost:8081).

## What You Are Seeing

The rosbag provides raw sensor data as if the vehicle were driving. The Autoware stack processes it through:

- **Sensing** — LiDAR point cloud preprocessing, IMU correction
- **Localization** — NDT scan matching aligns the vehicle to the point cloud map
- **Perception** — LiDAR-based object detection (CenterPoint) and tracking
- **Planning and Control** — trajectory generation and following

In RViz you should see LiDAR point clouds, detected objects, and the vehicle's estimated position on the map.

## Monitoring Tools

Open additional terminals to monitor the system:

```bash
just tool-plotjuggler   # Plot any ROS topic over time
just tool-tui           # Terminal dashboard showing pose, speed, and component states
just tool-rviz          # Additional RViz instance
```

## Localization Modes

The default localization uses NDT scan matching. You can select a different mode:

```bash
just launch-sim-logging pose_source:=ndt     # Default: LiDAR NDT
just launch-sim-logging pose_source:=isaac    # Isaac cuVSLAM (requires visual map)
```

## Playback Options

Control the rosbag playback:

```bash
# Play at half speed
ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock -r 0.5

# Loop playback
ros2 bag play data/rosbags/outdoor_20251226_153115/ --clock -l
```

## Troubleshooting

**LiDAR points appear but localization does not initialize**

NDT requires an initial pose estimate. If automatic initialization does not trigger, set the pose manually in RViz using **2D Pose Estimate**.

**No point clouds in RViz**

- Verify the rosbag is playing: check for output in the `ros2 bag play` terminal.
- Confirm the rosbag exists: `ls data/rosbags/outdoor_20251226_153115/`

**Perception nodes fail to load (CUDA out of memory)**

The CenterPoint and occupancy grid nodes require GPU memory. If they fail, perception will not work but localization and planning are unaffected.

## Next Steps

- [COSS Park Scenario](./coss-park-scenario.md) — full automated scenario with recording
- [Datasets and Rosbags](./datasets.md) — available recordings and how to create your own
- [Sensor Integration](../guides/sensor-integration/using-sensors.md) — configure sensors for live data
