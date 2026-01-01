# Operating the Vehicle

Before reading this article, please make sure you followed the [Software Installation](./installation/overview.md) and built the project. The project repository has a launch file `autosdv.launch.yaml` that defines the set of nodes to be executed and assigned parameters to start the whole driving system.

## The Simple Way

The Makefile has a receipt to start the whole system.

```sh
make launch
```

## Customize the Launch

You can either modify the launch file directly located here:

```
AutoSDV/src/launcher/autosdv_launch/launch/autosdv.launch.yaml
```

or assign argument values to the launch command. For example, to use Isaac Visual SLAM for localization:

```sh
source install/setup.bash
ros2 launch autosdv_launch autosdv.launch.yaml pose_source:=isaac
```

Or to run in simulation mode without hardware:

```sh
ros2 launch autosdv_launch autosdv.launch.yaml is_simulation:=true
```

### Common Arguments

| Argument                       | Description                                                        | Default              |
|--------------------------------|--------------------------------------------------------------------|----------------------|
| `is_simulation`                | Enable simulation mode (disables PWM output to hardware)           | `false`              |
| `sensor_suite`                 | Predefined sensor suite (robin_zed, vlp32c_zed_imu, etc.)          | `vlp32c_zed_imu`     |
| `lidar_model`                  | LiDAR model (cube1, robin-w, vlp32c)                               | (from suite)         |
| `camera_model`                 | Camera model (zedxm, usb, none)                                    | (from suite)         |
| `imu_source`                   | IMU source (mpu9250, zed)                                          | (from suite)         |
| `gnss_receiver`                | GNSS receiver type (ublox, septentrio, garmin)                     | (from suite)         |
| `use_gnss`                     | Enable GNSS for outdoor operation                                  | (from suite)         |
| `use_ntrip`                    | Enable NTRIP client for RTK corrections                            | `true`               |
| `use_mapless_mode`             | Enable mapless mode for indoor operation                           | `false`              |
| `pose_source`                  | Pose estimation source (ndt, isaac)                                | `ndt`                |
| `enable_zed_object_detection`  | Enable ZED camera object detection                                 | (from suite)         |
| `launch_perception`            | Launch perception module (object detection)                        | `true`               |

For a complete list of arguments, see the [main launch file](https://github.com/NEWSLabNTU/AutoSDV/blob/main/src/launcher/autosdv_launch/launch/autosdv.launch.yaml).

## Common Operations

### Autonomous Driving

Run autonomous driving with waypoint navigation:

```sh
make run-drive
```

This command launches the full system and executes autonomous driving based on poses defined in `scripts/testing/drive/poses.json`. The vehicle will navigate through the defined waypoints.

### Visualization and Monitoring

Launch RViz for 3D visualization:

```sh
make run-rviz
```

Launch PlotJuggler for real-time data plotting:

```sh
make run-plotjuggler
```

### Manual Control

Launch keyboard-based manual control:

```sh
make run-controller
```

### Testing Control System

Test the basic control system:

```sh
make play-basic-control
```

Run predefined trajectories:

```sh
make run-straight-10m  # Drive 10m straight
make run-circle        # Drive in a circle
```

### Recording and Playback

Record sensor data during outdoor operation:

```sh
make record-outdoor
```

Play back the most recent recording:

```sh
make play-outdoor
```
