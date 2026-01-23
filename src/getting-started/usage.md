# Operating the Vehicle

Before reading this article, please make sure you followed the [Software Installation](./installation/overview.md) and built the project. The project repository has a launch file `autosdv.launch.yaml` that defines the set of nodes to be executed and assigned parameters to start the whole driving system.

## The Simple Way

The project uses [Just](https://just.systems) for command running. Run `just` to see all available commands.

```sh
just launch
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

### Visualization and Monitoring

Launch drive monitor TUI (shows pose, speed, component states):

```sh
just tool-tui
```

Launch RViz for 3D visualization:

```sh
just tool-rviz
```

Launch PlotJuggler for real-time data plotting:

```sh
just tool-plotjuggler
```

### Manual Control

Launch keyboard-based manual control:

```sh
just tool-controller
```

### Testing Control System

Test the basic control system:

```sh
just control-basic
```

Run predefined trajectories:

```sh
just control-straight  # Drive 10m straight
just control-circle    # Drive in a circle
```

### Recording and Playback

Record sensor data during outdoor operation:

```sh
just bag-record
```

Play back the most recent recording:

```sh
just bag-play
```
