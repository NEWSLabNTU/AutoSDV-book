# Sensor Component

The
[`sensor_component`](https://github.com/NEWSLabNTU/F1EIGHTH/tree/main/src/autoware/sensor_component)
directory contains a collection of drivers data processros for sensors
on the F1EIGHTH vehicle. They are mostly provided by vendors and
existing open source projects.

> **Notice**
>
> The sensor component defines the collection of sensor drivers in
> Autoware. If you're looking for the composition of the sensor
> drivers, please refer to [Sensor Kit](sensor_kit.md) Chapter.

The F1EIGHTH Autoware adds the following ROS packages along with
official packages.

- ZED X Mini camera
- Blickfeld Cube1 LiDAR
- MPU9250 Nine-Axis Motion Sensor


## ZED X Mini Camera

The ROS2 package requires ZED SDK 4.2 to be installed on the
system. ZED SDK is installed by the setup script described in
[Installation Guide](installation.md). The driver package is located
at:

```
src/autoware/sensor_component/external/zed-ros2-wrapper
```

To run the standalone ZED camera driver:

```sh
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedxm
```

## Blickfeld Cube1 LiDAR


```
src/autoware/sensor_component/external/ros2_blickfeld_driver_src-v1.5.5
```

## MPU9250 Nine-Axis Motion Sensor


```
src/autoware/sensor_component/external/ros2_blickfeld_driver_src-v1.5.5
```
